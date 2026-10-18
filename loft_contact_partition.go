package decad

import (
	"context"
	"errors"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
)

// loftSourcePartition is a transient source certificate, never a body payload.
// It describes open side patches and cap incidence, not a held output shell.
type loftSourcePartition struct {
	carrier  int
	crossing [2]int
	seams    [2]loftcontact.LineSeam
	// -1 hides the whole Loft wall, +1 exposes it, 0 splits it at a seam.
	walls        []int8
	junction     [2][2]sectionrecord.Point2
	capOwners    [2][3]int // prism, Loft, prism-owned overlap cells
	capLevels    [3]*big.Rat
	topWallJoins int
	prepared     [2]loftPartitionPreparedCap
	sourceVisits uint64
}

type loftPartitionPoint struct{ u, v *big.Rat }
type loftPartitionSpan struct{ lo, hi *big.Rat }

// Each reconstructed entity gets exactly one source, direction, and cap level.
// This map is local to one Sketch arrangement and is never retained by a body.
type loftPartitionCapOrigin struct {
	operand, loop, segment int
	reversed               bool
	cap                    int
	level                  *big.Rat
	line                   sectionrecord.LineSeg
}

type loftPartitionFragmentKey struct {
	operand, segment int
	lo, hi           string
}

type loftPartitionFragmentUse struct {
	cell     int
	reversed bool
}

type loftPartitionCapProof struct {
	junction    [2]sectionrecord.Point2
	level       *big.Rat
	owners      [3]int
	carrierUses int
}

type loftPartitionPreparedCap struct {
	scene    *sketch.Sketch
	tags     map[sketch.Entity]prismcells.Origin
	raw      []*sketch.Profile
	capIndex int
	level    *big.Rat
}

// certifyLoftSourcePartition authenticates two live producer records and proves
// one strictly transverse, single-carrier source partition. The result cannot
// be used as a Union body without a separate output closure and two-way bound.
func certifyLoftSourcePartition(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, checked ...*loftVolumeArithmetic) (loftSourcePartition, bool, error) {
	return certifyLoftSourcePartitionMode(ctx, first, second, operation, false, checked...)
}

func certifyLoftSourcePartitionForDirectShell(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, checked ...*loftVolumeArithmetic) (loftSourcePartition, bool, error) {
	return certifyLoftSourcePartitionMode(ctx, first, second, operation, true, checked...)
}

func certifyLoftSourcePartitionMode(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, reportSeamSize bool,
	checked ...*loftVolumeArithmetic) (loftSourcePartition, bool, error) {
	var no loftSourcePartition
	var sourceSteps uint64
	arithmetic := loftReplayArithmetic(checked)
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	var pp prismPayload
	var lp loftPayload
	var prismBody, loftBody *Body
	switch p := first.n.Payload.(type) {
	case prismPayload:
		pp = p
		prismBody, loftBody = first, second
		var ok bool
		lp, ok = second.n.Payload.(loftPayload)
		if !ok {
			return no, false, nil
		}
	case loftPayload:
		lp = p
		prismBody, loftBody = second, first
		var ok bool
		pp, ok = second.n.Payload.(prismPayload)
		if !ok {
			return no, false, nil
		}
	default:
		return no, false, nil
	}
	wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := wall.Step(); err != nil {
				return err
			}
			sourceSteps++
			return nil
		},
		ErrFn: wall.Err,
	}
	if !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil || lp.SurfaceResult ||
		lp.ThroughBoreCut || lp.AnalyticBore || lp.ConstructionDelta != 0 ||
		len(lp.BlendSegs) != 0 || len(lp.Profile0.Holes) != 0 || len(lp.Profile1.Holes) != 0 ||
		pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.BlendSegs) != 0 ||
		pp.Xform != r3.Identity() || lp.Xform != r3.Identity() ||
		pp.Z0 != 0 || pp.Z1 <= 0 || lp.Frame0 != pp.Frame ||
		lp.Plane0 != (sectionrecord.PlaneRecord{Origin: lp.Frame0.Origin(), U: lp.Frame0.U(), V: lp.Frame0.V()}) ||
		lp.Plane1 != (sectionrecord.PlaneRecord{Origin: lp.Frame1.Origin(), U: lp.Frame1.U(), V: lp.Frame1.V()}) ||
		lp.Frame0.U() != lp.Frame1.U() || lp.Frame0.V() != lp.Frame1.V() {
		return no, false, nil
	}
	if len(lp.Alignment) > 1 {
		return no, false, nil
	}
	for _, offset := range lp.Alignment {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		if offset != 0 {
			return no, false, nil
		}
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pp.Frame, Xform: pp.Xform},
		prismplacement.Operand{Frame: lp.Frame1, Xform: lp.Xform})
	prismTop := proofarith.FloatRat(pp.Z1)
	if !axis.OK || !arithmetic.valid(axis.Shift) || !arithmetic.valid(prismTop) ||
		axis.Shift.Sign() <= 0 || arithmetic.cmp(axis.Shift, prismTop) >= 0 || !arithmetic.ok {
		return no, false, nil
	}
	prismLines, ok, err := loftPartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	bottom, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil {
		return no, false, err
	}
	if !ok {
		if !reportSeamSize {
			// Source spans alone do not prove curved cap ownership or output
			// coverage. Keep this private lane at a safe refusal for now.
			_, _, err = certifyLoftSourceSpanCaps(ctx, budget, prismBody, loftBody)
		}
		return no, false, err
	}
	top, ok, err := loftPartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(top) != len(bottom) {
		return no, false, err
	}
	if admitted, err := loftPartitionConvexPrism(budget, prismLines, arithmetic); err != nil || !admitted {
		return no, false, err
	}

	// The one-carrier subcase requires every moving Loft vertex to remain
	// strictly inside the other prism halfplanes at both ends. Signed sides
	// are affine in height, so their strict endpoint signs cover the interior.
	carrier := -1
	var crossing [2]int
	var seams [2]loftcontact.LineSeam
	var sides []int8
	for i, line := range prismLines {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		mixed := false
		for j := range bottom {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			a := arithmetic.partitionSide(line, bottom[j].Start)
			b := arithmetic.partitionSide(line, top[j].Start)
			if !arithmetic.ok || a.Sign() == 0 || b.Sign() != a.Sign() {
				return no, false, nil
			}
			if a.Sign() < 0 {
				mixed = true
			}
		}
		if !mixed {
			continue
		}
		if carrier >= 0 {
			return no, false, nil
		}
		var admitted bool
		var err error
		crossing, seams, sides, admitted, err = loftcontact.CertifyCarrierOccupancy(ctx, budget,
			bottom, top, [2]sectionrecord.Point2{line.Start, line.End}, reportSeamSize)
		if errors.Is(err, loftcontact.ErrOrderedSeamSize) {
			return no, false, errLoftDirectLimit
		}
		if err != nil || !admitted {
			return no, false, err
		}
		carrier = i
	}
	if carrier < 0 {
		return no, false, nil
	}
	walls := make([]int8, len(bottom))
	crossed := make(map[int]bool, 2)
	for _, i := range crossing {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		crossed[i] = true
	}
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		a, b := sides[i], sides[(i+1)%len(sides)]
		switch {
		case a > 0 && b > 0 && !crossed[i]:
			walls[i] = -1
		case a < 0 && b < 0 && !crossed[i]:
			walls[i] = 1
		case a != b && crossed[i]:
			walls[i] = 0
		default:
			return no, false, nil
		}
	}
	// One crossing enters the prism-interior side of the moving loop and
	// the other leaves it. This pairs the two open seam patches once each.
	if sides[crossing[0]] == sides[(crossing[0]+1)%len(sides)] ||
		sides[crossing[1]] == sides[(crossing[1]+1)%len(sides)] ||
		sides[crossing[0]] == sides[crossing[1]] {
		return no, false, nil
	}
	result := loftSourcePartition{carrier: carrier, crossing: crossing, seams: seams, walls: walls}
	result.capLevels = [3]*big.Rat{big.NewRat(0, 1), new(big.Rat).Set(axis.Shift), new(big.Rat).Set(prismTop)}
	if !reportSeamSize {
		// Reject cuts Sketch cannot witness before its paired-cap producer runs.
		for capIndex := range 2 {
			loftLines := bottom
			if capIndex == 1 {
				loftLines = top
			}
			height := big.NewRat(int64(capIndex), 1)
			for k, i := range crossing {
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				lineParam, carrierParam, ok := seams[k].At(height)
				if !ok || !arithmetic.valid(lineParam) || !arithmetic.valid(carrierParam) {
					return no, false, nil
				}
				cut := arithmetic.partitionLinePoint(loftLines[i], lineParam)
				carrierCut := arithmetic.partitionLinePoint(prismLines[carrier], carrierParam)
				if !arithmetic.partitionSamePoint(cut, carrierCut) || !arithmetic.ok {
					return no, false, nil
				}
				_, exactX := arithmetic.float64(cut.u)
				_, exactY := arithmetic.float64(cut.v)
				if !exactX || !exactY || !arithmetic.ok {
					return no, false, nil
				}
			}
		}
	}
	var prepared [2]loftPartitionPreparedCap
	var scenes [2]*sketch.Sketch
	for capIndex := range 2 {
		loftLines := bottom
		if capIndex == 1 {
			loftLines = top
		}
		loftLoop := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loftLines))}
		for i, line := range loftLines {
			if err := budget.Step(); err != nil {
				return no, false, err
			}
			loftLoop.Segments[i] = line
		}
		scene, tags, _, err := prismcells.BuildSceneRegions(budget,
			[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
			[]prismcells.SceneProfile{{Outer: loftLoop}},
			&prismcells.Reexpression{Identity: true})
		if err != nil {
			return no, false, err
		}
		if err := groundLoftPartitionScene(budget, scene); err != nil {
			return no, false, err
		}
		prepared[capIndex] = loftPartitionPreparedCap{
			scene: scene, tags: tags, capIndex: capIndex, level: result.capLevels[capIndex],
		}
		scenes[capIndex] = scene
	}
	paired, err := sketch.ProfilesTwoLineCapsBounded(ctx, scenes, func(n uint64) error {
		_, chargeErr := budget.StepN(n)
		return chargeErr
	})
	if errors.Is(err, geom.ErrUnsupportedLineScene) {
		return no, false, nil
	}
	if err != nil {
		return no, false, err
	}
	for i := range prepared {
		prepared[i].raw = paired[i]
	}
	if prepared[0].scene == prepared[1].scene {
		return no, false, nil
	}
	for capIndex := range 2 {
		loftLines := bottom
		level := result.capLevels[capIndex]
		if capIndex == 1 {
			loftLines = top
		}
		if err := budget.Err(); err != nil {
			return no, false, err
		}
		var admitted bool
		var err error
		capProof, admitted, err := loftPartitionCap(ctx, budget, prismLines, loftLines,
			carrier, crossing, seams, sides, walls, capIndex, level, &prepared[capIndex], arithmetic)
		if err != nil || !admitted {
			return no, false, err
		}
		if !arithmetic.valid(capProof.level) || arithmetic.cmp(capProof.level, level) != 0 || !arithmetic.ok {
			return no, false, nil
		}
		result.junction[capIndex], result.capOwners[capIndex] = capProof.junction, capProof.owners
		if capIndex == 0 && capProof.carrierUses != 2 || capIndex == 1 && capProof.carrierUses != 1 {
			return no, false, nil
		}
	}
	// The prism's own uncut top joins every complete source wall once. The
	// distinct level order leaves no Loft material at this final cap.
	for i, line := range prismLines {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		if line.End != prismLines[(i+1)%len(prismLines)].Start || line.TStart != 0 || line.TEnd != 1 {
			return no, false, nil
		}
		result.topWallJoins++
	}
	if result.topWallJoins != len(prismLines) ||
		arithmetic.cmp(result.capLevels[0], result.capLevels[1]) >= 0 ||
		arithmetic.cmp(result.capLevels[1], result.capLevels[2]) >= 0 || !arithmetic.ok {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	result.prepared = prepared
	result.sourceVisits = sourceSteps
	return result, true, nil
}

// groundLoftPartitionScene pins the private copied source points before the
// paired Sketch certificate reads them. It never changes a producer sketch.
func groundLoftPartitionScene(budget *proofbound.WorkBudget, scene *sketch.Sketch) error {
	for _, point := range scene.Points() {
		if err := budget.Step(); err != nil {
			return err
		}
		scene.Fix(point)
	}
	return budget.Err()
}

func loftPartitionLines(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord) ([]sectionrecord.LineSeg, bool, error) {
	if len(loop.Segments) < 3 || len(loop.Segments) > 32 {
		return nil, false, nil
	}
	lines := make([]sectionrecord.LineSeg, len(loop.Segments))
	for i, raw := range loop.Segments {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		line, ok := raw.(sectionrecord.LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return nil, false, nil
		}
		lines[i] = line
	}
	return lines, true, nil
}

func loftPartitionPointOf(p sectionrecord.Point2) (loftPartitionPoint, bool) {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	arithmetic := &loftVolumeArithmetic{ok: true}
	return loftPartitionPoint{u, v}, arithmetic.valid(u) && arithmetic.valid(v)
}

// The output replay keeps one arithmetic state through nested geometry checks.
// A failed operation must not be mistaken for an ordinary geometric miss.
func (c *loftVolumeArithmetic) partitionCross(a, b, p loftPartitionPoint) *big.Rat {
	abu, abv := c.sub(b.u, a.u), c.sub(b.v, a.v)
	apu, apv := c.sub(p.u, a.u), c.sub(p.v, a.v)
	return c.sub(c.mul(abu, apv), c.mul(abv, apu))
}

func (c *loftVolumeArithmetic) partitionSamePoint(a, b loftPartitionPoint) bool {
	same := c.cmp(a.u, b.u) == 0 && c.cmp(a.v, b.v) == 0
	return c.ok && same
}

func (c *loftVolumeArithmetic) partitionLinePoint(line sectionrecord.LineSeg, t *big.Rat) loftPartitionPoint {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	if !okA || !okB || !c.valid(t) {
		c.ok = false
		return loftPartitionPoint{}
	}
	return c.partitionInterpolate(a, b, t)
}

func (c *loftVolumeArithmetic) partitionInterpolate(a, b loftPartitionPoint, t *big.Rat) loftPartitionPoint {
	return loftPartitionPoint{
		u: c.add(a.u, c.mul(t, c.sub(b.u, a.u))),
		v: c.add(a.v, c.mul(t, c.sub(b.v, a.v))),
	}
}

func (c *loftVolumeArithmetic) partitionSide(line sectionrecord.LineSeg, p sectionrecord.Point2) *big.Rat {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	point, okP := loftPartitionPointOf(p)
	if !okA || !okB || !okP {
		c.ok = false
		return new(big.Rat)
	}
	return c.partitionCross(a, b, point)
}

func (c *loftVolumeArithmetic) partitionParameter(line sectionrecord.LineSeg, p loftPartitionPoint) (*big.Rat, bool) {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	if !okA || !okB {
		c.ok = false
		return nil, false
	}
	du, dv := c.sub(b.u, a.u), c.sub(b.v, a.v)
	var t *big.Rat
	if du.Sign() != 0 {
		t = c.quo(c.sub(p.u, a.u), du)
	} else if dv.Sign() != 0 {
		t = c.quo(c.sub(p.v, a.v), dv)
	} else {
		return nil, false
	}
	if c.cmp(t, loftOutputRat(0)) < 0 || c.cmp(t, loftOutputRat(1)) > 0 {
		return nil, false
	}
	q := c.partitionLinePoint(line, t)
	return t, c.partitionSamePoint(p, q)
}

func loftPartitionCross(a, b, c loftPartitionPoint) *big.Rat {
	abu := new(big.Rat).Sub(b.u, a.u)
	abv := new(big.Rat).Sub(b.v, a.v)
	acu := new(big.Rat).Sub(c.u, a.u)
	acv := new(big.Rat).Sub(c.v, a.v)
	return new(big.Rat).Sub(new(big.Rat).Mul(abu, acv), new(big.Rat).Mul(abv, acu))
}

func loftPartitionConvexPrism(budget *proofbound.WorkBudget, lines []sectionrecord.LineSeg,
	checked ...*loftVolumeArithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	for i, line := range lines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if line.End != lines[(i+1)%len(lines)].Start {
			return false, nil
		}
		for j := range lines {
			if j == i || j == (i+1)%len(lines) {
				continue
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			side := arithmetic.partitionSide(line, lines[j].Start)
			if !arithmetic.ok || side.Sign() <= 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

func loftPartitionSamePoint(a, b loftPartitionPoint) bool {
	arithmetic := &loftVolumeArithmetic{ok: true}
	return arithmetic.partitionSamePoint(a, b)
}

func loftPartitionParameter(line sectionrecord.LineSeg, p loftPartitionPoint) (*big.Rat, bool) {
	arithmetic := &loftVolumeArithmetic{ok: true}
	return arithmetic.partitionParameter(line, p)
}

func loftPartitionSpanEqual(budget *proofbound.WorkBudget, got, want []loftPartitionSpan,
	checked ...*loftVolumeArithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	if len(got) != len(want) {
		return false, nil
	}
	for _, spans := range [][]loftPartitionSpan{got, want} {
		for i := 1; i < len(spans); i++ {
			for j := i; j > 0; j-- {
				if err := budget.Step(); err != nil {
					return false, err
				}
				cmp := arithmetic.cmp(spans[j-1].lo, spans[j].lo)
				if !arithmetic.ok {
					return false, nil
				}
				if cmp <= 0 {
					break
				}
				spans[j-1], spans[j] = spans[j], spans[j-1]
			}
		}
	}
	for i := range got {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if arithmetic.cmp(got[i].lo, want[i].lo) != 0 ||
			arithmetic.cmp(got[i].hi, want[i].hi) != 0 || !arithmetic.ok {
			return false, nil
		}
	}
	return true, nil
}

func loftPartitionSceneOrigins(budget *proofbound.WorkBudget, scene *sketch.Sketch,
	tags map[sketch.Entity]prismcells.Origin, prismLines, loftLines []sectionrecord.LineSeg,
	capIndex int, level *big.Rat,
) (map[sketch.Entity]loftPartitionCapOrigin, bool, error) {
	if len(tags) != len(prismLines)+len(loftLines) {
		return nil, false, nil
	}
	entities := scene.Entities()
	if len(entities) != len(tags) {
		return nil, false, nil
	}
	seen := [2][]bool{make([]bool, len(prismLines)), make([]bool, len(loftLines))}
	mapped := make(map[sketch.Entity]loftPartitionCapOrigin, len(tags))
	for _, entity := range entities {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		origin, ok := tags[entity]
		if !ok || origin.Region != 0 || origin.Hole != -1 || origin.AuthoredReversed {
			return nil, false, nil
		}
		op := 0
		lines := prismLines
		if origin.IsB {
			op, lines = 1, loftLines
		}
		if origin.Segment < 0 || origin.Segment >= len(lines) || seen[op][origin.Segment] {
			return nil, false, nil
		}
		line, ok := entity.(*sketch.Line)
		if !ok || line.Start.X() != lines[origin.Segment].Start.U ||
			line.Start.Y() != lines[origin.Segment].Start.V ||
			line.End.X() != lines[origin.Segment].End.U ||
			line.End.Y() != lines[origin.Segment].End.V {
			return nil, false, nil
		}
		seen[op][origin.Segment] = true
		mapped[entity] = loftPartitionCapOrigin{
			operand: op, loop: -1, segment: origin.Segment, reversed: origin.AuthoredReversed,
			cap: capIndex, level: new(big.Rat).Set(level), line: lines[origin.Segment],
		}
	}
	for _, operand := range seen {
		for _, present := range operand {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			if !present {
				return nil, false, nil
			}
		}
	}
	return mapped, true, nil
}

func loftPartitionFragmentGeometryKey(a, b loftPartitionPoint,
	arithmetic ...*loftVolumeArithmetic) string {
	c := loftReplayArithmetic(arithmetic)
	u := c.cmp(a.u, b.u)
	if !c.ok {
		return ""
	}
	if u > 0 || u == 0 && c.cmp(a.v, b.v) > 0 {
		a, b = b, a
	}
	if !c.ok || !c.valid(a.u) || !c.valid(a.v) || !c.valid(b.u) || !c.valid(b.v) {
		return ""
	}
	return a.u.RatString() + "," + a.v.RatString() + ":" + b.u.RatString() + "," + b.v.RatString()
}

// loftPartitionCertifyFragments maps every raw Sketch fragment to one complete
// source-line interval. Float TStart/TEnd only supply Sketch's fragment claim;
// exact order and cuts come independently from walked points, source lines,
// LineSeam rational parameters, and the paired line-line junction witnesses.
func loftPartitionCertifyFragments(budget *proofbound.WorkBudget, scene *sketch.Sketch,
	cells []*sketch.Profile, origins map[sketch.Entity]loftPartitionCapOrigin,
	prismLines, loftLines []sectionrecord.LineSeg, carrier int, crossing [2]int,
	cut, carrierCut [2]*big.Rat, capIndex int, level *big.Rat,
	arithmetic ...*loftVolumeArithmetic,
) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	expected := [2][][]loftPartitionSpan{
		make([][]loftPartitionSpan, len(prismLines)),
		make([][]loftPartitionSpan, len(loftLines)),
	}
	for i := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i == carrier {
			expected[0][i] = []loftPartitionSpan{
				{zero, carrierCut[0]}, {carrierCut[0], carrierCut[1]}, {carrierCut[1], one},
			}
		} else {
			expected[0][i] = []loftPartitionSpan{{zero, one}}
		}
	}
	for i := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		k := -1
		for candidate, edge := range crossing {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if edge == i {
				k = candidate
			}
		}
		if k >= 0 {
			expected[1][i] = []loftPartitionSpan{{zero, cut[k]}, {cut[k], one}}
		} else {
			expected[1][i] = []loftPartitionSpan{{zero, one}}
		}
	}
	want := make(map[loftPartitionFragmentKey]struct{})
	for op, source := range expected {
		for segment, spans := range source {
			for _, span := range spans {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if c.cmp(span.lo, span.hi) >= 0 || !c.ok {
					return false, nil
				}
				want[loftPartitionFragmentKey{op, segment, span.lo.RatString(), span.hi.RatString()}] = struct{}{}
			}
		}
	}
	uses := make(map[loftPartitionFragmentKey][]loftPartitionFragmentUse, len(want))
	geometry := make(map[string]loftPartitionFragmentKey, len(want))
	for cellIndex, cell := range cells {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if cell == nil || cell.Sketch() != scene || cell.IsStale() || !cell.Valid ||
			len(cell.Holes) != 0 || len(cell.Outer) == 0 {
			return false, nil
		}
		for _, edge := range cell.Outer {
			if err := budget.Step(); err != nil {
				return false, err
			}
			origin, ok := origins[edge.Entity]
			if !ok || origin.cap != capIndex || c.cmp(origin.level, level) != 0 || !c.ok || origin.loop != -1 ||
				len(edge.Polyline) != 2 || !edge.TExact || math.IsNaN(edge.TStart) || math.IsNaN(edge.TEnd) ||
				edge.TStart < 0 || edge.TEnd > 1 || edge.TStart >= edge.TEnd {
				return false, nil
			}
			start, okStart := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
			end, okEnd := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
			if !okStart || !okEnd {
				return false, nil
			}
			a, okA := c.partitionParameter(origin.line, start)
			b, okB := c.partitionParameter(origin.line, end)
			if !okA || !okB || c.cmp(a, b) == 0 || (c.cmp(a, b) > 0) != edge.Reversed || !c.ok {
				return false, nil
			}
			if c.cmp(a, b) > 0 {
				a, b = b, a
			}
			if edge.Partial != (c.cmp(a, zero) != 0 || c.cmp(b, one) != 0) || !c.ok {
				return false, nil
			}
			key := loftPartitionFragmentKey{origin.operand, origin.segment, a.RatString(), b.RatString()}
			if _, ok := want[key]; !ok {
				return false, nil
			}
			geom := loftPartitionFragmentGeometryKey(start, end, c)
			if !c.ok {
				return false, nil
			}
			if prior, found := geometry[geom]; found && prior != key {
				return false, nil
			}
			geometry[geom] = key
			uses[key] = append(uses[key], loftPartitionFragmentUse{cell: cellIndex, reversed: edge.Reversed})
			if len(uses[key]) > 2 {
				return false, nil
			}
		}
	}
	for key := range want {
		if err := budget.Step(); err != nil {
			return false, err
		}
		seen := uses[key]
		if len(seen) == 0 || len(seen) > 2 || len(seen) == 2 &&
			(seen[0].cell == seen[1].cell || seen[0].reversed == seen[1].reversed) {
			return false, nil
		}
	}
	return c.ok, nil
}

func loftPartitionCap(ctx context.Context, budget *proofbound.WorkBudget,
	prismLines, loftLines []sectionrecord.LineSeg, carrier int, crossing [2]int, seams [2]loftcontact.LineSeam,
	sides []int8, walls []int8, capIndex int, level *big.Rat, prepared *loftPartitionPreparedCap,
	arithmetic ...*loftVolumeArithmetic,
) (loftPartitionCapProof, bool, error) {
	c := loftReplayArithmetic(arithmetic)
	if err := ctx.Err(); err != nil {
		return loftPartitionCapProof{}, false, err
	}
	if !c.valid(level) {
		return loftPartitionCapProof{}, false, nil
	}
	if prepared == nil || prepared.scene == nil || prepared.capIndex != capIndex ||
		prepared.level == nil || c.cmp(prepared.level, level) != 0 ||
		len(prepared.raw) != 3 || !c.ok {
		return loftPartitionCapProof{}, false, nil
	}
	none := loftPartitionCapProof{level: new(big.Rat).Set(level)}
	height := big.NewRat(int64(capIndex), 1)
	var points [2]loftPartitionPoint
	var cut [2]*big.Rat
	var carrierCut [2]*big.Rat
	for k, i := range crossing {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		lineParam, carrierParam, ok := seams[k].At(height)
		if !ok || !c.valid(lineParam) || !c.valid(carrierParam) {
			return none, false, nil
		}
		a := c.partitionLinePoint(loftLines[i], lineParam)
		b := c.partitionLinePoint(prismLines[carrier], carrierParam)
		if !c.partitionSamePoint(a, b) || !c.ok {
			return none, false, nil
		}
		points[k], cut[k], carrierCut[k] = a, lineParam, carrierParam
		fx, exactX := c.float64(a.u)
		fy, exactY := c.float64(a.v)
		if !exactX || !exactY || !c.ok {
			return none, false, nil
		}
		none.junction[k] = sectionrecord.Point2{U: fx, V: fy}
	}

	var scene *sketch.Sketch
	var tags map[sketch.Entity]prismcells.Origin
	var raw []*sketch.Profile
	scene, tags, raw = prepared.scene, prepared.tags, prepared.raw
	origins, mapped, err := loftPartitionSceneOrigins(budget, scene, tags, prismLines, loftLines, capIndex, level)
	if err != nil || !mapped {
		return none, false, err
	}
	// The raw arrangement keeps Sketch's private paired-crossing witness.
	// SplitRuns may copy an edge snapshot, so it is used only for membership.
	for _, cell := range raw {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		if cell == nil || cell.Sketch() != scene || cell.IsStale() {
			return none, false, nil
		}
	}
	for k, target := range points {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		found := 0
		for _, cell := range raw {
			if err := budget.Step(); err != nil {
				return none, false, err
			}
			for i, left := range cell.Outer {
				if err := budget.Step(); err != nil {
					return none, false, err
				}
				right := cell.Outer[(i+1)%len(cell.Outer)]
				a, okA := origins[left.Entity]
				b, okB := origins[right.Entity]
				if !okA || !okB || a.operand == b.operand ||
					a.operand == 1 && a.segment != crossing[k] ||
					b.operand == 1 && b.segment != crossing[k] ||
					a.operand == 0 && a.segment != carrier ||
					b.operand == 0 && b.segment != carrier {
					continue
				}
				witness, ok := scene.CertifyLineLineJunction(left, right)
				if !ok {
					continue
				}
				q, valid := loftPartitionPointOf(sectionrecord.Point2{U: witness[0], V: witness[1]})
				if valid && c.partitionSamePoint(q, target) {
					found++
				}
			}
		}
		if found == 0 {
			return none, false, nil
		}
	}
	fragments, err := loftPartitionCertifyFragments(budget, scene, raw, origins,
		prismLines, loftLines, carrier, crossing, cut, carrierCut, capIndex, level, c)
	if err != nil || !fragments || !c.ok {
		return none, false, err
	}
	cells := raw
	insideP, insideL, resolved, err := prismcells.Classify(budget, tags, cells)
	if err != nil || !resolved {
		return none, false, err
	}
	selected := make([]*sketch.Profile, 0, len(cells))
	selectedOnce := make(map[*sketch.Profile]struct{}, len(cells))
	for i, cell := range cells {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		owner := -1
		if capIndex == 0 {
			if insideP[i] {
				owner = 0 // P owns the nonempty lower overlap once.
				if insideL[i] {
					none.owners[2]++
				}
			} else if insideL[i] {
				owner = 1
			}
		} else if insideL[i] && !insideP[i] {
			owner = 1 // P continues above the Loft cap over the overlap.
		}
		if owner < 0 {
			continue
		}
		if _, duplicate := selectedOnce[cell]; duplicate {
			return none, false, nil
		}
		selectedOnce[cell] = struct{}{}
		none.owners[owner]++
		selected = append(selected, cell)
	}
	if capIndex == 0 && (none.owners[0] == 0 || none.owners[1] == 0 || none.owners[2] == 0) ||
		capIndex == 1 && (none.owners[0] != 0 || none.owners[1] == 0 || none.owners[2] != 0) ||
		none.owners[0]+none.owners[1] != len(selected) {
		return none, false, nil
	}
	// MergeWithChain is used as a transient Sketch closure check only. Its
	// candidate record is discarded; no output payload or shell is constructed.
	_, _, chain, closed, err := prismcells.MergeWithChain(budget, selected, "Loft source cap")
	if err != nil || !closed {
		return none, false, err
	}
	var spans [2][][]loftPartitionSpan
	spans[0] = make([][]loftPartitionSpan, len(prismLines))
	spans[1] = make([][]loftPartitionSpan, len(loftLines))
	var junctionCount [2]int
	for i, edge := range chain {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		origin, ok := origins[edge.Entity]
		if !ok || origin.cap != capIndex || c.cmp(origin.level, level) != 0 || !c.ok || origin.loop != -1 ||
			len(edge.Polyline) != 2 || !edge.TExact {
			return none, false, nil
		}
		lineSet := prismLines
		op := 0
		if origin.operand == 1 {
			lineSet, op = loftLines, 1
		}
		if origin.segment < 0 || origin.segment >= len(lineSet) ||
			edge.Reversed != origin.reversed && (capIndex == 0 || op == 1) ||
			capIndex == 1 && op == 0 && edge.Reversed == origin.reversed {
			return none, false, nil
		}
		line := origin.line
		start, startOK := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		end, endOK := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !startOK || !endOK {
			return none, false, nil
		}
		a, okA := c.partitionParameter(line, start)
		b, okB := c.partitionParameter(line, end)
		if !okA || !okB || c.cmp(a, b) == 0 || !c.ok {
			return none, false, nil
		}
		if c.cmp(a, b) > 0 {
			a, b = b, a
		}
		spans[op][origin.segment] = append(spans[op][origin.segment], loftPartitionSpan{a, b})
		if op == 0 && origin.segment == carrier {
			none.carrierUses++
		}
		next := chain[(i+1)%len(chain)]
		nextOrigin, ok := origins[next.Entity]
		if !ok {
			return none, false, nil
		}
		if origin.operand != nextOrigin.operand {
			for k, point := range points {
				if err := budget.Step(); err != nil {
					return none, false, err
				}
				if c.partitionSamePoint(end, point) {
					junctionCount[k]++
				}
				if !c.ok {
					return none, false, nil
				}
			}
		}
	}
	if junctionCount != [2]int{1, 1} {
		return none, false, nil
	}
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	// These exact intervals are the symbolic cap/side incidence list. The
	// carrier's lower complement meets its exposed lower wall; its upper
	// interior span meets the prism wall above the Loft's upper cap.
	for i := range prismLines {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		var expected []loftPartitionSpan
		switch {
		case i != carrier && capIndex == 0:
			expected = []loftPartitionSpan{{zero, one}}
		case i == carrier && capIndex == 0:
			expected = []loftPartitionSpan{{zero, carrierCut[0]}, {carrierCut[1], one}}
		case i == carrier && capIndex == 1:
			expected = []loftPartitionSpan{{carrierCut[0], carrierCut[1]}}
		}
		matched, err := loftPartitionSpanEqual(budget, spans[0][i], expected, c)
		if err != nil || !matched {
			return none, false, err
		}
	}
	for i := range loftLines {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		var expected []loftPartitionSpan
		switch walls[i] {
		case 1:
			expected = []loftPartitionSpan{{zero, one}}
		case 0:
			k := 0
			if crossing[1] == i {
				k = 1
			}
			if sides[i] < 0 {
				expected = []loftPartitionSpan{{zero, cut[k]}}
			} else {
				expected = []loftPartitionSpan{{cut[k], one}}
			}
		}
		matched, err := loftPartitionSpanEqual(budget, spans[1][i], expected, c)
		if err != nil || !matched {
			return none, false, err
		}
	}
	return none, c.ok, nil
}
