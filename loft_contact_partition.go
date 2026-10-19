package decad

import (
	"context"
	"errors"
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
	operation *proofbound.WorkBudget, checked ...*loftcontact.Arithmetic) (loftSourcePartition, bool, error) {
	return certifyLoftSourcePartitionMode(ctx, first, second, operation, false, checked...)
}

func certifyLoftSourcePartitionForDirectShell(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, checked ...*loftcontact.Arithmetic) (loftSourcePartition, bool, error) {
	return certifyLoftSourcePartitionMode(ctx, first, second, operation, true, checked...)
}

func certifyLoftSourcePartitionMode(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, reportSeamSize bool,
	checked ...*loftcontact.Arithmetic) (loftSourcePartition, bool, error) {
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
	if !axis.OK || !arithmetic.Valid(axis.Shift) || !arithmetic.Valid(prismTop) ||
		axis.Shift.Sign() <= 0 || arithmetic.Cmp(axis.Shift, prismTop) >= 0 || !arithmetic.OK {
		return no, false, nil
	}
	prismLines, ok, err := loftPartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		if err == nil && !ok && !reportSeamSize && len(pp.Profile.Outer.Segments) == 1 {
			if _, circle := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg); circle {
				// Read the original disc source within this same wall budget. A
				// circle/Loft cap intersection and output shell remain unproved.
				_, _, err = certifyLoftSourceSpanCaps(ctx, budget, prismBody, loftBody)
			}
		}
		return no, false, err
	}
	bottom, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil {
		return no, false, err
	}
	if !ok {
		if !reportSeamSize {
			// Source spans and ordered cap cells do not prove output coverage
			// or occupied volume. Keep this private lane at a safe refusal.
			source, certified, sourceErr := certifyLoftSourceSpanCaps(
				ctx, budget, prismBody, loftBody)
			if sourceErr != nil {
				return no, false, sourceErr
			}
			if certified {
				_, _, err = certifyLoftCapSourceOrder(ctx, budget, source)
			}
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
			a := arithmetic.LineSide(line, bottom[j].Start)
			b := arithmetic.LineSide(line, top[j].Start)
			if !arithmetic.OK || a.Sign() == 0 || b.Sign() != a.Sign() {
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
				if !ok || !arithmetic.Valid(lineParam) || !arithmetic.Valid(carrierParam) {
					return no, false, nil
				}
				cut := arithmetic.LinePoint(loftLines[i], lineParam)
				carrierCut := arithmetic.LinePoint(prismLines[carrier], carrierParam)
				if !arithmetic.SamePoint(cut, carrierCut) || !arithmetic.OK {
					return no, false, nil
				}
				_, exactX := arithmetic.Float64(cut.U)
				_, exactY := arithmetic.Float64(cut.V)
				if !exactX || !exactY || !arithmetic.OK {
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
		if !arithmetic.Valid(capProof.level) || arithmetic.Cmp(capProof.level, level) != 0 || !arithmetic.OK {
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
		arithmetic.Cmp(result.capLevels[0], result.capLevels[1]) >= 0 ||
		arithmetic.Cmp(result.capLevels[1], result.capLevels[2]) >= 0 || !arithmetic.OK {
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

func loftPartitionConvexPrism(budget *proofbound.WorkBudget, lines []sectionrecord.LineSeg,
	checked ...*loftcontact.Arithmetic) (bool, error) {
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
			side := arithmetic.LineSide(line, lines[j].Start)
			if !arithmetic.OK || side.Sign() <= 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

func loftPartitionSamePoint(a, b loftcontact.Point) bool {
	arithmetic := &loftcontact.Arithmetic{OK: true}
	return arithmetic.SamePoint(a, b)
}

func loftPartitionParameter(line sectionrecord.LineSeg, p loftcontact.Point) (*big.Rat, bool) {
	arithmetic := &loftcontact.Arithmetic{OK: true}
	return arithmetic.LineParameter(line, p)
}

func loftPartitionSpanEqual(budget *proofbound.WorkBudget, got, want []loftPartitionSpan,
	checked ...*loftcontact.Arithmetic) (bool, error) {
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
				cmp := arithmetic.Cmp(spans[j-1].lo, spans[j].lo)
				if !arithmetic.OK {
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
		if arithmetic.Cmp(got[i].lo, want[i].lo) != 0 ||
			arithmetic.Cmp(got[i].hi, want[i].hi) != 0 || !arithmetic.OK {
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

func loftPartitionFragmentGeometryKey(a, b loftcontact.Point,
	arithmetic ...*loftcontact.Arithmetic) string {
	c := loftReplayArithmetic(arithmetic)
	u := c.Cmp(a.U, b.U)
	if !c.OK {
		return ""
	}
	if u > 0 || u == 0 && c.Cmp(a.V, b.V) > 0 {
		a, b = b, a
	}
	if !c.OK || !c.Valid(a.U) || !c.Valid(a.V) || !c.Valid(b.U) || !c.Valid(b.V) {
		return ""
	}
	return a.U.RatString() + "," + a.V.RatString() + ":" + b.U.RatString() + "," + b.V.RatString()
}
