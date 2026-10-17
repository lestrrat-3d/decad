package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// loftSourcePartition is a transient source certificate, never a body payload.
// It describes open side patches and cap incidence, not a held output shell.
type loftSourcePartition struct {
	carrier  int
	crossing [2]int
	seams    [2]loftcontact.LineSeam
	// -1 hides the whole Loft wall, +1 exposes it, 0 splits it at a seam.
	walls    []int8
	junction [2][2]sectionrecord.Point2
}

type loftPartitionPoint struct{ u, v *big.Rat }
type loftPartitionSpan struct{ lo, hi *big.Rat }

// certifyLoftSourcePartition authenticates two live producer records and proves
// one strictly transverse, single-carrier source partition. The result cannot
// be used as a Union body without a separate output closure and two-way bound.
func certifyLoftSourcePartition(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftSourcePartition, bool, error) {
	var no loftSourcePartition
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	var pp prismPayload
	var lp loftPayload
	switch p := first.n.Payload.(type) {
	case prismPayload:
		pp = p
		var ok bool
		lp, ok = second.n.Payload.(loftPayload)
		if !ok {
			return no, false, nil
		}
	case loftPayload:
		lp = p
		var ok bool
		pp, ok = second.n.Payload.(prismPayload)
		if !ok {
			return no, false, nil
		}
	default:
		return no, false, nil
	}
	budget := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
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
	if !axis.OK || axis.Shift == nil || prismTop == nil ||
		axis.Shift.Sign() <= 0 || axis.Shift.Cmp(prismTop) >= 0 {
		return no, false, nil
	}
	prismLines, ok, err := loftPartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	bottom, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	top, ok, err := loftPartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(top) != len(bottom) {
		return no, false, err
	}
	if admitted, err := loftPartitionConvexPrism(budget, prismLines); err != nil || !admitted {
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
			a := loftPartitionSide(line, bottom[j].Start)
			b := loftPartitionSide(line, top[j].Start)
			if a == nil || b == nil || a.Sign() == 0 || b.Sign() != a.Sign() {
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
			bottom, top, [2]sectionrecord.Point2{line.Start, line.End})
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
	for cap := 0; cap < 2; cap++ {
		loftLines := bottom
		if cap == 1 {
			loftLines = top
		}
		if err := budget.Err(); err != nil {
			return no, false, err
		}
		var admitted bool
		var err error
		result.junction[cap], admitted, err = loftPartitionCap(ctx, budget, pp, prismLines, loftLines,
			carrier, crossing, seams, sides, walls, cap)
		if err != nil || !admitted {
			return no, false, err
		}
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return result, true, nil
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
	return loftPartitionPoint{u, v}, u != nil && v != nil
}

func loftPartitionCross(a, b, c loftPartitionPoint) *big.Rat {
	abu := new(big.Rat).Sub(b.u, a.u)
	abv := new(big.Rat).Sub(b.v, a.v)
	acu := new(big.Rat).Sub(c.u, a.u)
	acv := new(big.Rat).Sub(c.v, a.v)
	return new(big.Rat).Sub(new(big.Rat).Mul(abu, acv), new(big.Rat).Mul(abv, acu))
}

func loftPartitionSide(line sectionrecord.LineSeg, p sectionrecord.Point2) *big.Rat {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	c, okC := loftPartitionPointOf(p)
	if !okA || !okB || !okC {
		return nil
	}
	return loftPartitionCross(a, b, c)
}

func loftPartitionConvexPrism(budget *proofbound.WorkBudget, lines []sectionrecord.LineSeg) (bool, error) {
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
			side := loftPartitionSide(line, lines[j].Start)
			if side == nil || side.Sign() <= 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

func loftPartitionLinePoint(line sectionrecord.LineSeg, t *big.Rat) (loftPartitionPoint, bool) {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	if !okA || !okB || t == nil {
		return loftPartitionPoint{}, false
	}
	return loftPartitionPoint{
		u: new(big.Rat).Add(a.u, new(big.Rat).Mul(t, new(big.Rat).Sub(b.u, a.u))),
		v: new(big.Rat).Add(a.v, new(big.Rat).Mul(t, new(big.Rat).Sub(b.v, a.v))),
	}, true
}

func loftPartitionSamePoint(a, b loftPartitionPoint) bool {
	return a.u.Cmp(b.u) == 0 && a.v.Cmp(b.v) == 0
}

func loftPartitionParameter(line sectionrecord.LineSeg, p loftPartitionPoint) (*big.Rat, bool) {
	a, okA := loftPartitionPointOf(line.Start)
	b, okB := loftPartitionPointOf(line.End)
	if !okA || !okB {
		return nil, false
	}
	du, dv := new(big.Rat).Sub(b.u, a.u), new(big.Rat).Sub(b.v, a.v)
	var t *big.Rat
	if du.Sign() != 0 {
		t = new(big.Rat).Quo(new(big.Rat).Sub(p.u, a.u), du)
	} else if dv.Sign() != 0 {
		t = new(big.Rat).Quo(new(big.Rat).Sub(p.v, a.v), dv)
	} else {
		return nil, false
	}
	if t.Sign() < 0 || t.Cmp(big.NewRat(1, 1)) > 0 {
		return nil, false
	}
	q, ok := loftPartitionLinePoint(line, t)
	return t, ok && loftPartitionSamePoint(p, q)
}

func loftPartitionSpanEqual(budget *proofbound.WorkBudget, got, want []loftPartitionSpan) (bool, error) {
	if len(got) != len(want) {
		return false, nil
	}
	for _, spans := range [][]loftPartitionSpan{got, want} {
		for i := 1; i < len(spans); i++ {
			for j := i; j > 0; j-- {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if spans[j-1].lo.Cmp(spans[j].lo) <= 0 {
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
		if got[i].lo.Cmp(want[i].lo) != 0 || got[i].hi.Cmp(want[i].hi) != 0 {
			return false, nil
		}
	}
	return true, nil
}

func loftPartitionCap(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	prismLines, loftLines []sectionrecord.LineSeg, carrier int, crossing [2]int, seams [2]loftcontact.LineSeam,
	sides []int8, walls []int8, cap int,
) ([2]sectionrecord.Point2, bool, error) {
	var none [2]sectionrecord.Point2
	height := big.NewRat(int64(cap), 1)
	var points [2]loftPartitionPoint
	var cut [2]*big.Rat
	var carrierCut [2]*big.Rat
	for k, i := range crossing {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		lineParam, carrierParam, ok := seams[k].At(height)
		if !ok {
			return none, false, nil
		}
		a, okA := loftPartitionLinePoint(loftLines[i], lineParam)
		b, okB := loftPartitionLinePoint(prismLines[carrier], carrierParam)
		if !okA || !okB || !loftPartitionSamePoint(a, b) {
			return none, false, nil
		}
		points[k], cut[k], carrierCut[k] = a, lineParam, carrierParam
		fx, exactX := a.u.Float64()
		fy, exactY := a.v.Float64()
		if !exactX || !exactY {
			return none, false, nil
		}
		none[k] = sectionrecord.Point2{U: fx, V: fy}
	}

	loftLoop := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loftLines))}
	for i, line := range loftLines {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		loftLoop.Segments[i] = line
	}
	scene, tags, _, err := prismcells.BuildSceneRegions(budget,
		[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
		[]prismcells.SceneProfile{{Outer: loftLoop}},
		&prismcells.Reexpression{Identity: true})
	if err != nil {
		return none, false, err
	}
	raw, err := prismcells.ProfilesContext(ctx, scene.Profiles)
	if err != nil {
		return none, false, err
	}
	// The raw arrangement keeps Sketch's private paired-crossing witness.
	// SplitRuns may copy an edge snapshot, so it is used only for membership.
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
				a, okA := tags[left.Entity]
				b, okB := tags[right.Entity]
				if !okA || !okB || a.IsB == b.IsB || a.Segment != crossing[k] && a.IsB ||
					b.Segment != crossing[k] && b.IsB || a.Segment != carrier && !a.IsB ||
					b.Segment != carrier && !b.IsB {
					continue
				}
				witness, ok := scene.CertifyLineLineJunction(left, right)
				if !ok {
					continue
				}
				q, valid := loftPartitionPointOf(sectionrecord.Point2{U: witness[0], V: witness[1]})
				if valid && loftPartitionSamePoint(q, target) {
					found++
				}
			}
		}
		if found == 0 {
			return none, false, nil
		}
	}
	cells, err := prismcells.CellProfiles(ctx, budget, scene)
	if err != nil {
		return none, false, err
	}
	insideP, insideL, resolved, err := prismcells.Classify(budget, tags, cells)
	if err != nil || !resolved {
		return none, false, err
	}
	selected := make([]*sketch.Profile, 0, len(cells))
	for i, cell := range cells {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		if cap == 0 && (insideP[i] || insideL[i]) || cap == 1 && insideL[i] && !insideP[i] {
			selected = append(selected, cell)
		}
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
		origin, ok := tags[edge.Entity]
		if !ok || origin.Region != 0 || origin.Hole != -1 || len(edge.Polyline) != 2 || !edge.TExact {
			return none, false, nil
		}
		lineSet := prismLines
		op := 0
		if origin.IsB {
			lineSet, op = loftLines, 1
		}
		if origin.Segment < 0 || origin.Segment >= len(lineSet) ||
			edge.Reversed != origin.AuthoredReversed && (cap == 0 || origin.IsB) ||
			cap == 1 && !origin.IsB && edge.Reversed == origin.AuthoredReversed {
			return none, false, nil
		}
		line := lineSet[origin.Segment]
		start, startOK := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		end, endOK := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !startOK || !endOK {
			return none, false, nil
		}
		a, okA := loftPartitionParameter(line, start)
		b, okB := loftPartitionParameter(line, end)
		if !okA || !okB || a.Cmp(b) == 0 {
			return none, false, nil
		}
		if a.Cmp(b) > 0 {
			a, b = b, a
		}
		spans[op][origin.Segment] = append(spans[op][origin.Segment], loftPartitionSpan{a, b})
		next := chain[(i+1)%len(chain)]
		nextOrigin, ok := tags[next.Entity]
		if !ok {
			return none, false, nil
		}
		if origin.IsB != nextOrigin.IsB {
			for k, point := range points {
				if err := budget.Step(); err != nil {
					return none, false, err
				}
				if loftPartitionSamePoint(end, point) {
					junctionCount[k]++
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
		case i != carrier && cap == 0:
			expected = []loftPartitionSpan{{zero, one}}
		case i == carrier && cap == 0:
			expected = []loftPartitionSpan{{zero, carrierCut[0]}, {carrierCut[1], one}}
		case i == carrier && cap == 1:
			expected = []loftPartitionSpan{{carrierCut[0], carrierCut[1]}}
		}
		matched, err := loftPartitionSpanEqual(budget, spans[0][i], expected)
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
		matched, err := loftPartitionSpanEqual(budget, spans[1][i], expected)
		if err != nil || !matched {
			return none, false, err
		}
	}
	return none, true, nil
}
