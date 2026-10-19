package loftcontact

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// CertifyCapFragments maps every raw Sketch fragment to one complete
// source-line interval. Float TStart/TEnd only supply Sketch's fragment claim;
// exact order and cuts come independently from walked points, source lines,
// LineSeam rational parameters, and the paired line-line junction witnesses.
func CertifyCapFragments(budget *proofbound.WorkBudget, scene *sketch.Sketch,
	cells []*sketch.Profile, origins map[sketch.Entity]CapOrigin,
	prismLines, loftLines []sectionrecord.LineSeg, carrier int, crossing [2]int,
	cut, carrierCut [2]*big.Rat, capIndex int, level *big.Rat,
	arithmetic ...*Arithmetic,
) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	expected := [2][][]capSpan{
		make([][]capSpan, len(prismLines)),
		make([][]capSpan, len(loftLines)),
	}
	for i := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i == carrier {
			expected[0][i] = []capSpan{
				{zero, carrierCut[0]}, {carrierCut[0], carrierCut[1]}, {carrierCut[1], one},
			}
		} else {
			expected[0][i] = []capSpan{{zero, one}}
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
			expected[1][i] = []capSpan{{zero, cut[k]}, {cut[k], one}}
		} else {
			expected[1][i] = []capSpan{{zero, one}}
		}
	}
	want := make(map[fragmentKey]struct{})
	for op, source := range expected {
		for segment, spans := range source {
			for _, span := range spans {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if c.Cmp(span.lo, span.hi) >= 0 || !c.OK {
					return false, nil
				}
				want[fragmentKey{op, segment, span.lo.RatString(), span.hi.RatString()}] = struct{}{}
			}
		}
	}
	uses := make(map[fragmentKey][]fragmentUse, len(want))
	geometry := make(map[string]fragmentKey, len(want))
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
			if !ok || origin.Cap != capIndex || c.Cmp(origin.Level, level) != 0 || !c.OK || origin.Loop != -1 ||
				len(edge.Polyline) != 2 || !edge.TExact || math.IsNaN(edge.TStart) || math.IsNaN(edge.TEnd) ||
				edge.TStart < 0 || edge.TEnd > 1 || edge.TStart >= edge.TEnd {
				return false, nil
			}
			start, okStart := PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
			end, okEnd := PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
			if !okStart || !okEnd {
				return false, nil
			}
			a, okA := c.LineParameter(origin.Line, start)
			b, okB := c.LineParameter(origin.Line, end)
			if !okA || !okB || c.Cmp(a, b) == 0 || (c.Cmp(a, b) > 0) != edge.Reversed || !c.OK {
				return false, nil
			}
			if c.Cmp(a, b) > 0 {
				a, b = b, a
			}
			if edge.Partial != (c.Cmp(a, zero) != 0 || c.Cmp(b, one) != 0) || !c.OK {
				return false, nil
			}
			key := fragmentKey{origin.Operand, origin.Segment, a.RatString(), b.RatString()}
			if _, ok := want[key]; !ok {
				return false, nil
			}
			geom := fragmentGeometryKey(start, end, c)
			if !c.OK {
				return false, nil
			}
			if prior, found := geometry[geom]; found && prior != key {
				return false, nil
			}
			geometry[geom] = key
			uses[key] = append(uses[key], fragmentUse{cell: cellIndex, reversed: edge.Reversed})
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
	return c.OK, nil
}

// PartitionCap certifies one paired source cap: its junction witnesses, its
// fragments, its cell owners and its boundary spans.
func PartitionCap(ctx context.Context, budget *proofbound.WorkBudget,
	prismLines, loftLines []sectionrecord.LineSeg, carrier int, crossing [2]int, seams [2]LineSeam,
	sides []int8, walls []int8, capIndex int, level *big.Rat, prepared *PreparedCap,
	arithmetic ...*Arithmetic,
) (CapProof, bool, error) {
	c := ReplayArithmetic(arithmetic)
	if err := ctx.Err(); err != nil {
		return CapProof{}, false, err
	}
	if !c.Valid(level) {
		return CapProof{}, false, nil
	}
	if prepared == nil || prepared.Scene == nil || prepared.CapIndex != capIndex ||
		prepared.Level == nil || c.Cmp(prepared.Level, level) != 0 ||
		len(prepared.Raw) != 3 || !c.OK {
		return CapProof{}, false, nil
	}
	none := CapProof{Level: new(big.Rat).Set(level)}
	height := big.NewRat(int64(capIndex), 1)
	var points [2]Point
	var cut [2]*big.Rat
	var carrierCut [2]*big.Rat
	for k, i := range crossing {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		lineParam, carrierParam, ok := seams[k].At(height)
		if !ok || !c.Valid(lineParam) || !c.Valid(carrierParam) {
			return none, false, nil
		}
		a := c.LinePoint(loftLines[i], lineParam)
		b := c.LinePoint(prismLines[carrier], carrierParam)
		if !c.SamePoint(a, b) || !c.OK {
			return none, false, nil
		}
		points[k], cut[k], carrierCut[k] = a, lineParam, carrierParam
		fx, exactX := c.Float64(a.U)
		fy, exactY := c.Float64(a.V)
		if !exactX || !exactY || !c.OK {
			return none, false, nil
		}
		none.Junction[k] = sectionrecord.Point2{U: fx, V: fy}
	}

	var scene *sketch.Sketch
	var tags map[sketch.Entity]prismcells.Origin
	var raw []*sketch.Profile
	scene, tags, raw = prepared.Scene, prepared.Tags, prepared.Raw
	origins, mapped, err := SceneOrigins(budget, scene, tags, prismLines, loftLines, capIndex, level)
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
				if !okA || !okB || a.Operand == b.Operand ||
					a.Operand == 1 && a.Segment != crossing[k] ||
					b.Operand == 1 && b.Segment != crossing[k] ||
					a.Operand == 0 && a.Segment != carrier ||
					b.Operand == 0 && b.Segment != carrier {
					continue
				}
				witness, ok := scene.CertifyLineLineJunction(left, right)
				if !ok {
					continue
				}
				q, valid := PointOf(sectionrecord.Point2{U: witness[0], V: witness[1]})
				if valid && c.SamePoint(q, target) {
					found++
				}
			}
		}
		if found == 0 {
			return none, false, nil
		}
	}
	fragments, err := CertifyCapFragments(budget, scene, raw, origins,
		prismLines, loftLines, carrier, crossing, cut, carrierCut, capIndex, level, c)
	if err != nil || !fragments || !c.OK {
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
					none.Owners[2]++
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
		none.Owners[owner]++
		selected = append(selected, cell)
	}
	if capIndex == 0 && (none.Owners[0] == 0 || none.Owners[1] == 0 || none.Owners[2] == 0) ||
		capIndex == 1 && (none.Owners[0] != 0 || none.Owners[1] == 0 || none.Owners[2] != 0) ||
		none.Owners[0]+none.Owners[1] != len(selected) {
		return none, false, nil
	}
	// MergeWithChain is used as a transient Sketch closure check only. Its
	// candidate record is discarded; no output payload or shell is constructed.
	_, _, chain, closed, err := prismcells.MergeWithChain(budget, selected, "Loft source cap")
	if err != nil || !closed {
		return none, false, err
	}
	var spans [2][][]capSpan
	spans[0] = make([][]capSpan, len(prismLines))
	spans[1] = make([][]capSpan, len(loftLines))
	var junctionCount [2]int
	for i, edge := range chain {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		origin, ok := origins[edge.Entity]
		if !ok || origin.Cap != capIndex || c.Cmp(origin.Level, level) != 0 || !c.OK || origin.Loop != -1 ||
			len(edge.Polyline) != 2 || !edge.TExact {
			return none, false, nil
		}
		lineSet := prismLines
		op := 0
		if origin.Operand == 1 {
			lineSet, op = loftLines, 1
		}
		if origin.Segment < 0 || origin.Segment >= len(lineSet) ||
			edge.Reversed != origin.Reversed && (capIndex == 0 || op == 1) ||
			capIndex == 1 && op == 0 && edge.Reversed == origin.Reversed {
			return none, false, nil
		}
		line := origin.Line
		start, startOK := PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		end, endOK := PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !startOK || !endOK {
			return none, false, nil
		}
		a, okA := c.LineParameter(line, start)
		b, okB := c.LineParameter(line, end)
		if !okA || !okB || c.Cmp(a, b) == 0 || !c.OK {
			return none, false, nil
		}
		if c.Cmp(a, b) > 0 {
			a, b = b, a
		}
		spans[op][origin.Segment] = append(spans[op][origin.Segment], capSpan{a, b})
		if op == 0 && origin.Segment == carrier {
			none.CarrierUses++
		}
		next := chain[(i+1)%len(chain)]
		nextOrigin, ok := origins[next.Entity]
		if !ok {
			return none, false, nil
		}
		if origin.Operand != nextOrigin.Operand {
			for k, point := range points {
				if err := budget.Step(); err != nil {
					return none, false, err
				}
				if c.SamePoint(end, point) {
					junctionCount[k]++
				}
				if !c.OK {
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
		var expected []capSpan
		switch {
		case i != carrier && capIndex == 0:
			expected = []capSpan{{zero, one}}
		case i == carrier && capIndex == 0:
			expected = []capSpan{{zero, carrierCut[0]}, {carrierCut[1], one}}
		case i == carrier && capIndex == 1:
			expected = []capSpan{{carrierCut[0], carrierCut[1]}}
		}
		matched, err := spanEqual(budget, spans[0][i], expected, c)
		if err != nil || !matched {
			return none, false, err
		}
	}
	for i := range loftLines {
		if err := budget.Step(); err != nil {
			return none, false, err
		}
		var expected []capSpan
		switch walls[i] {
		case 1:
			expected = []capSpan{{zero, one}}
		case 0:
			k := 0
			if crossing[1] == i {
				k = 1
			}
			if sides[i] < 0 {
				expected = []capSpan{{zero, cut[k]}}
			} else {
				expected = []capSpan{{cut[k], one}}
			}
		}
		matched, err := spanEqual(budget, spans[1][i], expected, c)
		if err != nil || !matched {
			return none, false, err
		}
	}
	return none, c.OK, nil
}
