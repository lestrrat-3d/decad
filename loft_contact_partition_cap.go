package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

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
