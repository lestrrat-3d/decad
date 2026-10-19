package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// OutputCap adds the exact triangles of one output cap to shell in the prism
// frame. Cap 2 is the prism top. Caps 0 and 1 replay the source cap proof
// against these live records, then tile the selected Sketch cells.
func OutputCap(ctx context.Context, budget *proofbound.WorkBudget, frame r3.Frame,
	prismLines, bottom, top []sectionrecord.LineSeg, source SourcePartition,
	shell *OutputShell, capIndex int, checked ...*Arithmetic) (bool, error) {
	arithmetic := ReplayArithmetic(checked)
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	if capIndex == 2 {
		polygon := make([]Point, 0, len(prismLines)+2)
		for i, line := range prismLines {
			if err := budget.Step(); err != nil {
				return false, err
			}
			p, ok := PointOf(line.Start)
			if !ok {
				return false, nil
			}
			polygon = append(polygon, p)
			if i == source.Carrier {
				for _, seam := range source.Seams {
					_, t, ok := seam.At(big.NewRat(1, 1))
					if !ok {
						return false, nil
					}
					p = arithmetic.LinePoint(line, t)
					if !arithmetic.OK {
						return false, nil
					}
					polygon = append(polygon, p)
				}
			}
		}
		return TileCap(ctx, budget, frame, shell, polygon, source.CapLevels[2],
			OutputFace{Kind: 0, Owner: 0, Level: 2, Cell: 0}, false, arithmetic)
	}
	loftLines := bottom
	if capIndex == 1 {
		loftLines = top
	}
	level := source.CapLevels[capIndex]
	prepared := &source.Prepared[capIndex]
	if prepared.Scene == nil || prepared.Scene == source.Prepared[1-capIndex].Scene {
		return false, nil
	}
	// Re-run the accepted cap source proof against this call's live records.
	var cuts, carrierCuts [2]*big.Rat
	for k, seam := range source.Seams {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var ok bool
		cuts[k], carrierCuts[k], ok = seam.At(big.NewRat(int64(capIndex), 1))
		if !ok || !arithmetic.Valid(cuts[k]) || !arithmetic.Valid(carrierCuts[k]) {
			return false, nil
		}
	}
	var sides []int8
	for _, line := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		side := arithmetic.LineSide(prismLines[source.Carrier], line.Start)
		if !arithmetic.OK || side.Sign() == 0 {
			return false, nil
		}
		sides = append(sides, int8(side.Sign()))
	}
	capProof, ok, err := PartitionCap(ctx, budget, prismLines, loftLines,
		source.Carrier, source.Crossing, source.Seams, sides, source.Walls, capIndex, level,
		prepared, arithmetic)
	if err != nil || !ok || !arithmetic.OK || capProof.Owners != source.CapOwners[capIndex] ||
		capProof.Junction != source.Junction[capIndex] {
		return false, err
	}
	scene, tags, cells := prepared.Scene, prepared.Tags, prepared.Raw
	origins, mapped, err := SceneOrigins(budget, scene, tags,
		prismLines, loftLines, capIndex, level)
	if err != nil || !mapped {
		return false, err
	}
	mapped, err = CertifyCapFragments(budget, scene, cells, origins,
		prismLines, loftLines, source.Carrier, source.Crossing, cuts, carrierCuts, capIndex, level, arithmetic)
	if err != nil || !mapped {
		return false, err
	}
	insideP, insideL, resolved, err := prismcells.Classify(budget, tags, cells)
	if err != nil || !resolved {
		return false, err
	}
	var count [3]int
	selected := 0
	selectedCells := make([]*sketch.Profile, 0, 8)
	for i, cell := range cells {
		if err := budget.Step(); err != nil {
			return false, err
		}
		owner := -1
		if capIndex == 0 {
			if insideP[i] {
				owner = 0
				if insideL[i] {
					count[2]++
				}
			} else if insideL[i] {
				owner = 1
			}
		} else if insideL[i] && !insideP[i] {
			owner = 1
		}
		if owner < 0 {
			continue
		}
		selected++
		if selected > 8 || len(cell.Outer) < 3 || len(cell.Outer) > 16 || len(cell.Holes) != 0 {
			return false, nil
		}
		count[owner]++
		selectedCells = append(selectedCells, cell)
		polygon, valid, err := cellPolygon(budget, cell, origins)
		if err != nil || !valid {
			return false, err
		}
		valid, err = TileCap(ctx, budget, frame, shell, polygon, level,
			OutputFace{Kind: 0, Owner: owner, Level: capIndex, Cell: i}, capIndex == 0, arithmetic)
		if err != nil || !valid {
			return false, err
		}
	}
	if selected == 0 || count != source.CapOwners[capIndex] {
		return false, nil
	}
	return capBoundary(budget, selectedCells, origins, prismLines, loftLines,
		source, cuts, carrierCuts, capIndex, arithmetic)
}

func capBoundary(budget *proofbound.WorkBudget, selected []*sketch.Profile,
	origins map[sketch.Entity]CapOrigin, prismLines, loftLines []sectionrecord.LineSeg,
	source SourcePartition, cut, carrierCut [2]*big.Rat, capIndex int,
	arithmetic ...*Arithmetic) (bool, error) {
	c := ReplayArithmetic(arithmetic)
	_, _, chain, closed, err := prismcells.MergeWithChain(budget, selected, "Loft output cap")
	if err != nil || !closed {
		return false, err
	}
	spans := [2][][]capSpan{
		make([][]capSpan, len(prismLines)),
		make([][]capSpan, len(loftLines)),
	}
	for _, edge := range chain {
		if err := budget.Step(); err != nil {
			return false, err
		}
		origin, found := origins[edge.Entity]
		if !found || len(edge.Polyline) != 2 || !edge.TExact ||
			origin.Cap != capIndex || c.Cmp(origin.Level, source.CapLevels[capIndex]) != 0 || !c.OK {
			return false, nil
		}
		a, okA := PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return false, nil
		}
		lo, okLo := c.LineParameter(origin.Line, a)
		hi, okHi := c.LineParameter(origin.Line, b)
		if !okLo || !okHi || c.Cmp(lo, hi) == 0 || !c.OK {
			return false, nil
		}
		if c.Cmp(lo, hi) > 0 {
			lo, hi = hi, lo
		}
		spans[origin.Operand][origin.Segment] = append(spans[origin.Operand][origin.Segment],
			capSpan{lo, hi})
	}
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	for i := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var want []capSpan
		switch {
		case i != source.Carrier && capIndex == 0:
			want = []capSpan{{zero, one}}
		case i == source.Carrier && capIndex == 0:
			want = []capSpan{{zero, carrierCut[0]}, {carrierCut[1], one}}
		case i == source.Carrier && capIndex == 1:
			want = []capSpan{{carrierCut[0], carrierCut[1]}}
		}
		matched, err := spanEqual(budget, spans[0][i], want)
		if err != nil || !matched {
			return false, err
		}
	}
	for i := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var want []capSpan
		switch source.Walls[i] {
		case 1:
			want = []capSpan{{zero, one}}
		case 0:
			k := 0
			if source.Crossing[1] == i {
				k = 1
			}
			side := c.LineSide(prismLines[source.Carrier], loftLines[i].Start)
			if !c.OK || side.Sign() == 0 {
				return false, nil
			}
			if side.Sign() < 0 {
				want = []capSpan{{zero, cut[k]}}
			} else {
				want = []capSpan{{cut[k], one}}
			}
		}
		matched, err := spanEqual(budget, spans[1][i], want)
		if err != nil || !matched {
			return false, err
		}
	}
	return c.OK, nil
}

func cellPolygon(budget *proofbound.WorkBudget, cell *sketch.Profile,
	origins map[sketch.Entity]CapOrigin) ([]Point, bool, error) {
	polygon := make([]Point, 0, len(cell.Outer))
	arithmetic := &Arithmetic{OK: true}
	for i, edge := range cell.Outer {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		origin, found := origins[edge.Entity]
		if !found || len(edge.Polyline) != 2 || !edge.TExact {
			return nil, false, nil
		}
		a, okA := PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return nil, false, nil
		}
		_, srcA := arithmetic.LineParameter(origin.Line, a)
		_, srcB := arithmetic.LineParameter(origin.Line, b)
		next := cell.Outer[(i+1)%len(cell.Outer)]
		if !srcA || !srcB || !arithmetic.OK || len(next.Polyline) != 2 {
			return nil, false, nil
		}
		start, ok := PointOf(sectionrecord.Point2{
			U: next.Polyline[0][0], V: next.Polyline[0][1],
		})
		if !ok || !arithmetic.SamePoint(b, start) {
			return nil, false, nil
		}
		polygon = append(polygon, a)
	}
	return polygon, true, nil
}

// TileCap ear-clips polygon at height level, certifies the candidate tiling
// exactly, and adds its triangles to shell, reversed when downward.
func TileCap(ctx context.Context, budget *proofbound.WorkBudget, frame r3.Frame,
	shell *OutputShell, polygon []Point, level *big.Rat,
	face OutputFace, downward bool, arithmetic ...*Arithmetic) (bool, error) {
	n := len(polygon)
	if n < 3 || n > 16 || len(shell.Triangles)+n-2 > 512 ||
		len(shell.Vertices)+n > 512 {
		return false, nil
	}
	c := ReplayArithmetic(arithmetic)
	if !c.Valid(level) {
		return false, nil
	}
	for _, point := range polygon {
		if !c.Valid(point.U) || !c.Valid(point.V) {
			return false, nil
		}
	}
	// Hole-free EarClip attempts at most k ears of at most k vertex scans
	// at each polygon size k. Reserve its work in the shared output counter.
	var reserve uint64
	for k := 3; k <= n; k++ {
		reserve += uint64(k * k)
	}
	if _, err := budget.StepN(reserve); err != nil {
		return false, err
	}
	pts := make([]triangulation.Point2, n)
	idx := make([]int, n)
	for i, p := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		pts[i].U, _ = c.Float64(p.U)
		pts[i].V, _ = c.Float64(p.V)
		if !c.OK || proofbound.IsNonFinite(pts[i].U) || proofbound.IsNonFinite(pts[i].V) {
			return false, nil
		}
		idx[i] = i
	}
	candidate, err := triangulation.Triangulate(ctx, pts, [][]int{idx})
	if err != nil || len(candidate) != n-2 {
		return false, err
	}
	valid, err := CertifyCapTiling(budget, polygon, candidate, c)
	if err != nil || !valid || !c.OK {
		return false, err
	}
	for _, t := range candidate {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if downward {
			t[1], t[2] = t[2], t[1]
		}
		points := [3]OutputPoint{
			OutputP2(polygon[t[0]], level),
			OutputP2(polygon[t[1]], level),
			OutputP2(polygon[t[2]], level),
		}
		if !shell.Triangle(frame, points, face) {
			return false, nil
		}
	}
	return true, nil
}
