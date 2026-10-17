package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/sketch"
)

func loftOutputCap(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	prismLines, bottom, top []sectionrecord.LineSeg, source loftSourcePartition,
	shell *loftOutputShell, capIndex int) (bool, error) {
	if capIndex == 2 {
		polygon := make([]loftPartitionPoint, 0, len(prismLines)+2)
		for i, line := range prismLines {
			if err := budget.Step(); err != nil {
				return false, err
			}
			p, ok := loftPartitionPointOf(line.Start)
			if !ok {
				return false, nil
			}
			polygon = append(polygon, p)
			if i == source.carrier {
				for _, seam := range source.seams {
					_, t, ok := seam.At(loftOutputRat(1))
					if !ok {
						return false, nil
					}
					p, ok = loftPartitionLinePoint(line, t)
					if !ok {
						return false, nil
					}
					polygon = append(polygon, p)
				}
			}
		}
		return loftOutputTileCap(ctx, budget, pp, shell, polygon, source.capLevels[2],
			loftOutputFace{kind: 0, owner: 0, level: 2, cell: 0}, false)
	}
	loftLines := bottom
	if capIndex == 1 {
		loftLines = top
	}
	level := source.capLevels[capIndex]
	// Re-run the accepted cap source proof against this call's live records.
	var cuts, carrierCuts [2]*big.Rat
	for k, seam := range source.seams {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var ok bool
		cuts[k], carrierCuts[k], ok = seam.At(loftOutputRat(int64(capIndex)))
		if !ok {
			return false, nil
		}
	}
	var sides []int8
	for _, line := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		side := loftPartitionSide(prismLines[source.carrier], line.Start)
		if side == nil || side.Sign() == 0 {
			return false, nil
		}
		sides = append(sides, int8(side.Sign()))
	}
	capProof, ok, err := loftPartitionCap(ctx, budget, pp, prismLines, loftLines,
		source.carrier, source.crossing, source.seams, sides, source.walls, capIndex, level)
	if err != nil || !ok || capProof.owners != source.capOwners[capIndex] ||
		capProof.junction != source.junction[capIndex] {
		return false, err
	}
	loftLoop := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loftLines))}
	for i, line := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		loftLoop.Segments[i] = line
	}
	scene, tags, _, err := prismcells.BuildSceneRegions(budget,
		[]prismcells.SceneProfile{{Outer: pp.Profile.Outer}},
		[]prismcells.SceneProfile{{Outer: loftLoop}},
		&prismcells.Reexpression{Identity: true})
	if err != nil {
		return false, err
	}
	origins, mapped, err := loftPartitionSceneOrigins(budget, scene, tags,
		prismLines, loftLines, capIndex, level)
	if err != nil || !mapped {
		return false, err
	}
	cells, err := prismcells.ProfilesContext(ctx, scene.Profiles)
	if err != nil {
		return false, err
	}
	mapped, err = loftPartitionCertifyFragments(budget, scene, cells, origins,
		prismLines, loftLines, source.carrier, source.crossing, cuts, carrierCuts, capIndex, level)
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
		polygon, valid, err := loftOutputCellPolygon(budget, cell, origins)
		if err != nil || !valid {
			return false, err
		}
		valid, err = loftOutputTileCap(ctx, budget, pp, shell, polygon, level,
			loftOutputFace{kind: 0, owner: owner, level: capIndex, cell: i}, capIndex == 0)
		if err != nil || !valid {
			return false, err
		}
	}
	if selected == 0 || count != source.capOwners[capIndex] {
		return false, nil
	}
	return loftOutputCapBoundary(budget, selectedCells, origins, prismLines, loftLines,
		source, cuts, carrierCuts, capIndex)
}

func loftOutputCapBoundary(budget *proofbound.WorkBudget, selected []*sketch.Profile,
	origins map[sketch.Entity]loftPartitionCapOrigin, prismLines, loftLines []sectionrecord.LineSeg,
	source loftSourcePartition, cut, carrierCut [2]*big.Rat, capIndex int) (bool, error) {
	_, _, chain, closed, err := prismcells.MergeWithChain(budget, selected, "Loft output cap")
	if err != nil || !closed {
		return false, err
	}
	spans := [2][][]loftPartitionSpan{
		make([][]loftPartitionSpan, len(prismLines)),
		make([][]loftPartitionSpan, len(loftLines)),
	}
	for _, edge := range chain {
		if err := budget.Step(); err != nil {
			return false, err
		}
		origin, found := origins[edge.Entity]
		if !found || len(edge.Polyline) != 2 || !edge.TExact ||
			origin.cap != capIndex || origin.level.Cmp(source.capLevels[capIndex]) != 0 {
			return false, nil
		}
		a, okA := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return false, nil
		}
		lo, okLo := loftPartitionParameter(origin.line, a)
		hi, okHi := loftPartitionParameter(origin.line, b)
		if !okLo || !okHi || lo.Cmp(hi) == 0 {
			return false, nil
		}
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		spans[origin.operand][origin.segment] = append(spans[origin.operand][origin.segment],
			loftPartitionSpan{lo, hi})
	}
	zero, one := loftOutputRat(0), loftOutputRat(1)
	for i := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var want []loftPartitionSpan
		switch {
		case i != source.carrier && capIndex == 0:
			want = []loftPartitionSpan{{zero, one}}
		case i == source.carrier && capIndex == 0:
			want = []loftPartitionSpan{{zero, carrierCut[0]}, {carrierCut[1], one}}
		case i == source.carrier && capIndex == 1:
			want = []loftPartitionSpan{{carrierCut[0], carrierCut[1]}}
		}
		matched, err := loftPartitionSpanEqual(budget, spans[0][i], want)
		if err != nil || !matched {
			return false, err
		}
	}
	for i := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var want []loftPartitionSpan
		switch source.walls[i] {
		case 1:
			want = []loftPartitionSpan{{zero, one}}
		case 0:
			k := 0
			if source.crossing[1] == i {
				k = 1
			}
			side := loftPartitionSide(prismLines[source.carrier], loftLines[i].Start)
			if side == nil || side.Sign() == 0 {
				return false, nil
			}
			if side.Sign() < 0 {
				want = []loftPartitionSpan{{zero, cut[k]}}
			} else {
				want = []loftPartitionSpan{{cut[k], one}}
			}
		}
		matched, err := loftPartitionSpanEqual(budget, spans[1][i], want)
		if err != nil || !matched {
			return false, err
		}
	}
	return true, nil
}

func loftOutputCellPolygon(budget *proofbound.WorkBudget, cell *sketch.Profile,
	origins map[sketch.Entity]loftPartitionCapOrigin) ([]loftPartitionPoint, bool, error) {
	polygon := make([]loftPartitionPoint, 0, len(cell.Outer))
	for i, edge := range cell.Outer {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		origin, found := origins[edge.Entity]
		if !found || len(edge.Polyline) != 2 || !edge.TExact {
			return nil, false, nil
		}
		a, okA := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return nil, false, nil
		}
		_, srcA := loftPartitionParameter(origin.line, a)
		_, srcB := loftPartitionParameter(origin.line, b)
		if !srcA || !srcB || !loftPartitionSamePoint(b, polygonPointAt(cell.Outer, i+1)) {
			return nil, false, nil
		}
		polygon = append(polygon, a)
	}
	return polygon, true, nil
}

func polygonPointAt(edges []sketch.BoundaryEdge, i int) loftPartitionPoint {
	data := edges[i%len(edges)].Polyline[0]
	p, _ := loftPartitionPointOf(sectionrecord.Point2{U: data[0], V: data[1]})
	return p
}

func loftOutputTileCap(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	shell *loftOutputShell, polygon []loftPartitionPoint, level *big.Rat,
	face loftOutputFace, downward bool) (bool, error) {
	n := len(polygon)
	if n < 3 || n > 16 || len(shell.triangles)+n-2 > 512 ||
		len(shell.vertices)+n > 512 {
		return false, nil
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
		pts[i].U, _ = p.u.Float64()
		pts[i].V, _ = p.v.Float64()
		idx[i] = i
	}
	candidate, err := triangulation.Triangulate(ctx, pts, [][]int{idx})
	if err != nil || len(candidate) != n-2 {
		return false, err
	}
	valid, err := loftOutputCertifyCapTiling(budget, polygon, candidate)
	if err != nil || !valid {
		return false, err
	}
	for _, t := range candidate {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if downward {
			t[1], t[2] = t[2], t[1]
		}
		points := [3]loftOutputPoint{
			loftOutputP2(polygon[t[0]], level),
			loftOutputP2(polygon[t[1]], level),
			loftOutputP2(polygon[t[2]], level),
		}
		if !shell.triangle(pp.Frame, points, face) {
			return false, nil
		}
	}
	return true, nil
}

func loftOutputCertifyCapTiling(budget *proofbound.WorkBudget, polygon []loftPartitionPoint,
	tris [][3]int) (bool, error) {
	area := loftOutputPolygonArea(polygon)
	if area.Sign() <= 0 {
		return false, nil
	}
	sum := new(big.Rat)
	edges := make(map[string]int)
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for _, vertex := range tri {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if vertex < 0 || vertex >= len(polygon) {
				return false, nil
			}
		}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return false, nil
		}
		p := [3]loftPartitionPoint{polygon[tri[0]], polygon[tri[1]], polygon[tri[2]]}
		double := loftPartitionCross(p[0], p[1], p[2])
		if double.Sign() <= 0 {
			return false, nil
		}
		sum.Add(sum, double)
		inside, err := loftOutputTriangleInside(budget, polygon, p)
		if err != nil || !inside {
			return false, err
		}
		for j := range 3 {
			key := loftPartitionFragmentGeometryKey(p[j], p[(j+1)%3])
			dir := 1
			if loftOutputP2After(p[j], p[(j+1)%3]) {
				dir = -1
			}
			edges[key] += dir
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := [3]loftPartitionPoint{polygon[tris[j][0]], polygon[tris[j][1]], polygon[tris[j][2]]}
			overlap, err := loftOutputTrianglesOverlap(budget, p, q)
			if err != nil {
				return false, err
			}
			if overlap {
				return false, nil
			}
		}
	}
	if sum.Cmp(new(big.Rat).Mul(area, loftOutputRat(2))) != 0 {
		return false, nil
	}
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		b := polygon[(i+1)%len(polygon)]
		key := loftPartitionFragmentGeometryKey(a, b)
		dir := 1
		if loftOutputP2After(a, b) {
			dir = -1
		}
		edges[key] -= dir
	}
	for _, count := range edges {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if count != 0 {
			return false, nil
		}
	}
	return true, nil
}

func loftOutputP2After(a, b loftPartitionPoint) bool {
	if cmp := a.u.Cmp(b.u); cmp != 0 {
		return cmp > 0
	}
	return a.v.Cmp(b.v) > 0
}

func loftOutputPolygonArea(p []loftPartitionPoint) *big.Rat {
	area := new(big.Rat)
	for i, a := range p {
		b := p[(i+1)%len(p)]
		area.Add(area, new(big.Rat).Sub(new(big.Rat).Mul(a.u, b.v), new(big.Rat).Mul(a.v, b.u)))
	}
	return new(big.Rat).Quo(area, loftOutputRat(2))
}

func loftOutputOnSegment(p, a, b loftPartitionPoint) bool {
	if loftPartitionCross(a, b, p).Sign() != 0 {
		return false
	}
	return p.u.Cmp(loftOutputMinRat(a.u, b.u)) >= 0 && p.u.Cmp(loftOutputMaxRat(a.u, b.u)) <= 0 &&
		p.v.Cmp(loftOutputMinRat(a.v, b.v)) >= 0 && p.v.Cmp(loftOutputMaxRat(a.v, b.v)) <= 0
}

func loftOutputMinRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) < 0 {
		return a
	}
	return b
}

func loftOutputMaxRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}

// 1 is strict interior, 0 boundary, -1 exterior.
func loftOutputPointInPolygon(budget *proofbound.WorkBudget, p loftPartitionPoint,
	polygon []loftPartitionPoint) (int, error) {
	inside := false
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return -1, err
		}
		b := polygon[(i+1)%len(polygon)]
		if loftOutputOnSegment(p, a, b) {
			return 0, nil
		}
		if (a.v.Cmp(p.v) > 0) == (b.v.Cmp(p.v) > 0) {
			continue
		}
		x := new(big.Rat).Sub(b.u, a.u)
		x.Mul(x, new(big.Rat).Quo(new(big.Rat).Sub(p.v, a.v), new(big.Rat).Sub(b.v, a.v)))
		x.Add(x, a.u)
		if x.Cmp(p.u) > 0 {
			inside = !inside
		}
	}
	if inside {
		return 1, nil
	}
	return -1, nil
}

func loftOutputInterpolate(a, b loftPartitionPoint, t *big.Rat) loftPartitionPoint {
	return loftPartitionPoint{
		u: new(big.Rat).Add(a.u, new(big.Rat).Mul(t, new(big.Rat).Sub(b.u, a.u))),
		v: new(big.Rat).Add(a.v, new(big.Rat).Mul(t, new(big.Rat).Sub(b.v, a.v))),
	}
}

func loftOutputSegmentParam(a, b, p loftPartitionPoint) (*big.Rat, bool) {
	if !loftOutputOnSegment(p, a, b) {
		return nil, false
	}
	if a.u.Cmp(b.u) != 0 {
		return new(big.Rat).Quo(new(big.Rat).Sub(p.u, a.u), new(big.Rat).Sub(b.u, a.u)), true
	}
	if a.v.Cmp(b.v) != 0 {
		return new(big.Rat).Quo(new(big.Rat).Sub(p.v, a.v), new(big.Rat).Sub(b.v, a.v)), true
	}
	return nil, false
}

func loftOutputEdgeInside(budget *proofbound.WorkBudget, a, b loftPartitionPoint,
	polygon []loftPartitionPoint) (bool, error) {
	values := []*big.Rat{loftOutputRat(0), loftOutputRat(1)}
	for i, c := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		d := polygon[(i+1)%len(polygon)]
		r := loftPartitionPoint{new(big.Rat).Sub(b.u, a.u), new(big.Rat).Sub(b.v, a.v)}
		s := loftPartitionPoint{new(big.Rat).Sub(d.u, c.u), new(big.Rat).Sub(d.v, c.v)}
		den := new(big.Rat).Sub(new(big.Rat).Mul(r.u, s.v), new(big.Rat).Mul(r.v, s.u))
		if den.Sign() == 0 {
			for _, p := range []loftPartitionPoint{c, d} {
				if t, ok := loftOutputSegmentParam(a, b, p); ok {
					values = append(values, t)
				}
			}
			continue
		}
		ca := loftPartitionPoint{new(big.Rat).Sub(c.u, a.u), new(big.Rat).Sub(c.v, a.v)}
		t := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(ca.u, s.v),
			new(big.Rat).Mul(ca.v, s.u)), den)
		u := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(ca.u, r.v),
			new(big.Rat).Mul(ca.v, r.u)), den)
		if t.Sign() >= 0 && t.Cmp(loftOutputRat(1)) <= 0 &&
			u.Sign() >= 0 && u.Cmp(loftOutputRat(1)) <= 0 {
			values = append(values, t)
		}
	}
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && values[j].Cmp(values[j-1]) < 0; j-- {
			if err := budget.Step(); err != nil {
				return false, err
			}
			values[j], values[j-1] = values[j-1], values[j]
		}
	}
	for i := 1; i < len(values); i++ {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if values[i].Cmp(values[i-1]) == 0 {
			continue
		}
		mid := new(big.Rat).Add(values[i-1], values[i])
		mid.Quo(mid, loftOutputRat(2))
		status, err := loftOutputPointInPolygon(budget, loftOutputInterpolate(a, b, mid), polygon)
		if err != nil || status < 0 {
			return false, err
		}
	}
	return true, nil
}

func loftOutputTriangleInside(budget *proofbound.WorkBudget, polygon []loftPartitionPoint,
	tri [3]loftPartitionPoint) (bool, error) {
	for _, p := range tri {
		status, err := loftOutputPointInPolygon(budget, p, polygon)
		if err != nil || status < 0 {
			return false, err
		}
	}
	for i := range 3 {
		inside, err := loftOutputEdgeInside(budget, tri[i], tri[(i+1)%3], polygon)
		if err != nil || !inside {
			return false, err
		}
	}
	centroid := loftPartitionPoint{new(big.Rat), new(big.Rat)}
	for _, p := range tri {
		centroid.u.Add(centroid.u, p.u)
		centroid.v.Add(centroid.v, p.v)
	}
	centroid.u.Quo(centroid.u, loftOutputRat(3))
	centroid.v.Quo(centroid.v, loftOutputRat(3))
	status, err := loftOutputPointInPolygon(budget, centroid, polygon)
	return status == 1, err
}

func loftOutputTrianglesOverlap(budget *proofbound.WorkBudget,
	a, b [3]loftPartitionPoint) (bool, error) {
	poly := []loftPartitionPoint{a[0], a[1], a[2]}
	for i, c := range b {
		d := b[(i+1)%3]
		input := poly
		poly = nil
		for j, p := range input {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := input[(j+1)%len(input)]
			sp, sq := loftPartitionCross(c, d, p), loftPartitionCross(c, d, q)
			if sp.Sign() >= 0 {
				poly = append(poly, p)
			}
			if sp.Sign() == 0 || sq.Sign() == 0 || sp.Sign() == sq.Sign() {
				continue
			}
			t := new(big.Rat).Quo(sp, new(big.Rat).Sub(sp, sq))
			poly = append(poly, loftOutputInterpolate(p, q, t))
		}
		if len(poly) < 3 {
			return false, nil
		}
	}
	return loftOutputPolygonArea(poly).Sign() > 0, nil
}
