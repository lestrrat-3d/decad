package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

func loftOutputCap(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	prismLines, bottom, top []sectionrecord.LineSeg, source loftSourcePartition,
	shell *loftOutputShell, capIndex int, checked ...*loftcontact.Arithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	if capIndex == 2 {
		polygon := make([]loftcontact.Point, 0, len(prismLines)+2)
		for i, line := range prismLines {
			if err := budget.Step(); err != nil {
				return false, err
			}
			p, ok := loftcontact.PointOf(line.Start)
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
					p = arithmetic.LinePoint(line, t)
					if !arithmetic.OK {
						return false, nil
					}
					polygon = append(polygon, p)
				}
			}
		}
		return loftOutputTileCap(ctx, budget, pp, shell, polygon, source.capLevels[2],
			loftOutputFace{kind: 0, owner: 0, level: 2, cell: 0}, false, arithmetic)
	}
	loftLines := bottom
	if capIndex == 1 {
		loftLines = top
	}
	level := source.capLevels[capIndex]
	prepared := &source.prepared[capIndex]
	if prepared.scene == nil || prepared.scene == source.prepared[1-capIndex].scene {
		return false, nil
	}
	// Re-run the accepted cap source proof against this call's live records.
	var cuts, carrierCuts [2]*big.Rat
	for k, seam := range source.seams {
		if err := budget.Step(); err != nil {
			return false, err
		}
		var ok bool
		cuts[k], carrierCuts[k], ok = seam.At(loftOutputRat(int64(capIndex)))
		if !ok || !arithmetic.Valid(cuts[k]) || !arithmetic.Valid(carrierCuts[k]) {
			return false, nil
		}
	}
	var sides []int8
	for _, line := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		side := arithmetic.LineSide(prismLines[source.carrier], line.Start)
		if !arithmetic.OK || side.Sign() == 0 {
			return false, nil
		}
		sides = append(sides, int8(side.Sign()))
	}
	capProof, ok, err := loftPartitionCap(ctx, budget, prismLines, loftLines,
		source.carrier, source.crossing, source.seams, sides, source.walls, capIndex, level,
		prepared, arithmetic)
	if err != nil || !ok || !arithmetic.OK || capProof.owners != source.capOwners[capIndex] ||
		capProof.junction != source.junction[capIndex] {
		return false, err
	}
	scene, tags, cells := prepared.scene, prepared.tags, prepared.raw
	origins, mapped, err := loftPartitionSceneOrigins(budget, scene, tags,
		prismLines, loftLines, capIndex, level)
	if err != nil || !mapped {
		return false, err
	}
	mapped, err = loftPartitionCertifyFragments(budget, scene, cells, origins,
		prismLines, loftLines, source.carrier, source.crossing, cuts, carrierCuts, capIndex, level, arithmetic)
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
			loftOutputFace{kind: 0, owner: owner, level: capIndex, cell: i}, capIndex == 0, arithmetic)
		if err != nil || !valid {
			return false, err
		}
	}
	if selected == 0 || count != source.capOwners[capIndex] {
		return false, nil
	}
	return loftOutputCapBoundary(budget, selectedCells, origins, prismLines, loftLines,
		source, cuts, carrierCuts, capIndex, arithmetic)
}

func loftOutputCapBoundary(budget *proofbound.WorkBudget, selected []*sketch.Profile,
	origins map[sketch.Entity]loftPartitionCapOrigin, prismLines, loftLines []sectionrecord.LineSeg,
	source loftSourcePartition, cut, carrierCut [2]*big.Rat, capIndex int,
	arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
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
			origin.cap != capIndex || c.Cmp(origin.level, source.capLevels[capIndex]) != 0 || !c.OK {
			return false, nil
		}
		a, okA := loftcontact.PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := loftcontact.PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return false, nil
		}
		lo, okLo := c.LineParameter(origin.line, a)
		hi, okHi := c.LineParameter(origin.line, b)
		if !okLo || !okHi || c.Cmp(lo, hi) == 0 || !c.OK {
			return false, nil
		}
		if c.Cmp(lo, hi) > 0 {
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
			side := c.LineSide(prismLines[source.carrier], loftLines[i].Start)
			if !c.OK || side.Sign() == 0 {
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
	return c.OK, nil
}

func loftOutputCellPolygon(budget *proofbound.WorkBudget, cell *sketch.Profile,
	origins map[sketch.Entity]loftPartitionCapOrigin) ([]loftcontact.Point, bool, error) {
	polygon := make([]loftcontact.Point, 0, len(cell.Outer))
	arithmetic := &loftcontact.Arithmetic{OK: true}
	for i, edge := range cell.Outer {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		origin, found := origins[edge.Entity]
		if !found || len(edge.Polyline) != 2 || !edge.TExact {
			return nil, false, nil
		}
		a, okA := loftcontact.PointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := loftcontact.PointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return nil, false, nil
		}
		_, srcA := arithmetic.LineParameter(origin.line, a)
		_, srcB := arithmetic.LineParameter(origin.line, b)
		next := cell.Outer[(i+1)%len(cell.Outer)]
		if !srcA || !srcB || !arithmetic.OK || len(next.Polyline) != 2 {
			return nil, false, nil
		}
		start, ok := loftcontact.PointOf(sectionrecord.Point2{
			U: next.Polyline[0][0], V: next.Polyline[0][1],
		})
		if !ok || !loftPartitionSamePoint(b, start) {
			return nil, false, nil
		}
		polygon = append(polygon, a)
	}
	return polygon, true, nil
}

func loftOutputTileCap(ctx context.Context, budget *proofbound.WorkBudget, pp prismPayload,
	shell *loftOutputShell, polygon []loftcontact.Point, level *big.Rat,
	face loftOutputFace, downward bool, arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	n := len(polygon)
	if n < 3 || n > 16 || len(shell.triangles)+n-2 > 512 ||
		len(shell.vertices)+n > 512 {
		return false, nil
	}
	c := loftReplayArithmetic(arithmetic)
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
	valid, err := loftOutputCertifyCapTiling(budget, polygon, candidate, c)
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

func loftOutputCertifyCapTiling(budget *proofbound.WorkBudget, polygon []loftcontact.Point,
	tris [][3]int, arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	area := loftOutputPolygonArea(polygon, c)
	if !c.OK || area.Sign() <= 0 {
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
		p := [3]loftcontact.Point{polygon[tri[0]], polygon[tri[1]], polygon[tri[2]]}
		double := c.PointCross(p[0], p[1], p[2])
		if !c.OK || double.Sign() <= 0 {
			return false, nil
		}
		sum = c.Add(sum, double)
		if !c.OK {
			return false, nil
		}
		inside, err := loftOutputTriangleInside(budget, polygon, p, c)
		if err != nil || !inside || !c.OK {
			return false, err
		}
		for j := range 3 {
			key := loftPartitionFragmentGeometryKey(p[j], p[(j+1)%3], c)
			if !c.OK {
				return false, nil
			}
			dir := 1
			if loftOutputP2After(p[j], p[(j+1)%3], c) {
				dir = -1
			}
			if !c.OK {
				return false, nil
			}
			edges[key] += dir
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := [3]loftcontact.Point{polygon[tris[j][0]], polygon[tris[j][1]], polygon[tris[j][2]]}
			overlap, err := loftOutputTrianglesOverlap(budget, p, q, c)
			if err != nil || !c.OK {
				return false, err
			}
			if overlap {
				return false, nil
			}
		}
	}
	if c.Cmp(sum, c.Mul(area, loftOutputRat(2))) != 0 || !c.OK {
		return false, nil
	}
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		b := polygon[(i+1)%len(polygon)]
		key := loftPartitionFragmentGeometryKey(a, b, c)
		if !c.OK {
			return false, nil
		}
		dir := 1
		if loftOutputP2After(a, b, c) {
			dir = -1
		}
		if !c.OK {
			return false, nil
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

func loftOutputP2After(a, b loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) bool {
	c := loftReplayArithmetic(arithmetic)
	if cmp := c.Cmp(a.U, b.U); cmp != 0 {
		return cmp > 0
	}
	return c.Cmp(a.V, b.V) > 0 && c.OK
}

func loftOutputPolygonArea(p []loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) *big.Rat {
	c := loftReplayArithmetic(arithmetic)
	area := new(big.Rat)
	for i, a := range p {
		b := p[(i+1)%len(p)]
		area = c.Add(area, c.Sub(c.Mul(a.U, b.V), c.Mul(a.V, b.U)))
		if !c.OK {
			return area
		}
	}
	return c.Quo(area, loftOutputRat(2))
}

func loftOutputOnSegment(p, a, b loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) bool {
	c := loftReplayArithmetic(arithmetic)
	if c.PointCross(a, b, p).Sign() != 0 || !c.OK {
		return false
	}
	inside := c.Cmp(p.U, loftOutputMinRat(a.U, b.U, c)) >= 0 &&
		c.Cmp(p.U, loftOutputMaxRat(a.U, b.U, c)) <= 0 &&
		c.Cmp(p.V, loftOutputMinRat(a.V, b.V, c)) >= 0 &&
		c.Cmp(p.V, loftOutputMaxRat(a.V, b.V, c)) <= 0
	return c.OK && inside
}

func loftOutputMinRat(a, b *big.Rat, arithmetic ...*loftcontact.Arithmetic) *big.Rat {
	if loftReplayArithmetic(arithmetic).Cmp(a, b) < 0 {
		return a
	}
	return b
}

func loftOutputMaxRat(a, b *big.Rat, arithmetic ...*loftcontact.Arithmetic) *big.Rat {
	if loftReplayArithmetic(arithmetic).Cmp(a, b) > 0 {
		return a
	}
	return b
}

// 1 is strict interior, 0 boundary, -1 exterior.
func loftOutputPointInPolygon(budget *proofbound.WorkBudget, p loftcontact.Point,
	polygon []loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) (int, error) {
	c := loftReplayArithmetic(arithmetic)
	inside := false
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return -1, err
		}
		b := polygon[(i+1)%len(polygon)]
		if loftOutputOnSegment(p, a, b, c) {
			return 0, nil
		}
		if !c.OK {
			return -1, nil
		}
		if (c.Cmp(a.V, p.V) > 0) == (c.Cmp(b.V, p.V) > 0) {
			if !c.OK {
				return -1, nil
			}
			continue
		}
		x := c.Add(c.Mul(c.Sub(b.U, a.U), c.Quo(c.Sub(p.V, a.V), c.Sub(b.V, a.V))), a.U)
		if c.Cmp(x, p.U) > 0 {
			inside = !inside
		}
		if !c.OK {
			return -1, nil
		}
	}
	if inside {
		return 1, nil
	}
	return -1, nil
}

func loftOutputSegmentParam(a, b, p loftcontact.Point,
	arithmetic ...*loftcontact.Arithmetic) (*big.Rat, bool) {
	c := loftReplayArithmetic(arithmetic)
	if !loftOutputOnSegment(p, a, b, c) || !c.OK {
		return nil, false
	}
	if c.Cmp(a.U, b.U) != 0 {
		t := c.Quo(c.Sub(p.U, a.U), c.Sub(b.U, a.U))
		return t, c.OK
	}
	if c.Cmp(a.V, b.V) != 0 {
		t := c.Quo(c.Sub(p.V, a.V), c.Sub(b.V, a.V))
		return t, c.OK
	}
	return nil, false
}

func loftOutputEdgeInside(budget *proofbound.WorkBudget, a, b loftcontact.Point,
	polygon []loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	values := []*big.Rat{loftOutputRat(0), loftOutputRat(1)}
	for i := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		d := polygon[(i+1)%len(polygon)]
		r := loftcontact.Point{U: c.Sub(b.U, a.U), V: c.Sub(b.V, a.V)}
		s := loftcontact.Point{U: c.Sub(d.U, polygon[i].U), V: c.Sub(d.V, polygon[i].V)}
		den := c.Sub(c.Mul(r.U, s.V), c.Mul(r.V, s.U))
		if !c.OK {
			return false, nil
		}
		if den.Sign() == 0 {
			for _, p := range []loftcontact.Point{polygon[i], d} {
				if t, ok := loftOutputSegmentParam(a, b, p, c); ok {
					values = append(values, t)
				}
				if !c.OK {
					return false, nil
				}
			}
			continue
		}
		ca := loftcontact.Point{U: c.Sub(polygon[i].U, a.U), V: c.Sub(polygon[i].V, a.V)}
		t := c.Quo(c.Sub(c.Mul(ca.U, s.V), c.Mul(ca.V, s.U)), den)
		u := c.Quo(c.Sub(c.Mul(ca.U, r.V), c.Mul(ca.V, r.U)), den)
		if !c.OK {
			return false, nil
		}
		if t.Sign() >= 0 && c.Cmp(t, loftOutputRat(1)) <= 0 &&
			u.Sign() >= 0 && c.Cmp(u, loftOutputRat(1)) <= 0 {
			values = append(values, t)
		}
		if !c.OK {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && c.Cmp(values[j], values[j-1]) < 0; j-- {
			if err := budget.Step(); err != nil {
				return false, err
			}
			values[j], values[j-1] = values[j-1], values[j]
		}
		if !c.OK {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if c.Cmp(values[i], values[i-1]) == 0 {
			if !c.OK {
				return false, nil
			}
			continue
		}
		mid := c.Quo(c.Add(values[i-1], values[i]), loftOutputRat(2))
		point := c.Interpolate(a, b, mid)
		if !c.OK {
			return false, nil
		}
		status, err := loftOutputPointInPolygon(budget, point, polygon, c)
		if err != nil || status < 0 || !c.OK {
			return false, err
		}
	}
	return true, nil
}

func loftOutputTriangleInside(budget *proofbound.WorkBudget, polygon []loftcontact.Point,
	tri [3]loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	for _, p := range tri {
		status, err := loftOutputPointInPolygon(budget, p, polygon, c)
		if err != nil || status < 0 || !c.OK {
			return false, err
		}
	}
	for i := range 3 {
		inside, err := loftOutputEdgeInside(budget, tri[i], tri[(i+1)%3], polygon, c)
		if err != nil || !inside || !c.OK {
			return false, err
		}
	}
	centroid := loftcontact.Point{U: new(big.Rat), V: new(big.Rat)}
	for _, p := range tri {
		centroid.U = c.Add(centroid.U, p.U)
		centroid.V = c.Add(centroid.V, p.V)
	}
	centroid.U = c.Quo(centroid.U, loftOutputRat(3))
	centroid.V = c.Quo(centroid.V, loftOutputRat(3))
	if !c.OK {
		return false, nil
	}
	status, err := loftOutputPointInPolygon(budget, centroid, polygon, c)
	return c.OK && status == 1, err
}

func loftOutputTrianglesOverlap(budget *proofbound.WorkBudget,
	a, b [3]loftcontact.Point, arithmetic ...*loftcontact.Arithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	poly := []loftcontact.Point{a[0], a[1], a[2]}
	for i := range b {
		d := b[(i+1)%3]
		input := poly
		poly = nil
		for j, p := range input {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := input[(j+1)%len(input)]
			sp, sq := c.PointCross(b[i], d, p), c.PointCross(b[i], d, q)
			if !c.OK {
				return false, nil
			}
			if sp.Sign() >= 0 {
				poly = append(poly, p)
			}
			if sp.Sign() == 0 || sq.Sign() == 0 || sp.Sign() == sq.Sign() {
				continue
			}
			t := c.Quo(sp, c.Sub(sp, sq))
			poly = append(poly, c.Interpolate(p, q, t))
			if !c.OK {
				return false, nil
			}
		}
		if len(poly) < 3 {
			return false, nil
		}
	}
	area := loftOutputPolygonArea(poly, c)
	return c.OK && area.Sign() > 0, nil
}

// All coordinates here are exact rationals over authenticated source floats.
// The output certificate and its source/held pairing die with the call.
type loftOutputPoint struct{ u, v, z *big.Rat }

type loftOutputFace struct {
	kind, owner, segment, level int
	cell                        int
}

type loftOutputVertex struct {
	point      loftOutputPoint
	world      [3]*big.Rat
	held       r3.Vec
	bound      float64
	incidences map[loftOutputFace]struct{}
}

type loftOutputShell struct {
	vertices       []loftOutputVertex
	triangles      [][3]int
	faces          []loftOutputFace
	triangleBounds []float64
	index          map[string]int
	heldIndex      map[[3]uint64]string
	maxBound       float64
}

func loftOutputRat(v int64) *big.Rat { return big.NewRat(v, 1) }

func loftReplayArithmetic(optional []*loftcontact.Arithmetic) *loftcontact.Arithmetic {
	if len(optional) != 0 {
		return optional[0]
	}
	return &loftcontact.Arithmetic{OK: true}
}

func loftOutputP2(p loftcontact.Point, z *big.Rat) loftOutputPoint {
	return loftOutputPoint{p.U, p.V, z}
}

func loftOutputPointKey(p loftOutputPoint) string {
	return p.u.RatString() + "," + p.v.RatString() + "," + p.z.RatString()
}

func loftOutputSamePoint(a, b loftOutputPoint) bool {
	arithmetic := &loftcontact.Arithmetic{OK: true}
	same := arithmetic.Cmp(a.u, b.u) == 0 && arithmetic.Cmp(a.v, b.v) == 0 &&
		arithmetic.Cmp(a.z, b.z) == 0
	return arithmetic.OK && same
}

func loftOutputWorld(frame r3.Frame, p loftOutputPoint) ([3]*big.Rat, r3.Vec, float64, bool) {
	var exact [3]*big.Rat
	c := &loftcontact.Arithmetic{OK: true}
	if !c.Valid(p.u) || !c.Valid(p.v) || !c.Valid(p.z) {
		return exact, r3.Vec{}, 0, false
	}
	o, u, v, n := frame.Origin(), frame.U(), frame.V(), frame.N()
	if !proofbound.FiniteVec(o) || !proofbound.FiniteVec(u) ||
		!proofbound.FiniteVec(v) || !proofbound.FiniteVec(n) {
		return exact, r3.Vec{}, 0, false
	}
	coords := [4][3]float64{{o.X, o.Y, o.Z}, {u.X, u.Y, u.Z},
		{v.X, v.Y, v.Z}, {n.X, n.Y, n.Z}}
	params := [3]*big.Rat{p.u, p.v, p.z}
	var held [3]float64
	gap2 := new(big.Rat)
	for i := range exact {
		exact[i] = proofarith.FloatRat(coords[0][i])
		for j, param := range params {
			term := c.Mul(proofarith.FloatRat(coords[j+1][i]), param)
			exact[i] = c.Add(exact[i], term)
			if !c.OK {
				return exact, r3.Vec{}, 0, false
			}
		}
		held[i], _ = c.Float64(exact[i])
		if !c.OK || proofbound.IsNonFinite(held[i]) {
			return exact, r3.Vec{}, 0, false
		}
		delta := c.Sub(exact[i], proofarith.FloatRat(held[i]))
		gap2 = c.Add(gap2, c.Mul(delta, delta))
		if !c.OK {
			return exact, r3.Vec{}, 0, false
		}
	}
	// RatSqrtUp compares a finite float square with gap2 using up to 2,252
	// shift bits on its numerator or 2,048 on its denominator.
	if gap2.Num().BitLen() > loftcontact.RationalBits-2252 ||
		gap2.Denom().BitLen() > loftcontact.RationalBits-2048 {
		return exact, r3.Vec{}, 0, false
	}
	bound := proofbound.RatSqrtUp(gap2)
	if proofbound.IsNonFinite(bound) {
		return exact, r3.Vec{}, 0, false
	}
	return exact, r3.Vec{X: held[0], Y: held[1], Z: held[2]}, bound, true
}

func loftOutputHeldKey(v r3.Vec) [3]uint64 {
	coords := [3]float64{v.X, v.Y, v.Z}
	var key [3]uint64
	for i, x := range coords {
		if x != 0 {
			key[i] = math.Float64bits(x)
		}
	}
	return key
}

func (s *loftOutputShell) vertex(frame r3.Frame, p loftOutputPoint, face loftOutputFace) (int, bool) {
	c := &loftcontact.Arithmetic{OK: true}
	if !c.Valid(p.u) || !c.Valid(p.v) || !c.Valid(p.z) {
		return 0, false
	}
	key := loftOutputPointKey(p)
	if index, exists := s.index[key]; exists {
		s.vertices[index].incidences[face] = struct{}{}
		return index, true
	}
	if len(s.vertices) == 512 {
		return 0, false
	}
	world, held, bound, ok := loftOutputWorld(frame, p)
	if !ok {
		return 0, false
	}
	heldKey := loftOutputHeldKey(held)
	if prior, exists := s.heldIndex[heldKey]; exists && prior != key {
		return 0, false
	}
	index := len(s.vertices)
	s.index[key], s.heldIndex[heldKey] = index, key
	s.vertices = append(s.vertices, loftOutputVertex{point: p, world: world, held: held,
		bound: bound, incidences: map[loftOutputFace]struct{}{face: {}}})
	s.maxBound = max(s.maxBound, bound)
	return index, true
}

func (s *loftOutputShell) triangle(frame r3.Frame, points [3]loftOutputPoint,
	face loftOutputFace) bool {
	if len(s.triangles) == 512 {
		return false
	}
	var ids [3]int
	for i, p := range points {
		var ok bool
		ids[i], ok = s.vertex(frame, p, face)
		if !ok {
			return false
		}
	}
	if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] ||
		loftOutputCrossZero(points) {
		return false
	}
	s.triangles = append(s.triangles, ids)
	s.faces = append(s.faces, face)
	s.triangleBounds = append(s.triangleBounds, max(s.vertices[ids[0]].bound,
		s.vertices[ids[1]].bound, s.vertices[ids[2]].bound))
	return true
}

func loftOutputCrossZero(p [3]loftOutputPoint) bool {
	c := &loftcontact.Arithmetic{OK: true}
	a := [3]*big.Rat{c.Sub(p[1].u, p[0].u),
		c.Sub(p[1].v, p[0].v), c.Sub(p[1].z, p[0].z)}
	b := [3]*big.Rat{c.Sub(p[2].u, p[0].u),
		c.Sub(p[2].v, p[0].v), c.Sub(p[2].z, p[0].z)}
	if !c.OK {
		return true
	}
	for i := range a {
		j, k := (i+1)%3, (i+2)%3
		cross := c.Sub(c.Mul(a[j], b[k]), c.Mul(a[k], b[j]))
		if !c.OK {
			return true
		}
		if cross.Sign() != 0 {
			return false
		}
	}
	return true
}

func loftOutputNewShell() loftOutputShell {
	return loftOutputShell{index: make(map[string]int), heldIndex: make(map[[3]uint64]string)}
}
