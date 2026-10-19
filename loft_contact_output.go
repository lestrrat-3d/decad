package decad

import (
	"context"
	"math"
	"math/big"

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
	shell *loftOutputShell, capIndex int, checked ...*loftVolumeArithmetic) (bool, error) {
	arithmetic := loftReplayArithmetic(checked)
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
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
					p = arithmetic.partitionLinePoint(line, t)
					if !arithmetic.ok {
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
		if !ok || !arithmetic.valid(cuts[k]) || !arithmetic.valid(carrierCuts[k]) {
			return false, nil
		}
	}
	var sides []int8
	for _, line := range loftLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		side := arithmetic.partitionSide(prismLines[source.carrier], line.Start)
		if !arithmetic.ok || side.Sign() == 0 {
			return false, nil
		}
		sides = append(sides, int8(side.Sign()))
	}
	capProof, ok, err := loftPartitionCap(ctx, budget, prismLines, loftLines,
		source.carrier, source.crossing, source.seams, sides, source.walls, capIndex, level,
		prepared, arithmetic)
	if err != nil || !ok || !arithmetic.ok || capProof.owners != source.capOwners[capIndex] ||
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
	arithmetic ...*loftVolumeArithmetic) (bool, error) {
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
			origin.cap != capIndex || c.cmp(origin.level, source.capLevels[capIndex]) != 0 || !c.ok {
			return false, nil
		}
		a, okA := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[0][0], V: edge.Polyline[0][1]})
		b, okB := loftPartitionPointOf(sectionrecord.Point2{U: edge.Polyline[1][0], V: edge.Polyline[1][1]})
		if !okA || !okB {
			return false, nil
		}
		lo, okLo := c.partitionParameter(origin.line, a)
		hi, okHi := c.partitionParameter(origin.line, b)
		if !okLo || !okHi || c.cmp(lo, hi) == 0 || !c.ok {
			return false, nil
		}
		if c.cmp(lo, hi) > 0 {
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
			side := c.partitionSide(prismLines[source.carrier], loftLines[i].Start)
			if !c.ok || side.Sign() == 0 {
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
	return c.ok, nil
}

func loftOutputCellPolygon(budget *proofbound.WorkBudget, cell *sketch.Profile,
	origins map[sketch.Entity]loftPartitionCapOrigin) ([]loftPartitionPoint, bool, error) {
	polygon := make([]loftPartitionPoint, 0, len(cell.Outer))
	arithmetic := &loftVolumeArithmetic{ok: true}
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
		_, srcA := arithmetic.partitionParameter(origin.line, a)
		_, srcB := arithmetic.partitionParameter(origin.line, b)
		next := cell.Outer[(i+1)%len(cell.Outer)]
		if !srcA || !srcB || !arithmetic.ok || len(next.Polyline) != 2 {
			return nil, false, nil
		}
		start, ok := loftPartitionPointOf(sectionrecord.Point2{
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
	shell *loftOutputShell, polygon []loftPartitionPoint, level *big.Rat,
	face loftOutputFace, downward bool, arithmetic ...*loftVolumeArithmetic) (bool, error) {
	n := len(polygon)
	if n < 3 || n > 16 || len(shell.triangles)+n-2 > 512 ||
		len(shell.vertices)+n > 512 {
		return false, nil
	}
	c := loftReplayArithmetic(arithmetic)
	if !c.valid(level) {
		return false, nil
	}
	for _, point := range polygon {
		if !c.valid(point.u) || !c.valid(point.v) {
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
		pts[i].U, _ = c.float64(p.u)
		pts[i].V, _ = c.float64(p.v)
		if !c.ok || proofbound.IsNonFinite(pts[i].U) || proofbound.IsNonFinite(pts[i].V) {
			return false, nil
		}
		idx[i] = i
	}
	candidate, err := triangulation.Triangulate(ctx, pts, [][]int{idx})
	if err != nil || len(candidate) != n-2 {
		return false, err
	}
	valid, err := loftOutputCertifyCapTiling(budget, polygon, candidate, c)
	if err != nil || !valid || !c.ok {
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
	tris [][3]int, arithmetic ...*loftVolumeArithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	area := loftOutputPolygonArea(polygon, c)
	if !c.ok || area.Sign() <= 0 {
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
		double := c.partitionCross(p[0], p[1], p[2])
		if !c.ok || double.Sign() <= 0 {
			return false, nil
		}
		sum = c.add(sum, double)
		if !c.ok {
			return false, nil
		}
		inside, err := loftOutputTriangleInside(budget, polygon, p, c)
		if err != nil || !inside || !c.ok {
			return false, err
		}
		for j := range 3 {
			key := loftPartitionFragmentGeometryKey(p[j], p[(j+1)%3], c)
			if !c.ok {
				return false, nil
			}
			dir := 1
			if loftOutputP2After(p[j], p[(j+1)%3], c) {
				dir = -1
			}
			if !c.ok {
				return false, nil
			}
			edges[key] += dir
		}
		for j := range i {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := [3]loftPartitionPoint{polygon[tris[j][0]], polygon[tris[j][1]], polygon[tris[j][2]]}
			overlap, err := loftOutputTrianglesOverlap(budget, p, q, c)
			if err != nil || !c.ok {
				return false, err
			}
			if overlap {
				return false, nil
			}
		}
	}
	if c.cmp(sum, c.mul(area, loftOutputRat(2))) != 0 || !c.ok {
		return false, nil
	}
	for i, a := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		b := polygon[(i+1)%len(polygon)]
		key := loftPartitionFragmentGeometryKey(a, b, c)
		if !c.ok {
			return false, nil
		}
		dir := 1
		if loftOutputP2After(a, b, c) {
			dir = -1
		}
		if !c.ok {
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

func loftOutputP2After(a, b loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) bool {
	c := loftReplayArithmetic(arithmetic)
	if cmp := c.cmp(a.u, b.u); cmp != 0 {
		return cmp > 0
	}
	return c.cmp(a.v, b.v) > 0 && c.ok
}

func loftOutputPolygonArea(p []loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) *big.Rat {
	c := loftReplayArithmetic(arithmetic)
	area := new(big.Rat)
	for i, a := range p {
		b := p[(i+1)%len(p)]
		area = c.add(area, c.sub(c.mul(a.u, b.v), c.mul(a.v, b.u)))
		if !c.ok {
			return area
		}
	}
	return c.quo(area, loftOutputRat(2))
}

func loftOutputOnSegment(p, a, b loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) bool {
	c := loftReplayArithmetic(arithmetic)
	if c.partitionCross(a, b, p).Sign() != 0 || !c.ok {
		return false
	}
	inside := c.cmp(p.u, loftOutputMinRat(a.u, b.u, c)) >= 0 &&
		c.cmp(p.u, loftOutputMaxRat(a.u, b.u, c)) <= 0 &&
		c.cmp(p.v, loftOutputMinRat(a.v, b.v, c)) >= 0 &&
		c.cmp(p.v, loftOutputMaxRat(a.v, b.v, c)) <= 0
	return c.ok && inside
}

func loftOutputMinRat(a, b *big.Rat, arithmetic ...*loftVolumeArithmetic) *big.Rat {
	if loftReplayArithmetic(arithmetic).cmp(a, b) < 0 {
		return a
	}
	return b
}

func loftOutputMaxRat(a, b *big.Rat, arithmetic ...*loftVolumeArithmetic) *big.Rat {
	if loftReplayArithmetic(arithmetic).cmp(a, b) > 0 {
		return a
	}
	return b
}

// 1 is strict interior, 0 boundary, -1 exterior.
func loftOutputPointInPolygon(budget *proofbound.WorkBudget, p loftPartitionPoint,
	polygon []loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) (int, error) {
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
		if !c.ok {
			return -1, nil
		}
		if (c.cmp(a.v, p.v) > 0) == (c.cmp(b.v, p.v) > 0) {
			if !c.ok {
				return -1, nil
			}
			continue
		}
		x := c.add(c.mul(c.sub(b.u, a.u), c.quo(c.sub(p.v, a.v), c.sub(b.v, a.v))), a.u)
		if c.cmp(x, p.u) > 0 {
			inside = !inside
		}
		if !c.ok {
			return -1, nil
		}
	}
	if inside {
		return 1, nil
	}
	return -1, nil
}

func loftOutputSegmentParam(a, b, p loftPartitionPoint,
	arithmetic ...*loftVolumeArithmetic) (*big.Rat, bool) {
	c := loftReplayArithmetic(arithmetic)
	if !loftOutputOnSegment(p, a, b, c) || !c.ok {
		return nil, false
	}
	if c.cmp(a.u, b.u) != 0 {
		t := c.quo(c.sub(p.u, a.u), c.sub(b.u, a.u))
		return t, c.ok
	}
	if c.cmp(a.v, b.v) != 0 {
		t := c.quo(c.sub(p.v, a.v), c.sub(b.v, a.v))
		return t, c.ok
	}
	return nil, false
}

func loftOutputEdgeInside(budget *proofbound.WorkBudget, a, b loftPartitionPoint,
	polygon []loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	values := []*big.Rat{loftOutputRat(0), loftOutputRat(1)}
	for i := range polygon {
		if err := budget.Step(); err != nil {
			return false, err
		}
		d := polygon[(i+1)%len(polygon)]
		r := loftPartitionPoint{c.sub(b.u, a.u), c.sub(b.v, a.v)}
		s := loftPartitionPoint{c.sub(d.u, polygon[i].u), c.sub(d.v, polygon[i].v)}
		den := c.sub(c.mul(r.u, s.v), c.mul(r.v, s.u))
		if !c.ok {
			return false, nil
		}
		if den.Sign() == 0 {
			for _, p := range []loftPartitionPoint{polygon[i], d} {
				if t, ok := loftOutputSegmentParam(a, b, p, c); ok {
					values = append(values, t)
				}
				if !c.ok {
					return false, nil
				}
			}
			continue
		}
		ca := loftPartitionPoint{c.sub(polygon[i].u, a.u), c.sub(polygon[i].v, a.v)}
		t := c.quo(c.sub(c.mul(ca.u, s.v), c.mul(ca.v, s.u)), den)
		u := c.quo(c.sub(c.mul(ca.u, r.v), c.mul(ca.v, r.u)), den)
		if !c.ok {
			return false, nil
		}
		if t.Sign() >= 0 && c.cmp(t, loftOutputRat(1)) <= 0 &&
			u.Sign() >= 0 && c.cmp(u, loftOutputRat(1)) <= 0 {
			values = append(values, t)
		}
		if !c.ok {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && c.cmp(values[j], values[j-1]) < 0; j-- {
			if err := budget.Step(); err != nil {
				return false, err
			}
			values[j], values[j-1] = values[j-1], values[j]
		}
		if !c.ok {
			return false, nil
		}
	}
	for i := 1; i < len(values); i++ {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if c.cmp(values[i], values[i-1]) == 0 {
			if !c.ok {
				return false, nil
			}
			continue
		}
		mid := c.quo(c.add(values[i-1], values[i]), loftOutputRat(2))
		point := c.partitionInterpolate(a, b, mid)
		if !c.ok {
			return false, nil
		}
		status, err := loftOutputPointInPolygon(budget, point, polygon, c)
		if err != nil || status < 0 || !c.ok {
			return false, err
		}
	}
	return true, nil
}

func loftOutputTriangleInside(budget *proofbound.WorkBudget, polygon []loftPartitionPoint,
	tri [3]loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	for _, p := range tri {
		status, err := loftOutputPointInPolygon(budget, p, polygon, c)
		if err != nil || status < 0 || !c.ok {
			return false, err
		}
	}
	for i := range 3 {
		inside, err := loftOutputEdgeInside(budget, tri[i], tri[(i+1)%3], polygon, c)
		if err != nil || !inside || !c.ok {
			return false, err
		}
	}
	centroid := loftPartitionPoint{new(big.Rat), new(big.Rat)}
	for _, p := range tri {
		centroid.u = c.add(centroid.u, p.u)
		centroid.v = c.add(centroid.v, p.v)
	}
	centroid.u = c.quo(centroid.u, loftOutputRat(3))
	centroid.v = c.quo(centroid.v, loftOutputRat(3))
	if !c.ok {
		return false, nil
	}
	status, err := loftOutputPointInPolygon(budget, centroid, polygon, c)
	return c.ok && status == 1, err
}

func loftOutputTrianglesOverlap(budget *proofbound.WorkBudget,
	a, b [3]loftPartitionPoint, arithmetic ...*loftVolumeArithmetic) (bool, error) {
	c := loftReplayArithmetic(arithmetic)
	poly := []loftPartitionPoint{a[0], a[1], a[2]}
	for i := range b {
		d := b[(i+1)%3]
		input := poly
		poly = nil
		for j, p := range input {
			if err := budget.Step(); err != nil {
				return false, err
			}
			q := input[(j+1)%len(input)]
			sp, sq := c.partitionCross(b[i], d, p), c.partitionCross(b[i], d, q)
			if !c.ok {
				return false, nil
			}
			if sp.Sign() >= 0 {
				poly = append(poly, p)
			}
			if sp.Sign() == 0 || sq.Sign() == 0 || sp.Sign() == sq.Sign() {
				continue
			}
			t := c.quo(sp, c.sub(sp, sq))
			poly = append(poly, c.partitionInterpolate(p, q, t))
			if !c.ok {
				return false, nil
			}
		}
		if len(poly) < 3 {
			return false, nil
		}
	}
	area := loftOutputPolygonArea(poly, c)
	return c.ok && area.Sign() > 0, nil
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

func loftReplayArithmetic(optional []*loftVolumeArithmetic) *loftVolumeArithmetic {
	if len(optional) != 0 {
		return optional[0]
	}
	return &loftVolumeArithmetic{ok: true}
}

// float64 checks the alignment performed by big.Rat.Float64 before it can
// allocate a temporary wider than the replay rational limit.
func (c *loftVolumeArithmetic) float64(a *big.Rat) (float64, bool) {
	if !c.valid(a) {
		return 0, false
	}
	if numeratorBits := a.Num().BitLen(); numeratorBits != 0 {
		denominatorBits := a.Denom().BitLen()
		if shift := 54 - (numeratorBits - denominatorBits); shift > 0 &&
			numeratorBits+shift > loftVolumeRationalBits {
			c.ok = false
			return 0, false
		}
	}
	return a.Float64()
}

func loftOutputP2(p loftPartitionPoint, z *big.Rat) loftOutputPoint {
	return loftOutputPoint{p.u, p.v, z}
}

func loftOutputPointKey(p loftOutputPoint) string {
	return p.u.RatString() + "," + p.v.RatString() + "," + p.z.RatString()
}

func loftOutputSamePoint(a, b loftOutputPoint) bool {
	arithmetic := &loftVolumeArithmetic{ok: true}
	same := arithmetic.cmp(a.u, b.u) == 0 && arithmetic.cmp(a.v, b.v) == 0 &&
		arithmetic.cmp(a.z, b.z) == 0
	return arithmetic.ok && same
}

func loftOutputWorld(frame r3.Frame, p loftOutputPoint) ([3]*big.Rat, r3.Vec, float64, bool) {
	var exact [3]*big.Rat
	c := &loftVolumeArithmetic{ok: true}
	if !c.valid(p.u) || !c.valid(p.v) || !c.valid(p.z) {
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
			term := c.mul(proofarith.FloatRat(coords[j+1][i]), param)
			exact[i] = c.add(exact[i], term)
			if !c.ok {
				return exact, r3.Vec{}, 0, false
			}
		}
		held[i], _ = c.float64(exact[i])
		if !c.ok || proofbound.IsNonFinite(held[i]) {
			return exact, r3.Vec{}, 0, false
		}
		delta := c.sub(exact[i], proofarith.FloatRat(held[i]))
		gap2 = c.add(gap2, c.mul(delta, delta))
		if !c.ok {
			return exact, r3.Vec{}, 0, false
		}
	}
	// RatSqrtUp compares a finite float square with gap2 using up to 2,252
	// shift bits on its numerator or 2,048 on its denominator.
	if gap2.Num().BitLen() > loftVolumeRationalBits-2252 ||
		gap2.Denom().BitLen() > loftVolumeRationalBits-2048 {
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
	c := &loftVolumeArithmetic{ok: true}
	if !c.valid(p.u) || !c.valid(p.v) || !c.valid(p.z) {
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
	c := &loftVolumeArithmetic{ok: true}
	a := [3]*big.Rat{c.sub(p[1].u, p[0].u),
		c.sub(p[1].v, p[0].v), c.sub(p[1].z, p[0].z)}
	b := [3]*big.Rat{c.sub(p[2].u, p[0].u),
		c.sub(p[2].v, p[0].v), c.sub(p[2].z, p[0].z)}
	if !c.ok {
		return true
	}
	for i := range a {
		j, k := (i+1)%3, (i+2)%3
		cross := c.sub(c.mul(a[j], b[k]), c.mul(a[k], b[j]))
		if !c.ok {
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
