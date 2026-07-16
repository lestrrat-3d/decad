package decad

import (
	"fmt"
	"math"
	"sort"
)

// This file is the cap triangulator behind Body.Tessellate: a plane region
// given as a chorded outer boundary plus hole boundaries becomes triangles by
// hole bridging (Eberly's max-u visibility construction) followed by ear
// clipping — correct for non-convex outlines with holes, and deterministic.

// cross2 returns the z-component of (b − a) × (c − a): positive when a, b, c
// turn counter-clockwise.
func cross2(a, b, c Point2) float64 {
	return (b.U-a.U)*(c.V-a.V) - (b.V-a.V)*(c.U-a.U)
}

// pointInTri reports whether p lies inside or on the triangle a, b, c,
// whichever way the triangle is wound.
func pointInTri(p, a, b, c Point2) bool {
	d1 := cross2(a, b, p)
	d2 := cross2(b, c, p)
	d3 := cross2(c, a, p)
	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0
	return !hasNeg || !hasPos
}

// triangulate2D triangulates the plane region bounded by loops of indices
// into pts — loops[0] the outer boundary, counter-clockwise; the rest holes,
// clockwise — and returns counter-clockwise triangles as index triples. Each
// hole is bridged into the outer boundary at its vertex of maximum u, turning
// the region into one weakly-simple polygon, which ear clipping then reduces;
// zero-area corners are dropped without emitting a facet.
func triangulate2D(pts []Point2, loops [][]int) ([][3]int, error) {
	if len(loops) == 0 || len(loops[0]) < 3 {
		return nil, fmt.Errorf(`%w: a cap needs at least three boundary samples`, ErrDegenerate)
	}
	merged := append([]int(nil), loops[0]...)
	holes := append([][]int(nil), loops[1:]...)
	// Bridge right-to-left so each hole's visibility ray meets geometry that
	// is already part of the merged polygon.
	sort.SliceStable(holes, func(i, j int) bool {
		return maxU(pts, holes[i]) > maxU(pts, holes[j])
	})
	for _, hole := range holes {
		var err error
		merged, err = bridgeHole(pts, merged, hole)
		if err != nil {
			return nil, err
		}
	}
	return earClip(pts, merged)
}

// maxU returns the largest u-coordinate over the loop's vertices.
func maxU(pts []Point2, loop []int) float64 {
	u := math.Inf(-1)
	for _, i := range loop {
		u = math.Max(u, pts[i].U)
	}
	return u
}

// bridgeHole splices a hole loop into the merged polygon along a bridge
// between two mutually visible vertices — Eberly's construction: from the
// hole vertex M of maximum u, cast a ray in +u to the closest boundary
// crossing I on edge (a, b); bridge to the edge endpoint P of maximum u
// unless a reflex vertex inside triangle (M, I, P) is closer to the ray, in
// which case bridge to that vertex.
func bridgeHole(pts []Point2, merged, hole []int) ([]int, error) {
	if len(hole) < 3 {
		return nil, fmt.Errorf(`%w: a hole needs at least three boundary samples`, ErrDegenerate)
	}
	mi := 0
	for i := 1; i < len(hole); i++ {
		if pts[hole[i]].U > pts[hole[mi]].U {
			mi = i
		}
	}
	m := pts[hole[mi]]

	// The closest crossing of the ray m + t·(1, 0) with the merged boundary.
	// The half-open rule counts a crossing at a shared vertex exactly once
	// and skips horizontal edges.
	n := len(merged)
	bestU := math.Inf(1)
	bestEdge := -1
	for i := range n {
		a, b := pts[merged[i]], pts[merged[(i+1)%n]]
		if (a.V <= m.V) == (b.V <= m.V) {
			continue
		}
		u := a.U + (m.V-a.V)*(b.U-a.U)/(b.V-a.V)
		if u < m.U || u >= bestU {
			continue
		}
		bestU = u
		bestEdge = i
	}
	if bestEdge < 0 {
		return nil, fmt.Errorf(`%w: a hole lies outside its cap boundary`, ErrDegenerate)
	}
	if bestU == m.U {
		// The nearest crossing IS the hole vertex: the hole touches the
		// boundary it is being bridged into, and a zero-length bridge would
		// emit duplicate directed edges — a cracked mesh. The pinch is a
		// degenerate cap region, so it is an error, never a wrong mesh.
		return nil, fmt.Errorf(`%w: a hole touches its cap boundary`, ErrDegenerate)
	}
	hit := Point2{U: bestU, V: m.V}
	ai, bi := bestEdge, (bestEdge+1)%n
	a, b := pts[merged[ai]], pts[merged[bi]]

	// P: the intersected edge's endpoint of maximum u (a vertical edge ties;
	// take the upper endpoint). Bridging to the edge's far endpoint — even when
	// the ray's hit point I lands exactly on the near endpoint — is what keeps
	// the bridge off a hole edge collinear with the +u ray: an axis-aligned
	// tunnel whose corner shares the ray's v would otherwise get a bridge
	// running along its own edge, a zero-area sliver ear clipping dissolves and
	// leaves cracked. The far endpoint's visibility is proven by the reflex
	// search below, so this is Eberly's construction as stated, never a nearer
	// short-circuit that trades a proof for a coincidence.
	bridge := ai
	if b.U > a.U || (b.U == a.U && b.V > a.V) {
		bridge = bi
	}
	p := pts[merged[bridge]]
	if p == m {
		return nil, fmt.Errorf(`%w: a hole touches its cap boundary`, ErrDegenerate)
	}
	// A reflex vertex inside triangle (M, I, P) would block the bridge; of
	// those, the one whose direction from M lies closest to the ray (nearest
	// first on ties) is visible instead.
	bestCos, bestDist := math.Inf(-1), math.Inf(1)
	for j := range n {
		if j == bridge {
			continue
		}
		r := pts[merged[j]]
		prev, next := pts[merged[(j-1+n)%n]], pts[merged[(j+1)%n]]
		if cross2(prev, r, next) >= 0 {
			continue
		}
		if !pointInTri(r, m, hit, p) {
			continue
		}
		d := math.Hypot(r.U-m.U, r.V-m.V)
		if d == 0 {
			continue
		}
		cos := (r.U - m.U) / d
		if cos > bestCos || (cos == bestCos && d < bestDist) {
			bestCos, bestDist = cos, d
			bridge = j
		}
	}

	// Splice: …, bridge, M, the hole walked once around, M, bridge, … — the
	// bridge edge appears twice, once in each direction, keeping the merged
	// polygon weakly simple and counter-clockwise.
	out := make([]int, 0, n+len(hole)+2)
	out = append(out, merged[:bridge+1]...)
	for k := 0; k <= len(hole); k++ {
		out = append(out, hole[(mi+k)%len(hole)])
	}
	out = append(out, merged[bridge:]...)
	return out, nil
}

// earClip triangulates a counter-clockwise weakly-simple polygon by ear
// clipping: a strictly convex corner whose triangle contains no reflex vertex
// is emitted and removed. A zero-area (collinear) corner is removed WITHOUT
// emitting only when it is a bridge stub — the tip of a bridge's zero-width
// there-and-back channel, recognised by an index coincidence among the corner's
// three vertices (the spike apex has the SAME anchor index on both sides, and
// the residual doubled anchor an adjacent identical index). Removing such a
// stub drops only a bridge edge and its exact reverse, no boundary. A collinear
// corner whose three vertices are DISTINCT indices is a genuine boundary corner
// — its two edges are real boundary edges (possibly one from each of two holes
// sharing a horizontal v-line) — so it is deferred, never collapsed, since
// merging its edges would drop a boundary edge and crack the shared band. The
// deferred vertex turns strictly convex once an adjacent corner is clipped, so
// the pass always makes progress; a pass that clips nothing means the boundary
// self-intersects, which is an error, never a wrong mesh.
func earClip(pts []Point2, poly []int) ([][3]int, error) {
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 2 {
		n := len(idx)
		clipped := false
		for i := range n {
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			cr := cross2(pts[ia], pts[ib], pts[ic])
			if cr < 0 {
				continue
			}
			if cr == 0 && !isBridgeStub(ia, ib, ic) {
				continue
			}
			if cr > 0 && earBlocked(pts, idx, i) {
				continue
			}
			if cr > 0 {
				tris = append(tris, [3]int{ia, ib, ic})
			}
			idx = append(idx[:i], idx[i+1:]...)
			clipped = true
			break
		}
		if !clipped {
			return nil, fmt.Errorf(`%w: the chorded cap boundary self-intersects; tessellate at a finer tolerance`, ErrDegenerate)
		}
	}
	return tris, nil
}

// isBridgeStub reports whether the zero-area corner (ia, ib, ic) is a bridge
// stub safe to collapse — the tip or residue of a bridge's zero-width channel,
// where an index coincides among the three corner vertices. The spike apex M
// carries the SAME bridge anchor index P on both sides (ia == ic); collapsing it
// drops the reverse edges P→M and M→P and nothing else. The doubled anchor left
// behind carries an adjacent identical index (ia == ib or ib == ic); collapsing
// it drops only a zero-length self edge. A collinear corner whose three vertices
// are DISTINCT indices is NOT a stub — it is a genuine boundary corner whose two
// edges (one possibly from another hole sharing this v-line) must be kept.
func isBridgeStub(ia, ib, ic int) bool {
	return ia == ic || ia == ib || ib == ic
}

// earBlocked reports whether the candidate ear at position i is cut off from
// the region by another boundary vertex. Only a reflex vertex can block an
// ear of a simple polygon, and that rule also handles the duplicate vertices
// a hole bridge introduces.
func earBlocked(pts []Point2, idx []int, i int) bool {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	for j := range n {
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		prev, next := pts[idx[(j-1+n)%n]], pts[idx[(j+1)%n]]
		if cross2(prev, p, next) >= 0 {
			continue
		}
		if pointInTri(p, a, b, c) {
			return true
		}
	}
	return false
}
