package decad

import (
	"context"
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

// triangulate2DContext triangulates the plane region bounded by loops of
// indices into pts — loops[0] the outer boundary, counter-clockwise; the rest
// holes, clockwise — and returns counter-clockwise triangles as index triples.
// Each hole is bridged into the outer boundary at its vertex of maximum u,
// turning the region into one weakly-simple polygon, which ear clipping then
// reduces; zero-area corners are dropped without emitting a facet. The caller
// context reaches every quadratic bridge and ear-clipping walk.
func triangulate2DContext(ctx context.Context, pts []Point2, loops [][]int) ([][3]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(loops) == 0 || len(loops[0]) < 3 {
		return nil, fmt.Errorf(`%w: a cap needs at least three boundary samples`, ErrDegenerate)
	}
	budget := newWorkBudget(ctx)
	merged := append([]int(nil), loops[0]...)
	holes := append([][]int(nil), loops[1:]...)
	// Bridge right-to-left so each hole's visibility ray meets geometry that
	// is already part of the merged polygon. Each key is read once, under the
	// budget: the comparison is a leaf, and rescanning a hole's samples inside
	// it would put a walk of every hole behind every comparison.
	keys := make([]float64, len(holes))
	for i, hole := range holes {
		var err error
		if keys[i], err = maxU(budget, pts, hole); err != nil {
			return nil, err
		}
	}
	order := make([]int, len(holes))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(i, j int) bool { return keys[order[i]] > keys[order[j]] })
	for _, h := range order {
		hole := holes[h]
		if err := budget.step(); err != nil {
			return nil, err
		}
		var err error
		merged, err = bridgeHole(ctx, pts, merged, hole)
		if err != nil {
			return nil, err
		}
	}
	return earClip(ctx, pts, merged)
}

// maxU returns the largest u-coordinate over the loop's vertices.
func maxU(budget *workBudget, pts []Point2, loop []int) (float64, error) {
	u := math.Inf(-1)
	for _, i := range loop {
		if err := budget.step(); err != nil {
			return 0, err
		}
		u = math.Max(u, pts[i].U)
	}
	return u, nil
}

// bridgeHole splices a hole loop into the merged polygon along a bridge
// between two mutually visible vertices — Eberly's construction: from the
// hole vertex M of maximum u, cast a ray in +u to the closest boundary
// crossing I on edge (a, b); bridge to the edge endpoint P of maximum u
// unless a reflex vertex inside triangle (M, I, P) is closer to the ray, in
// which case bridge to that vertex.
func bridgeHole(ctx context.Context, pts []Point2, merged, hole []int) ([]int, error) {
	if len(hole) < 3 {
		return nil, fmt.Errorf(`%w: a hole needs at least three boundary samples`, ErrDegenerate)
	}
	budget := newWorkBudget(ctx)
	mi := 0
	for i := 1; i < len(hole); i++ {
		if err := budget.step(); err != nil {
			return nil, err
		}
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
		if err := budget.step(); err != nil {
			return nil, err
		}
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
		// A hole with no boundary crossing anywhere along its +u ray is not
		// inside the region at all. That is the operand's own geometry, which
		// no tolerance changes, so it stays an untyped ErrDegenerate that
		// Verify returns (docs/interference-design.md §7.1).
		return nil, fmt.Errorf(`%w: a hole lies outside its cap boundary`, ErrDegenerate)
	}
	if bestU == m.U {
		// The nearest crossing IS the hole vertex: the chorded hole touches the
		// chorded boundary it is being bridged into, and a zero-length bridge
		// would emit duplicate directed edges — a cracked mesh. Each chording
		// lies within its own sagitta of the true curve, so a pinch between
		// them is a property of where the chords fell, the same class
		// requireLoopClearance refuses: an expected undecided outcome
		// (docs/interference-design.md §7.1), never a wrong mesh.
		return nil, &tessellationExpectedError{err: fmt.Errorf(`%w: a hole touches its cap boundary`, ErrDegenerate)}
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
		// The bridge target coincides with the hole vertex: the same chord-
		// placed pinch as above, and the same expected undecided outcome.
		return nil, &tessellationExpectedError{err: fmt.Errorf(`%w: a hole touches its cap boundary`, ErrDegenerate)}
	}
	// A reflex vertex inside triangle (M, I, P) would block the bridge; of
	// those, the one whose direction from M lies closest to the ray (nearest
	// first on ties) is visible instead.
	bestCos, bestDist := math.Inf(-1), math.Inf(1)
	for j := range n {
		if err := budget.step(); err != nil {
			return nil, err
		}
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

	// The chosen bridge VERTEX is proven visible from M, but a vertex two
	// earlier bridges already landed on appears at several positions in merged,
	// and only one of them can take this bridge without crossing the channels
	// already cut there. Pick that position by the wedge each occurrence owns.
	occ, ok := bridgeOccurrence(pts, merged, bridge, m)
	if !ok {
		// No occurrence's interior wedge admits the bridge: the anchor is already
		// so densely bridged that this hole's channel has nowhere consistent to
		// go. That is a property of where the chords fell, the same class
		// requireLoopClearance refuses — an expected undecided outcome, never a
		// wrong mesh.
		return nil, &tessellationExpectedError{err: fmt.Errorf(`%w: no occurrence of a hole's bridge anchor admits the bridge`, ErrDegenerate)}
	}
	bridge = occ

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
func earClip(ctx context.Context, pts []Point2, poly []int) ([][3]int, error) {
	idx := append([]int(nil), poly...)
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			cr := cross2(pts[ia], pts[ib], pts[ic])
			if cr < 0 {
				continue
			}
			if cr == 0 {
				// A zero-area corner is removed only when it is a bridge stub, and
				// never emits a triangle — so the reflex-blocking scan, whose
				// result is discarded for such a corner, is skipped entirely.
				if !isBridgeStub(ia, ib, ic) {
					continue
				}
			} else {
				blocked, err := earBlocked(ctx, pts, idx, i)
				if err != nil {
					return nil, err
				}
				if blocked {
					continue
				}
				tris = append(tris, [3]int{ia, ib, ic})
			}
			idx = append(idx[:i], idx[i+1:]...)
			clipped = true
			break
		}
		if !clipped {
			// The operand is valid; its CHORDING is too coarse to prove the
			// region's topology. requireLoopClearance proves distinct loops
			// apart, but a single loop's own chords can still cross over a
			// narrow concave feature, and that lands here. So this is an
			// expected undecided outcome, not an evaluator failure
			// (docs/interference-design.md §7.1): read-only interference takes
			// it as an undecided pair, while public Tessellate still sees
			// ErrDegenerate through Unwrap.
			return nil, &tessellationExpectedError{err: fmt.Errorf(`%w: the chorded cap boundary self-intersects; tessellate at a finer tolerance`, ErrDegenerate)}
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
func earBlocked(ctx context.Context, pts []Point2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	for j := range n {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		prev, next := pts[idx[(j-1+n)%n]], pts[idx[(j+1)%n]]
		if cross2(prev, p, next) >= 0 {
			continue
		}
		if pointInTri(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}

// wedgeContains reports whether the direction from p to m lies strictly inside
// the interior wedge the counter-clockwise corner (q, p, r) spans. q is the
// corner's predecessor and r its successor, so the wedge runs counter-clockwise
// from the direction of r round to the direction of q; a corner turning left
// spans less than half a turn and a corner turning right spans more, which is
// why the two branches differ.
func wedgeContains(q, p, r, m Point2) bool {
	ax, ay := q.U-p.U, q.V-p.V
	bx, by := r.U-p.U, r.V-p.V
	dx, dy := m.U-p.U, m.V-p.V
	crossBA := bx*ay - by*ax
	crossBD := bx*dy - by*dx
	crossDA := dx*ay - dy*ax
	if crossBA > 0 {
		return crossBD > 0 && crossDA > 0
	}
	return crossBD > 0 || crossDA > 0
}

// bridgeOccurrence picks WHICH occurrence of the chosen bridge vertex the hole
// splices into. A vertex two earlier bridges already landed on appears several
// times in merged, each occurrence owning its own share of the interior
// directions at that point; those shares are disjoint and cover the interior
// between them, so the occurrence whose wedge holds the direction to m is the
// only one that keeps the merged polygon from crossing itself there. Splicing
// at any other occurrence leaves two bridge channels interleaved at the shared
// point — a boundary earClip then, correctly, cannot reduce.
//
// A vertex that appears once is left exactly as the caller chose it: there is no
// second occurrence to prefer, so this makes no judgement about it and cannot
// refuse a bridge the construction already proved visible.
func bridgeOccurrence(pts []Point2, merged []int, bridge int, m Point2) (int, bool) {
	n := len(merged)
	v := merged[bridge]
	occurrences := 0
	for k := range n {
		if merged[k] == v {
			occurrences++
		}
	}
	if occurrences < 2 {
		return bridge, true
	}
	for k := range n {
		if merged[k] != v {
			continue
		}
		q, p, r := pts[merged[(k-1+n)%n]], pts[merged[k]], pts[merged[(k+1)%n]]
		if wedgeContains(q, p, r, m) {
			return k, true
		}
	}
	return 0, false
}
