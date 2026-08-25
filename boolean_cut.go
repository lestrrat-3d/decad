package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"
)

// This file subdivides one facet along its exact contact segments
// (docs/evaluator-design.md §9). The segments assemble into chains — the
// intersection curve's trace across the facet — every chain endpoint lies on
// the facet's own boundary (an interior endpoint would mean the curve stops
// mid-surface, impossible between closed solids), and a closed chain (the
// other solid punching through the facet's interior) is first opened by a
// split line chosen to pass through no existing vertex. The facet is then
// split polygon-by-polygon along the chains, each resulting region is exactly
// one side of the other solid, and exact ear clipping triangulates it. All of
// it is rational arithmetic on a 2D projection of the facet's own plane —
// nothing is ever snapped or welded by distance.

// xseg is one exact contact segment assigned to a facet, with the partner
// facet whose plane it lies in, and whether each endpoint lies on THIS
// facet's own boundary.
type xseg struct {
	a, b             xpt
	aOnEdge, bOnEdge bool
	partner          [3]r3.Vec
	// viaParity marks a contact whose region classification may NOT be read
	// off the partner facet's plane: the segment runs along an EDGE of the
	// other operand, whose boundary there is the DIHEDRAL between two facets,
	// and one plane of a dihedral decides nothing (a reflex hinge reverses the
	// reading). Such a region falls back to exact parity, which is global and
	// cannot be fooled by a local plane.
	viaParity bool
}

// cutRegion is one classified-to-be region of a subdivided facet: its exact
// triangles, and its classification anchor — the partner facet whose plane
// the region's bounding chain lies in, with a probe strictly inside the
// region triangle adjacent to that chain. A region no chain bounds (a corner
// piece an artificial split line severed) carries no anchor and is
// classified by parity instead.
type cutRegion struct {
	tris      [][3]xpt
	partner   [3]r3.Vec
	hasAnchor bool
	probe     xpt
}

// cutVert is one subdivision vertex: its exact 2D projection, its exact 3D
// point, and whether it lies on a region boundary (the facet's own edges or
// a split line) — the points chains break at.
type cutVert struct {
	p2       xp2
	p3       xpt
	boundary bool
}

// cutEdge is one chain edge between two subdivision vertices.
type cutEdge struct {
	a, b    int
	partner [3]r3.Vec
	// viaParity carries xseg.viaParity: this chain edge anchors no region.
	viaParity bool
}

// chainAnchor is what a chain edge offers a region for classification: the
// partner facet whose plane side is the answer — unless viaParity, in which
// case the edge anchors nothing and the region falls back to exact parity.
type chainAnchor struct {
	partner   [3]r3.Vec
	viaParity bool
}

// triCutter is the working state of one facet's subdivision.
type triCutter struct {
	verts []cutVert
	index map[string]int
	edges []cutEdge
	swap  bool // projection coordinate swap that keeps the facet CCW
	u, v  int  // projection axes
	work  *workBudget
}

func (tc *triCutter) proj(p xpt) xp2 {
	a := ratCoordOf(p, tc.u)
	b := ratCoordOf(p, tc.v)
	if tc.swap {
		a, b = b, a
	}
	return newXP2(a, b)
}

// addVert interns a vertex by its exact 2D identity; boundary is sticky.
func (tc *triCutter) addVert(p2 xp2, p3 xpt, boundary bool) int {
	k := p2.key2()
	if i, ok := tc.index[k]; ok {
		if boundary {
			tc.verts[i].boundary = true
		}
		return i
	}
	tc.index[k] = len(tc.verts)
	tc.verts = append(tc.verts, cutVert{p2: p2, p3: p3, boundary: boundary})
	return len(tc.verts) - 1
}

// cutTriangle subdivides the facet along its contact segments and returns
// the classified regions.
func cutTriangle(ctx context.Context, xtri [3]xpt, normal xpt, segs []xseg) ([]cutRegion, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	tc := &triCutter{index: map[string]int{}, work: newWorkBudget(ctx)}
	tc.u, tc.v = projAxes(normal)

	// Keep the projected facet counter-clockwise, so polygon areas and ear
	// clipping read the facet's own orientation.
	corner := func(p xpt) xp2 { return newXP2(ratCoordOf(p, tc.u), ratCoordOf(p, tc.v)) }
	if cross2xSign(corner(xtri[0]), corner(xtri[1]), corner(xtri[2])) < 0 {
		tc.swap = true
	}
	var cornerIdx [3]int
	for i := range 3 {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		cornerIdx[i] = tc.addVert(tc.proj(xtri[i]), xtri[i], true)
	}

	// Intern the contact segments as chain edges.
	seen := map[[2]int]struct{}{}
	for _, s := range segs {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		ia := tc.addVert(tc.proj(s.a), s.a, s.aOnEdge)
		ib := tc.addVert(tc.proj(s.b), s.b, s.bOnEdge)
		if ia == ib {
			continue
		}
		k := [2]int{min(ia, ib), max(ia, ib)}
		if _, ok := seen[k]; ok {
			continue
		}
		seen[k] = struct{}{}
		tc.edges = append(tc.edges, cutEdge{a: ia, b: ib, partner: s.partner, viaParity: s.viaParity})
	}
	if len(tc.edges) == 0 {
		// Every contact collapsed to a point: the facet is whole, classified
		// by parity.
		probe := xCentroid(xtri[0], xtri[1], xtri[2])
		return []cutRegion{{tris: [][3]xpt{xtri}, probe: probe}}, nil
	}

	// Open every closed chain with split lines, then split the facet into
	// convex pieces along those same lines.
	lines, chains, err := tc.openLoops()
	if err != nil {
		return nil, err
	}
	pieces := [][]int{{cornerIdx[0], cornerIdx[1], cornerIdx[2]}}
	for _, c := range lines {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		var next [][]int
		for _, piece := range pieces {
			left, right, err := tc.splitConvexByU(piece, c)
			if err != nil {
				return nil, err
			}
			if left != nil {
				next = append(next, left)
			}
			if right != nil {
				next = append(next, right)
			}
		}
		pieces = next
	}

	// Assign each chain to the piece containing it, then split the pieces
	// along their chains into the final region polygons.
	byPiece := make([][]chainPath, len(pieces))
	for _, ch := range chains {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		probe, err := tc.chainProbe(ch)
		if err != nil {
			return nil, err
		}
		placed := false
		for pi, piece := range pieces {
			pts, err := tc.polyPoints(piece)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := pointInPoly2(tc.work, pts, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a contact chain lies on a subdivision boundary`, ErrBooleanFailed)
			}
			if inside {
				byPiece[pi] = append(byPiece[pi], ch)
				placed = true
				break
			}
		}
		if !placed {
			return nil, fmt.Errorf(`%w: a contact chain escaped every subdivision piece`, ErrBooleanFailed)
		}
	}

	chainEdges := map[[2]int]chainAnchor{}
	for _, e := range tc.edges {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		chainEdges[[2]int{min(e.a, e.b), max(e.a, e.b)}] = chainAnchor{partner: e.partner, viaParity: e.viaParity}
	}

	var regions []cutRegion
	for pi, piece := range pieces {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		polys, err := tc.splitByChains(piece, byPiece[pi])
		if err != nil {
			return nil, err
		}
		for _, poly := range polys {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			reg, err := tc.regionOf(poly, chainEdges)
			if err != nil {
				return nil, err
			}
			regions = append(regions, reg)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return regions, nil
}

// chainPath is one open chain: an ordered vertex walk whose endpoints lie on
// region boundaries.
type chainPath struct {
	verts []int
}

// chainProbe is an exact point strictly inside the chain's carrier piece:
// an interior chain vertex when one exists, else the midpoint of the first
// edge.
func (tc *triCutter) chainProbe(ch chainPath) (xp2, error) {
	for _, vi := range ch.verts[1 : len(ch.verts)-1] {
		if err := tc.work.step(); err != nil {
			return xp2{}, err
		}
		if !tc.verts[vi].boundary {
			return tc.verts[vi].p2, nil
		}
	}
	a, b := tc.verts[ch.verts[0]].p2, tc.verts[ch.verts[1]].p2
	half := big.NewRat(1, 2)
	return newXP2(
		new(big.Rat).Mul(half, new(big.Rat).Add(a.u, b.u)),
		new(big.Rat).Mul(half, new(big.Rat).Add(a.v, b.v)),
	), nil
}

func (tc *triCutter) polyPoints(poly []int) ([]xp2, error) {
	out := make([]xp2, len(poly))
	for i, vi := range poly {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		out[i] = tc.verts[vi].p2
	}
	return out, nil
}

// openLoops builds the chain decomposition, and while any chain is a closed
// loop, adds a vertical split line through the loop — through no existing
// vertex, so every crossing is transversal — splitting the chain edges it
// crosses. It returns the split lines and the final all-open chains.
func (tc *triCutter) openLoops() ([]*big.Rat, []chainPath, error) {
	var lines []*big.Rat
	guard := len(tc.edges) + 8
	for iter := 0; ; iter++ {
		if err := tc.work.step(); err != nil {
			return nil, nil, err
		}
		if iter > guard {
			return nil, nil, fmt.Errorf(`%w: loop opening did not converge`, ErrBooleanFailed)
		}
		chains, loops, err := tc.buildChains()
		if err != nil {
			return nil, nil, err
		}
		if len(loops) == 0 {
			return lines, chains, nil
		}
		c, err := tc.chooseSplitU(loops[0])
		if err != nil {
			return nil, nil, err
		}
		if err := tc.splitEdgesAtU(c); err != nil {
			return nil, nil, err
		}
		lines = append(lines, c)
	}
}

// buildChains walks the contact edges into maximal chains, breaking at
// boundary vertices; a walk that returns to its start through interior
// vertices only is a closed loop. A branching interior vertex means the
// intersection curves cross on this facet — a contact the predicates refuse.
func (tc *triCutter) buildChains() ([]chainPath, []chainPath, error) {
	adj := map[int][]int{} // vertex -> edge indices
	for ei, e := range tc.edges {
		if err := tc.work.step(); err != nil {
			return nil, nil, err
		}
		adj[e.a] = append(adj[e.a], ei)
		adj[e.b] = append(adj[e.b], ei)
	}
	for vi, list := range adj {
		if err := tc.work.step(); err != nil {
			return nil, nil, err
		}
		if !tc.verts[vi].boundary && len(list) > 2 {
			return nil, nil, errUnclassifiableContact(`intersection curves branch on a facet`)
		}
		if !tc.verts[vi].boundary && len(list) == 1 {
			return nil, nil, fmt.Errorf(`%w: a contact chain dangles mid-facet`, ErrBooleanFailed)
		}
	}
	otherEnd := func(ei, vi int) int {
		if tc.edges[ei].a == vi {
			return tc.edges[ei].b
		}
		return tc.edges[ei].a
	}
	used := make([]bool, len(tc.edges))
	var chains, loops []chainPath
	// A chain interior vertex continues the walk exactly when it is
	// interior with degree two.
	continues := func(vi int) bool { return !tc.verts[vi].boundary && len(adj[vi]) == 2 }
	nextEdge := func(vi, from int) (int, error) {
		for _, ei := range adj[vi] {
			if err := tc.work.step(); err != nil {
				return -1, err
			}
			if ei != from && !used[ei] {
				return ei, nil
			}
		}
		return -1, nil
	}
	for start := range tc.edges {
		if err := tc.work.step(); err != nil {
			return nil, nil, err
		}
		if used[start] {
			continue
		}
		used[start] = true
		path := []int{tc.edges[start].a, tc.edges[start].b}
		isLoop := false
		// Extend forward from the tail, then backward from the head.
		for continues(path[len(path)-1]) {
			if err := tc.work.step(); err != nil {
				return nil, nil, err
			}
			ei, err := nextEdge(path[len(path)-1], -1)
			if err != nil {
				return nil, nil, err
			}
			if ei < 0 {
				isLoop = true
				break
			}
			used[ei] = true
			path = append(path, otherEnd(ei, path[len(path)-1]))
		}
		if !isLoop {
			for continues(path[0]) {
				if err := tc.work.step(); err != nil {
					return nil, nil, err
				}
				ei, err := nextEdge(path[0], -1)
				if err != nil {
					return nil, nil, err
				}
				if ei < 0 {
					break
				}
				used[ei] = true
				next := otherEnd(ei, path[0])
				path = append(path, 0)
				for i := len(path) - 1; i > 0; i-- {
					if err := tc.work.step(); err != nil {
						return nil, nil, err
					}
					path[i] = path[i-1]
				}
				path[0] = next
			}
		}
		if isLoop || path[0] == path[len(path)-1] {
			loops = append(loops, chainPath{verts: path})
			continue
		}
		chains = append(chains, chainPath{verts: path})
	}
	return chains, loops, nil
}

// chooseSplitU picks a vertical line u = c strictly inside the loop's span
// that passes through no existing vertex, bisecting toward the span's low
// end until it clears — finitely many vertex ordinates, so it terminates.
func (tc *triCutter) chooseSplitU(loop chainPath) (*big.Rat, error) {
	lo, hi := tc.verts[loop.verts[0]].p2.u, tc.verts[loop.verts[0]].p2.u
	for _, vi := range loop.verts {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		u := tc.verts[vi].p2.u
		if u.Cmp(lo) < 0 {
			lo = u
		}
		if u.Cmp(hi) > 0 {
			hi = u
		}
	}
	if lo.Cmp(hi) == 0 {
		return nil, fmt.Errorf(`%w: a closed contact chain has no width`, ErrBooleanFailed)
	}
	half := big.NewRat(1, 2)
	c := new(big.Rat).Mul(half, new(big.Rat).Add(lo, hi))
	for range 64 {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		hit := false
		for _, v := range tc.verts {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			if v.p2.u.Cmp(c) == 0 {
				hit = true
				break
			}
		}
		if !hit {
			return c, nil
		}
		c = new(big.Rat).Mul(half, new(big.Rat).Add(lo, c))
	}
	return nil, fmt.Errorf(`%w: no vertex-free split line found`, ErrBooleanFailed)
}

// splitEdgesAtU splits every chain edge strictly straddling u = c at the
// exact crossing, which becomes a boundary vertex — a chain break.
func (tc *triCutter) splitEdgesAtU(c *big.Rat) error {
	var out []cutEdge
	for _, e := range tc.edges {
		if err := tc.work.step(); err != nil {
			return err
		}
		ua := tc.verts[e.a].p2.u
		ub := tc.verts[e.b].p2.u
		sa := ua.Cmp(c)
		sb := ub.Cmp(c)
		if sa*sb >= 0 {
			out = append(out, e)
			continue
		}
		t := new(big.Rat).Quo(new(big.Rat).Sub(c, ua), new(big.Rat).Sub(ub, ua))
		p2 := newXP2(
			new(big.Rat).Set(c),
			new(big.Rat).Add(tc.verts[e.a].p2.v, new(big.Rat).Mul(t, new(big.Rat).Sub(tc.verts[e.b].p2.v, tc.verts[e.a].p2.v))),
		)
		p3 := xlerp(tc.verts[e.a].p3, tc.verts[e.b].p3, t.Num(), t.Denom())
		mid := tc.addVert(p2, p3, true)
		out = append(out,
			cutEdge{a: e.a, b: mid, partner: e.partner, viaParity: e.viaParity},
			cutEdge{a: mid, b: e.b, partner: e.partner, viaParity: e.viaParity})
	}
	tc.edges = out
	return nil
}

// splitConvexByU splits a convex polygon along u = c. A side that would be
// empty returns nil, leaving the piece whole on the other side.
func (tc *triCutter) splitConvexByU(piece []int, c *big.Rat) ([]int, []int, error) {
	n := len(piece)
	var left, right []int
	for i := range n {
		if err := tc.work.step(); err != nil {
			return nil, nil, err
		}
		vi, vj := piece[i], piece[(i+1)%n]
		si := tc.verts[vi].p2.u.Cmp(c)
		sj := tc.verts[vj].p2.u.Cmp(c)
		if si <= 0 {
			left = append(left, vi)
		}
		if si >= 0 {
			right = append(right, vi)
		}
		if si*sj >= 0 {
			continue
		}
		ua := tc.verts[vi].p2.u
		ub := tc.verts[vj].p2.u
		t := new(big.Rat).Quo(new(big.Rat).Sub(c, ua), new(big.Rat).Sub(ub, ua))
		p2 := newXP2(
			new(big.Rat).Set(c),
			new(big.Rat).Add(tc.verts[vi].p2.v, new(big.Rat).Mul(t, new(big.Rat).Sub(tc.verts[vj].p2.v, tc.verts[vi].p2.v))),
		)
		p3 := xlerp(tc.verts[vi].p3, tc.verts[vj].p3, t.Num(), t.Denom())
		mid := tc.addVert(p2, p3, true)
		left = append(left, mid)
		right = append(right, mid)
	}
	leftPts, err := tc.polyPoints(left)
	if err != nil {
		return nil, nil, err
	}
	leftArea, err := polyArea2(tc.work, leftPts)
	if err != nil {
		return nil, nil, err
	}
	if len(left) < 3 || leftArea.Sign() <= 0 {
		left = nil
	}
	rightPts, err := tc.polyPoints(right)
	if err != nil {
		return nil, nil, err
	}
	rightArea, err := polyArea2(tc.work, rightPts)
	if err != nil {
		return nil, nil, err
	}
	if len(right) < 3 || rightArea.Sign() <= 0 {
		right = nil
	}
	return left, right, nil
}

// splitByChains splits one piece polygon along its assigned chains,
// returning the final region polygons.
func (tc *triCutter) splitByChains(piece []int, chains []chainPath) ([][]int, error) {
	type work struct {
		poly   []int
		chains []chainPath
	}
	stack := []work{{poly: piece, chains: chains}}
	var out [][]int
	for len(stack) > 0 {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		w := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if len(w.chains) == 0 {
			out = append(out, w.poly)
			continue
		}
		ch := w.chains[0]
		rest := w.chains[1:]
		poly, err := tc.insertOnBoundary(w.poly, ch.verts[0])
		if err != nil {
			return nil, err
		}
		poly, err = tc.insertOnBoundary(poly, ch.verts[len(ch.verts)-1])
		if err != nil {
			return nil, err
		}
		i, err := tc.indexOf(poly, ch.verts[0])
		if err != nil {
			return nil, err
		}
		j, err := tc.indexOf(poly, ch.verts[len(ch.verts)-1])
		if err != nil {
			return nil, err
		}
		if i < 0 || j < 0 || i == j {
			return nil, fmt.Errorf(`%w: a chain endpoint is missing from its region boundary`, ErrBooleanFailed)
		}
		// Walk the boundary i→j one way and j→i the other, closing each side
		// with the chain path so both stay counter-clockwise.
		interior := ch.verts[1 : len(ch.verts)-1]
		var polyA []int
		for k := i; ; k = (k + 1) % len(poly) {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, poly[k])
			if k == j {
				break
			}
		}
		for _, vi := range slices.Backward(interior) {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, vi)
		}
		var polyB []int
		for k := j; ; k = (k + 1) % len(poly) {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, poly[k])
			if k == i {
				break
			}
		}
		for _, vi := range interior {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, vi)
		}
		ptsA, err := tc.polyPoints(polyA)
		if err != nil {
			return nil, err
		}
		areaA, err := polyArea2(tc.work, ptsA)
		if err != nil {
			return nil, err
		}
		ptsB, err := tc.polyPoints(polyB)
		if err != nil {
			return nil, err
		}
		areaB, err := polyArea2(tc.work, ptsB)
		if err != nil {
			return nil, err
		}
		if areaA.Sign() <= 0 || areaB.Sign() <= 0 {
			return nil, fmt.Errorf(`%w: a chain split produced a non-positive region`, ErrBooleanFailed)
		}
		wa := work{poly: polyA}
		wb := work{poly: polyB}
		for _, rc := range rest {
			if err := tc.work.step(); err != nil {
				return nil, err
			}
			probe, err := tc.chainProbe(rc)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := pointInPoly2(tc.work, ptsA, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a chain lies on a freshly split boundary`, ErrBooleanFailed)
			}
			if inside {
				wa.chains = append(wa.chains, rc)
				continue
			}
			inside, onBoundary, err = pointInPoly2(tc.work, ptsB, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary || !inside {
				return nil, fmt.Errorf(`%w: a chain escaped both split regions`, ErrBooleanFailed)
			}
			wb.chains = append(wb.chains, rc)
		}
		stack = append(stack, wa, wb)
	}
	return out, nil
}

// insertOnBoundary ensures the vertex appears on the polygon boundary,
// splicing it into the edge whose interior it lies on. Identity is the
// vertex id — subdivision vertices are interned by exact coordinates.
func (tc *triCutter) insertOnBoundary(poly []int, vi int) ([]int, error) {
	found, err := tc.indexOf(poly, vi)
	if err != nil {
		return nil, err
	}
	if found >= 0 {
		return poly, nil
	}
	p := tc.verts[vi].p2
	n := len(poly)
	for i := range n {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		a := tc.verts[poly[i]].p2
		b := tc.verts[poly[(i+1)%n]].p2
		_, interior := onSegment2(a, b, p)
		if !interior {
			continue
		}
		out := make([]int, 0, n+1)
		out = append(out, poly[:i+1]...)
		out = append(out, vi)
		out = append(out, poly[i+1:]...)
		return out, nil
	}
	return nil, fmt.Errorf(`%w: a chain endpoint lies on no region boundary`, ErrBooleanFailed)
}

func (tc *triCutter) indexOf(poly []int, vi int) (int, error) {
	for i, v := range poly {
		if err := tc.work.step(); err != nil {
			return -1, err
		}
		if v == vi {
			return i, nil
		}
	}
	return -1, nil
}

// regionOf triangulates one final region polygon and finds its
// classification anchor: the region triangle adjacent to a chain edge, whose
// exact centroid probes the partner facet's plane side.
func (tc *triCutter) regionOf(poly []int, chainEdges map[[2]int]chainAnchor) (cutRegion, error) {
	points, err := tc.collectP2()
	if err != nil {
		return cutRegion{}, err
	}
	tris2, err := earClipX(tc.work, points, poly)
	if err != nil {
		return cutRegion{}, err
	}
	if len(tris2) == 0 {
		return cutRegion{}, fmt.Errorf(`%w: a region triangulated to nothing`, ErrBooleanFailed)
	}
	reg := cutRegion{}
	for _, t := range tris2 {
		if err := tc.work.step(); err != nil {
			return cutRegion{}, err
		}
		corners := [3]xpt{tc.verts[t[0]].p3, tc.verts[t[1]].p3, tc.verts[t[2]].p3}
		reg.tris = append(reg.tris, corners)
		if reg.hasAnchor {
			continue
		}
		for k := range 3 {
			if err := tc.work.step(); err != nil {
				return cutRegion{}, err
			}
			key := [2]int{min(t[k], t[(k+1)%3]), max(t[k], t[(k+1)%3])}
			anchor, ok := chainEdges[key]
			if !ok || anchor.viaParity {
				continue
			}
			reg.hasAnchor = true
			reg.partner = anchor.partner
			reg.probe = xCentroid(corners[0], corners[1], corners[2])
			break
		}
	}
	if !reg.hasAnchor {
		first := reg.tris[0]
		reg.probe = xCentroid(first[0], first[1], first[2])
	}
	return reg, nil
}

// collectP2 is the projected-point view of the vertex table, as earClipX
// wants it.
func (tc *triCutter) collectP2() ([]xp2, error) {
	out := make([]xp2, len(tc.verts))
	for i, v := range tc.verts {
		if err := tc.work.step(); err != nil {
			return nil, err
		}
		out[i] = v.p2
	}
	return out, nil
}
