package meshbool

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file subdivides one facet along its exact contact segments
// (agent-docs/evaluator-design.md §9). The segments assemble into chains — the
// intersection curve's trace across the facet — every chain endpoint lies on
// the facet's own boundary (an interior endpoint would mean the curve stops
// mid-surface, impossible between closed solids), and a closed chain (the
// other solid punching through the facet's interior) is first opened by a
// split line chosen to pass through no existing vertex. The facet is then
// split polygon-by-polygon along the chains, each resulting region is exactly
// one side of the other solid, and exact ear clipping triangulates it. All of
// it is rational arithmetic on a 2D projection of the facet's own plane —
// nothing is ever snapped or welded by distance.

// Xseg is one exact contact segment assigned to a facet, with the partner
// facet whose plane it lies in, and whether each endpoint lies on THIS
// facet's own boundary.
type Xseg struct {
	A, B             proof.Xpt
	AOnEdge, BOnEdge bool
	Partner          [3]r3.Vec
	// viaParity marks a contact whose region classification may NOT be read
	// off the partner facet's plane: the segment runs along an EDGE of the
	// other operand, whose boundary there is the DIHEDRAL between two facets,
	// and one plane of a dihedral decides nothing (a reflex hinge reverses the
	// reading). Such a region falls back to exact parity, which is global and
	// cannot be fooled by a local plane.
	ViaParity bool
}

// CutRegion is one classified-to-be region of a subdivided facet: its exact
// triangles, and its classification anchor — the partner facet whose plane
// the region's bounding chain lies in, with a probe strictly inside the
// region triangle adjacent to that chain. A region no chain bounds (a corner
// piece an artificial split line severed) carries no anchor and is
// classified by parity instead.
type CutRegion struct {
	Tris      [][3]proof.Xpt
	Partner   [3]r3.Vec
	HasAnchor bool
	Probe     proof.Xpt
}

// CutVert is one subdivision vertex: its exact 2D projection, its exact 3D
// point, and whether it lies on a region boundary (the facet's own edges or
// a split line) — the points chains break at.
type CutVert struct {
	P2       Xp2
	P3       proof.Xpt
	Boundary bool
}

// CutEdge is one chain edge between two subdivision vertices.
type CutEdge struct {
	A, B    int
	Partner [3]r3.Vec
	// viaParity carries Xseg.viaParity: this chain edge anchors no region.
	ViaParity bool
}

// ChainAnchor is what a chain edge offers a region for classification: the
// partner facet whose plane side is the answer — unless viaParity, in which
// case the edge anchors nothing and the region falls back to exact parity.
type ChainAnchor struct {
	Partner   [3]r3.Vec
	ViaParity bool
}

// TriCutter is the working state of one facet's subdivision. The zero value
// has no vertices; Work must be set before any method that steps it.
type TriCutter struct {
	Verts        []CutVert
	Edges        []CutEdge
	Swap         bool // projection coordinate swap that keeps the facet CCW
	U, V         int  // projection axes
	Work         *proofbound.WorkBudget
	projections  *cutProjectionScratch
	index        cutVertIndex
	raw          cutRawIndex
	seen         map[[2]int]struct{}
	anchors      map[[2]int]ChainAnchor
	regionPoints []Xp2
}

func (tc *TriCutter) Proj(p proof.Xpt) Xp2 {
	u, v := tc.U, tc.V
	if tc.Swap {
		u, v = v, u
	}
	return tc.project(p, u, v)
}

func (tc *TriCutter) project(p proof.Xpt, u, v int) Xp2 {
	if tc.projections == nil {
		return NewXP2FromXpt(p, u, v)
	}
	if p.W.Sign() > 0 {
		return newDeferredXP2(XIntCoordOf(p, u), XIntCoordOf(p, v), p.W, tc.projections.rat(), tc.projections.rat())
	}
	return newXP2FromXptInto(p, u, v, tc.projections.rat(), tc.projections.rat())
}

// AddVert interns a vertex by its exact 2D identity (cutVertIndex); boundary
// is sticky.
func (tc *TriCutter) AddVert(p2 Xp2, p3 proof.Xpt, boundary bool) int {
	h := cutProjectionHash(p2, tc.index.floatKeys)
	if i, ok := tc.index.lookup(tc.Verts, h, p2); ok {
		if boundary {
			tc.Verts[i].Boundary = true
		}
		return i
	}
	tc.index.addForPoint(h, len(tc.Verts), p2)
	tc.Verts = append(tc.Verts, CutVert{P2: p2, P3: p3, Boundary: boundary})
	return len(tc.Verts) - 1
}

// CutTriangle subdivides the facet along its contact segments and returns
// the classified regions.
func CutTriangle(ctx context.Context, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg) ([]CutRegion, error) {
	return cutTriangle(ctx, xtri, normal, segs, nil)
}

func cutTriangle(ctx context.Context, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg, projections *cutProjectionScratch) ([]CutRegion, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return newTriCutter(ctx, projections).cut(ctx, xtri, normal, segs)
}

// cut is cutTriangle's work on a fresh cutter, every step charged to tc.Work.
func (tc *TriCutter) cut(ctx context.Context, xtri [3]proof.Xpt, normal proof.Xpt, segs []Xseg) ([]CutRegion, error) {
	tc.U, tc.V = ProjAxes(normal)
	cornerIdx, err := tc.internFacet(xtri, segs)
	if err != nil {
		return nil, err
	}
	if len(tc.Edges) == 0 {
		// Every contact collapsed to a point: the facet is whole, classified
		// by parity.
		probe := XCentroid(xtri[0], xtri[1], xtri[2])
		return []CutRegion{{Tris: [][3]proof.Xpt{xtri}, Probe: probe}}, nil
	}

	// Open every closed chain with split lines, then split the facet into
	// convex pieces along those same lines.
	lines, chains, err := tc.OpenLoops()
	if err != nil {
		return nil, err
	}
	pieces := [][]int{{cornerIdx[0], cornerIdx[1], cornerIdx[2]}}
	for _, c := range lines {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		var next [][]int
		for _, piece := range pieces {
			left, right, err := tc.SplitConvexByU(piece, c)
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
	// along their chains into the final region polygons. The vertex table is
	// final from here on (nothing below adds a vertex or changes a boundary
	// flag), so each chain carries the probe found here into SplitByChains.
	byPiece := make([][]ChainPath, len(pieces))
	for _, ch := range chains {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		probe, steps, err := tc.chainProbe(ch)
		if err != nil {
			return nil, err
		}
		ch.probe, ch.probeSteps, ch.hasProbe = probe, steps, true
		placed := false
		for pi, piece := range pieces {
			pts, err := tc.PolyPoints(piece)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := PointInPoly2(tc.Work, pts, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a contact chain lies on a subdivision boundary`, decaderr.ErrBooleanFailed)
			}
			if inside {
				byPiece[pi] = append(byPiece[pi], ch)
				placed = true
				break
			}
		}
		if !placed {
			return nil, fmt.Errorf(`%w: a contact chain escaped every subdivision piece`, decaderr.ErrBooleanFailed)
		}
	}

	chainEdges := tc.anchors
	for _, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		chainEdges[[2]int{min(e.A, e.B), max(e.A, e.B)}] = ChainAnchor{Partner: e.Partner, ViaParity: e.ViaParity}
	}

	var regions []CutRegion
	for pi, piece := range pieces {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		polys, err := tc.SplitByChains(piece, byPiece[pi])
		if err != nil {
			return nil, err
		}
		for _, poly := range polys {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			reg, err := tc.RegionOf(poly, chainEdges)
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

// internFacet interns the facet's corners and its contact segments, the
// latter as chain edges, and returns the corners' vertex indices. It sets
// Swap, so the projected facet is counter-clockwise and polygon areas and ear
// clipping read the facet's own orientation. tc.U and tc.V must be set.
//
// Each corner is projected once. Projecting on the swapped axes builds the
// same rationals, floats and homogeneous integers with the U and V fields
// exchanged, which is what xp2Swapped does to the orientation projection.
// Segment endpoints intern through addPoint.
func (tc *TriCutter) internFacet(xtri [3]proof.Xpt, segs []Xseg) ([3]int, error) {
	var corners [3]Xp2
	for i, p := range xtri {
		corners[i] = tc.project(p, tc.U, tc.V)
	}
	if Cross2xSign(corners[0], corners[1], corners[2]) < 0 {
		tc.Swap = true
		for i := range corners {
			corners[i] = xp2Swapped(corners[i])
		}
	}
	var cornerIdx [3]int
	for i, p := range xtri {
		if err := tc.Work.Step(); err != nil {
			return cornerIdx, err
		}
		c := corners[i]
		cornerIdx[i] = tc.AddVert(c, p, true)
		tc.raw.add(rawHash(c.Hu, c.Hv, c.Hw), c.Hu, c.Hv, c.Hw, cornerIdx[i])
	}

	seen := tc.seen
	if seen == nil {
		seen = map[[2]int]struct{}{}
		tc.seen = seen
	}
	for _, s := range segs {
		if err := tc.Work.Step(); err != nil {
			return cornerIdx, err
		}
		ia := tc.addPoint(s.A, s.AOnEdge)
		ib := tc.addPoint(s.B, s.BOnEdge)
		if ia == ib {
			continue
		}
		k := [2]int{min(ia, ib), max(ia, ib)}
		if _, ok := seen[k]; ok {
			continue
		}
		seen[k] = struct{}{}
		tc.Edges = append(tc.Edges, CutEdge{A: ia, B: ib, Partner: s.Partner, ViaParity: s.ViaParity})
	}
	return cornerIdx, nil
}

// xp2Swapped exchanges a projection's U and V fields.
func xp2Swapped(p Xp2) Xp2 {
	p.U, p.V = p.V, p.U
	p.Fu, p.Fv = p.Fv, p.Fu
	p.Hu, p.Hv = p.Hv, p.Hu
	return p
}

// ChainPath is one open chain: an ordered vertex walk whose endpoints lie on
// region boundaries.
type ChainPath struct {
	Verts []int
	// probe is ChainProbe's answer for this chain and probeSteps the number
	// of budget steps it took, recorded by cutTriangle once the vertex table
	// is final, when hasProbe is set. ChainProbe reads only that table, so
	// the recorded answer is the one a new call would return.
	probe      Xp2
	probeSteps int
	hasProbe   bool
}

// ChainProbe is an exact point strictly inside the chain's carrier piece:
// an interior chain vertex when one exists, else the midpoint of the first
// edge.
func (tc *TriCutter) ChainProbe(ch ChainPath) (Xp2, error) {
	p, _, err := tc.chainProbe(ch)
	return p, err
}

// chainProbe is ChainProbe, also returning the number of budget steps it
// took.
func (tc *TriCutter) chainProbe(ch ChainPath) (Xp2, int, error) {
	steps := 0
	for _, vi := range ch.Verts[1 : len(ch.Verts)-1] {
		if err := tc.Work.Step(); err != nil {
			return Xp2{}, steps, err
		}
		steps++
		if !tc.Verts[vi].Boundary {
			return tc.Verts[vi].P2, steps, nil
		}
	}
	a, b := tc.Verts[ch.Verts[0]].P2, tc.Verts[ch.Verts[1]].P2
	if a.deferredRats && b.deferredRats {
		return deferredMidpointXP2(a, b), steps, nil
	}
	materializeXP2(a)
	materializeXP2(b)
	half := big.NewRat(1, 2)
	return NewXP2(
		proof.MulRat(new(big.Rat), half, proof.AddRat(new(big.Rat), a.U, b.U)),
		proof.MulRat(new(big.Rat), half, proof.AddRat(new(big.Rat), a.V, b.V)),
	), steps, nil
}

// recordedChainProbe is ChainProbe for a chain cutTriangle has recorded a
// probe for: it takes the same budget steps and returns the recorded point.
// The point is shared between calls; its readers never mutate it (Xp2). A
// chain without a record takes ChainProbe.
func (tc *TriCutter) recordedChainProbe(ch ChainPath) (Xp2, error) {
	if !ch.hasProbe {
		return tc.ChainProbe(ch)
	}
	for range ch.probeSteps {
		if err := tc.Work.Step(); err != nil {
			return Xp2{}, err
		}
	}
	return ch.probe, nil
}

func (tc *TriCutter) PolyPoints(poly []int) ([]Xp2, error) {
	out := make([]Xp2, len(poly))
	for i, vi := range poly {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		out[i] = tc.Verts[vi].P2
	}
	return out, nil
}

// OpenLoops builds the chain decomposition, and while any chain is a closed
// loop, adds a vertical split line through the loop — through no existing
// vertex, so every crossing is transversal — splitting the chain edges it
// crosses. It returns the split lines and the final all-open chains.
func (tc *TriCutter) OpenLoops() ([]*big.Rat, []ChainPath, error) {
	var lines []*big.Rat
	guard := len(tc.Edges) + 8
	for iter := 0; ; iter++ {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if iter > guard {
			return nil, nil, fmt.Errorf(`%w: loop opening did not converge`, decaderr.ErrBooleanFailed)
		}
		chains, loops, err := tc.BuildChains()
		if err != nil {
			return nil, nil, err
		}
		if len(loops) == 0 {
			return lines, chains, nil
		}
		c, err := tc.ChooseSplitU(loops[0])
		if err != nil {
			return nil, nil, err
		}
		if err := tc.SplitEdgesAtU(c); err != nil {
			return nil, nil, err
		}
		lines = append(lines, c)
	}
}

// BuildChains walks the contact edges into maximal chains, breaking at
// boundary vertices; a walk that returns to its start through interior
// vertices only is a closed loop. A branching interior vertex means the
// intersection curves cross on this facet — a contact the predicates refuse.
func (tc *TriCutter) BuildChains() ([]ChainPath, []ChainPath, error) {
	adj := map[int][]int{} // vertex -> edge indices
	for ei, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		adj[e.A] = append(adj[e.A], ei)
		adj[e.B] = append(adj[e.B], ei)
	}
	for vi, list := range adj {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if !tc.Verts[vi].Boundary && len(list) > 2 {
			return nil, nil, ErrUnclassifiableContact(`intersection curves branch on a facet`)
		}
		if !tc.Verts[vi].Boundary && len(list) == 1 {
			return nil, nil, fmt.Errorf(`%w: a contact chain dangles mid-facet`, decaderr.ErrBooleanFailed)
		}
	}
	otherEnd := func(ei, vi int) int {
		if tc.Edges[ei].A == vi {
			return tc.Edges[ei].B
		}
		return tc.Edges[ei].A
	}
	used := make([]bool, len(tc.Edges))
	var chains, loops []ChainPath
	// A chain interior vertex continues the walk exactly when it is
	// interior with degree two.
	continues := func(vi int) bool { return !tc.Verts[vi].Boundary && len(adj[vi]) == 2 }
	nextEdge := func(vi, from int) (int, error) {
		for _, ei := range adj[vi] {
			if err := tc.Work.Step(); err != nil {
				return -1, err
			}
			if ei != from && !used[ei] {
				return ei, nil
			}
		}
		return -1, nil
	}
	for start := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		if used[start] {
			continue
		}
		used[start] = true
		path := []int{tc.Edges[start].A, tc.Edges[start].B}
		isLoop := false
		// Extend forward from the tail, then backward from the head.
		for continues(path[len(path)-1]) {
			if err := tc.Work.Step(); err != nil {
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
				if err := tc.Work.Step(); err != nil {
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
					if err := tc.Work.Step(); err != nil {
						return nil, nil, err
					}
					path[i] = path[i-1]
				}
				path[0] = next
			}
		}
		if isLoop || path[0] == path[len(path)-1] {
			loops = append(loops, ChainPath{Verts: path})
			continue
		}
		chains = append(chains, ChainPath{Verts: path})
	}
	return chains, loops, nil
}

// ChooseSplitU picks a vertical line u = c strictly inside the loop's span
// that passes through no existing vertex, bisecting toward the span's low
// end until it clears — finitely many vertex ordinates, so it terminates.
func (tc *TriCutter) ChooseSplitU(loop ChainPath) (*big.Rat, error) {
	materializeXP2(tc.Verts[loop.Verts[0]].P2)
	lo, hi := tc.Verts[loop.Verts[0]].P2.U, tc.Verts[loop.Verts[0]].P2.U
	for _, vi := range loop.Verts {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		materializeXP2(tc.Verts[vi].P2)
		u := tc.Verts[vi].P2.U
		if u.Cmp(lo) < 0 {
			lo = u
		}
		if u.Cmp(hi) > 0 {
			hi = u
		}
	}
	if lo.Cmp(hi) == 0 {
		return nil, fmt.Errorf(`%w: a closed contact chain has no width`, decaderr.ErrBooleanFailed)
	}
	half := big.NewRat(1, 2)
	c := new(big.Rat).Mul(half, new(big.Rat).Add(lo, hi))
	for range 64 {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		hit := false
		for _, v := range tc.Verts {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			materializeXP2(v.P2)
			if v.P2.U.Cmp(c) == 0 {
				hit = true
				break
			}
		}
		if !hit {
			return c, nil
		}
		c = new(big.Rat).Mul(half, new(big.Rat).Add(lo, c))
	}
	return nil, fmt.Errorf(`%w: no vertex-free split line found`, decaderr.ErrBooleanFailed)
}

// SplitEdgesAtU splits every chain edge strictly straddling u = c at the
// exact crossing, which becomes a boundary vertex — a chain break.
func (tc *TriCutter) SplitEdgesAtU(c *big.Rat) error {
	var out []CutEdge
	for _, e := range tc.Edges {
		if err := tc.Work.Step(); err != nil {
			return err
		}
		materializeXP2(tc.Verts[e.A].P2)
		materializeXP2(tc.Verts[e.B].P2)
		ua := tc.Verts[e.A].P2.U
		ub := tc.Verts[e.B].P2.U
		sa := ua.Cmp(c)
		sb := ub.Cmp(c)
		if sa*sb >= 0 {
			out = append(out, e)
			continue
		}
		t := new(big.Rat).Quo(new(big.Rat).Sub(c, ua), new(big.Rat).Sub(ub, ua))
		p2 := NewXP2(
			new(big.Rat).Set(c),
			new(big.Rat).Add(tc.Verts[e.A].P2.V, new(big.Rat).Mul(t, new(big.Rat).Sub(tc.Verts[e.B].P2.V, tc.Verts[e.A].P2.V))),
		)
		p3 := proof.Xlerp(tc.Verts[e.A].P3, tc.Verts[e.B].P3, new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom()))
		mid := tc.AddVert(p2, p3, true)
		out = append(out,
			CutEdge{A: e.A, B: mid, Partner: e.Partner, ViaParity: e.ViaParity},
			CutEdge{A: mid, B: e.B, Partner: e.Partner, ViaParity: e.ViaParity})
	}
	tc.Edges = out
	return nil
}

// SplitConvexByU splits a convex polygon along u = c. A side that would be
// empty returns nil, leaving the piece whole on the other side.
func (tc *TriCutter) SplitConvexByU(piece []int, c *big.Rat) ([]int, []int, error) {
	n := len(piece)
	var left, right []int
	for i := range n {
		if err := tc.Work.Step(); err != nil {
			return nil, nil, err
		}
		vi, vj := piece[i], piece[(i+1)%n]
		materializeXP2(tc.Verts[vi].P2)
		materializeXP2(tc.Verts[vj].P2)
		si := tc.Verts[vi].P2.U.Cmp(c)
		sj := tc.Verts[vj].P2.U.Cmp(c)
		if si <= 0 {
			left = append(left, vi)
		}
		if si >= 0 {
			right = append(right, vi)
		}
		if si*sj >= 0 {
			continue
		}
		ua := tc.Verts[vi].P2.U
		ub := tc.Verts[vj].P2.U
		t := new(big.Rat).Quo(new(big.Rat).Sub(c, ua), new(big.Rat).Sub(ub, ua))
		p2 := NewXP2(
			new(big.Rat).Set(c),
			new(big.Rat).Add(tc.Verts[vi].P2.V, new(big.Rat).Mul(t, new(big.Rat).Sub(tc.Verts[vj].P2.V, tc.Verts[vi].P2.V))),
		)
		p3 := proof.Xlerp(tc.Verts[vi].P3, tc.Verts[vj].P3, new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom()))
		mid := tc.AddVert(p2, p3, true)
		left = append(left, mid)
		right = append(right, mid)
	}
	leftPts, err := tc.PolyPoints(left)
	if err != nil {
		return nil, nil, err
	}
	leftArea, err := PolyArea2Sign(tc.Work, leftPts)
	if err != nil {
		return nil, nil, err
	}
	if len(left) < 3 || leftArea <= 0 {
		left = nil
	}
	rightPts, err := tc.PolyPoints(right)
	if err != nil {
		return nil, nil, err
	}
	rightArea, err := PolyArea2Sign(tc.Work, rightPts)
	if err != nil {
		return nil, nil, err
	}
	if len(right) < 3 || rightArea <= 0 {
		right = nil
	}
	return left, right, nil
}

// SplitByChains splits one piece polygon along its assigned chains,
// returning the final region polygons.
func (tc *TriCutter) SplitByChains(piece []int, chains []ChainPath) ([][]int, error) {
	type work struct {
		poly   []int
		chains []ChainPath
	}
	stack := []work{{poly: piece, chains: chains}}
	var out [][]int
	for len(stack) > 0 {
		if err := tc.Work.Step(); err != nil {
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
		poly, err := tc.InsertOnBoundary(w.poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		poly, err = tc.InsertOnBoundary(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		i, err := tc.IndexOf(poly, ch.Verts[0])
		if err != nil {
			return nil, err
		}
		j, err := tc.IndexOf(poly, ch.Verts[len(ch.Verts)-1])
		if err != nil {
			return nil, err
		}
		if i < 0 || j < 0 || i == j {
			return nil, fmt.Errorf(`%w: a chain endpoint is missing from its region boundary`, decaderr.ErrBooleanFailed)
		}
		// Walk the boundary i→j one way and j→i the other, closing each side
		// with the chain path so both stay counter-clockwise.
		interior := ch.Verts[1 : len(ch.Verts)-1]
		var polyA []int
		for k := i; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, poly[k])
			if k == j {
				break
			}
		}
		for _, vi := range slices.Backward(interior) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyA = append(polyA, vi)
		}
		var polyB []int
		for k := j; ; k = (k + 1) % len(poly) {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, poly[k])
			if k == i {
				break
			}
		}
		for _, vi := range interior {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			polyB = append(polyB, vi)
		}
		ptsA, err := tc.PolyPoints(polyA)
		if err != nil {
			return nil, err
		}
		areaA, err := PolyArea2Sign(tc.Work, ptsA)
		if err != nil {
			return nil, err
		}
		ptsB, err := tc.PolyPoints(polyB)
		if err != nil {
			return nil, err
		}
		areaB, err := PolyArea2Sign(tc.Work, ptsB)
		if err != nil {
			return nil, err
		}
		if areaA <= 0 || areaB <= 0 {
			return nil, fmt.Errorf(`%w: a chain split produced a non-positive region`, decaderr.ErrBooleanFailed)
		}
		wa := work{poly: polyA}
		wb := work{poly: polyB}
		for _, rc := range rest {
			if err := tc.Work.Step(); err != nil {
				return nil, err
			}
			probe, err := tc.recordedChainProbe(rc)
			if err != nil {
				return nil, err
			}
			inside, onBoundary, err := PointInPoly2(tc.Work, ptsA, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary {
				return nil, fmt.Errorf(`%w: a chain lies on a freshly split boundary`, decaderr.ErrBooleanFailed)
			}
			if inside {
				wa.chains = append(wa.chains, rc)
				continue
			}
			inside, onBoundary, err = PointInPoly2(tc.Work, ptsB, probe)
			if err != nil {
				return nil, err
			}
			if onBoundary || !inside {
				return nil, fmt.Errorf(`%w: a chain escaped both split regions`, decaderr.ErrBooleanFailed)
			}
			wb.chains = append(wb.chains, rc)
		}
		stack = append(stack, wa, wb)
	}
	return out, nil
}

// InsertOnBoundary ensures the vertex appears on the polygon boundary,
// splicing it into the edge whose interior it lies on. Identity is the
// vertex id — subdivision vertices are interned by exact coordinates.
func (tc *TriCutter) InsertOnBoundary(poly []int, vi int) ([]int, error) {
	found, err := tc.IndexOf(poly, vi)
	if err != nil {
		return nil, err
	}
	if found >= 0 {
		return poly, nil
	}
	p := tc.Verts[vi].P2
	n := len(poly)
	for i := range n {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		a := tc.Verts[poly[i]].P2
		b := tc.Verts[poly[(i+1)%n]].P2
		_, interior := OnSegment2(a, b, p)
		if !interior {
			continue
		}
		out := make([]int, 0, n+1)
		out = append(out, poly[:i+1]...)
		out = append(out, vi)
		out = append(out, poly[i+1:]...)
		return out, nil
	}
	return nil, fmt.Errorf(`%w: a chain endpoint lies on no region boundary`, decaderr.ErrBooleanFailed)
}

func (tc *TriCutter) IndexOf(poly []int, vi int) (int, error) {
	for i, v := range poly {
		if err := tc.Work.Step(); err != nil {
			return -1, err
		}
		if v == vi {
			return i, nil
		}
	}
	return -1, nil
}

// RegionOf triangulates one final region polygon and finds its
// classification anchor: the region triangle adjacent to a chain edge, whose
// exact centroid probes the partner facet's plane side.
func (tc *TriCutter) RegionOf(poly []int, chainEdges map[[2]int]ChainAnchor) (CutRegion, error) {
	points, err := tc.collectP2Into(tc.regionPoints)
	if err != nil {
		return CutRegion{}, err
	}
	// EarClipX only reads the projections; the result carries copied 3D points.
	tc.regionPoints = points
	tris2, err := EarClipX(tc.Work, points, poly)
	if err != nil {
		return CutRegion{}, err
	}
	if len(tris2) == 0 {
		return CutRegion{}, fmt.Errorf(`%w: a region triangulated to nothing`, decaderr.ErrBooleanFailed)
	}
	reg := CutRegion{}
	for _, t := range tris2 {
		if err := tc.Work.Step(); err != nil {
			return CutRegion{}, err
		}
		corners := [3]proof.Xpt{tc.Verts[t[0]].P3, tc.Verts[t[1]].P3, tc.Verts[t[2]].P3}
		reg.Tris = append(reg.Tris, corners)
		if reg.HasAnchor {
			continue
		}
		for k := range 3 {
			if err := tc.Work.Step(); err != nil {
				return CutRegion{}, err
			}
			key := [2]int{min(t[k], t[(k+1)%3]), max(t[k], t[(k+1)%3])}
			anchor, ok := chainEdges[key]
			if !ok || anchor.ViaParity {
				continue
			}
			reg.HasAnchor = true
			reg.Partner = anchor.Partner
			reg.Probe = XCentroid(corners[0], corners[1], corners[2])
			break
		}
	}
	if !reg.HasAnchor {
		first := reg.Tris[0]
		reg.Probe = XCentroid(first[0], first[1], first[2])
	}
	return reg, nil
}

// CollectP2 is the projected-point view of the vertex table, as EarClipX
// wants it.
func (tc *TriCutter) CollectP2() ([]Xp2, error) {
	return tc.collectP2Into(make([]Xp2, len(tc.Verts)))
}

func (tc *TriCutter) collectP2Into(out []Xp2) ([]Xp2, error) {
	if cap(out) < len(tc.Verts) {
		out = make([]Xp2, len(tc.Verts))
	} else {
		out = out[:len(tc.Verts)]
	}
	for i, v := range tc.Verts {
		if err := tc.Work.Step(); err != nil {
			return nil, err
		}
		out[i] = v.P2
	}
	return out, nil
}
