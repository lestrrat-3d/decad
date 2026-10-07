// Package facetedtopology groups audited mesh boundaries into face loops and edge chains.
package facetedtopology

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type Edge struct {
	Vertices        []int
	Faces           [2]int
	Convex          bool
	Bound           float64
	Length          float64
	LengthBound     float64
	LengthUnbounded bool
}

type Coedge struct {
	Edge    *Edge
	Forward bool
}

type Loop struct {
	Coedges []Coedge
	Outer   bool
}

type Result struct {
	Edges     []*Edge
	FaceLoops map[int][]*Loop
}

// Build traces the boundary chains and loops of an audited faceted mesh.
func Build(
	ctx context.Context,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []int,
	facePlanar map[int]bool,
	vertexBound []float64,
) (Result, error) {
	var result Result
	err := build(ctx, verts, tris, facetFace, facePlanar, vertexBound, &result)
	return result, err
}

func build(
	ctx context.Context,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []int,
	facePlanar map[int]bool,
	vertexBound []float64,
	result *Result,
) error {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return err
	}
	if len(vertexBound) != len(verts) {
		return fmt.Errorf(`%w: a faceted body carries %d vertex bounds for %d vertices`, decaderr.ErrBooleanFailed, len(vertexBound), len(verts))
	}

	// Directed halfedge → facet, and the boundary predicate: the twin facet
	// belongs to a different face.
	halfOwner := map[[2]int]int{}
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			halfOwner[[2]int{t[k], t[(k+1)%3]}] = i
		}
	}
	isBoundary := func(u, v int) bool {
		mine, ok := halfOwner[[2]int{u, v}]
		if !ok {
			return false
		}
		twin, ok := halfOwner[[2]int{v, u}]
		if !ok {
			return false
		}
		return facetFace[mine] != facetFace[twin]
	}

	// Undirected boundary edges with their face pair and exact hinge sign.
	type hingeInfo struct {
		fa, fb int
		sign   int
	}
	hinges := map[[2]int]hingeInfo{}
	var boundaryEdges [][2]int
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			u, v := t[k], t[(k+1)%3]
			if u > v || !isBoundary(u, v) {
				continue
			}
			mine := i
			twin := halfOwner[[2]int{v, u}]
			// The hinge: the twin's opposite vertex against this facet's
			// plane — below (negative) is material bending away: convex.
			tw := tris[twin]
			opp := tw[0] + tw[1] + tw[2] - u - v
			s := proof.OrientSign(verts[t[0]], verts[t[1]], verts[t[2]], verts[opp])
			hinges[[2]int{u, v}] = hingeInfo{fa: facetFace[mine], fb: facetFace[twin], sign: s}
			boundaryEdges = append(boundaryEdges, [2]int{u, v})
		}
	}
	if len(boundaryEdges) == 0 {
		return fmt.Errorf(`%w: a faceted body carries no face boundaries`, decaderr.ErrBooleanFailed)
	}

	// Chain the boundary edges: a vertex continues a chain when exactly two
	// boundary edges meet there with the same face pair and hinge sign.
	incident := map[int][][2]int{}
	for _, e := range boundaryEdges {
		if err := budget.Step(); err != nil {
			return err
		}
		incident[e[0]] = append(incident[e[0]], e)
		incident[e[1]] = append(incident[e[1]], e)
	}
	samePair := func(a, b hingeInfo) bool {
		return a.sign == b.sign &&
			(a.fa == b.fa && a.fb == b.fb || a.fa == b.fb && a.fb == b.fa)
	}
	// A chain breaks at any vertex that does not join exactly two boundary
	// edges of the same face pair and hinge sign.
	breakAt := func(v int) bool {
		list := incident[v]
		if len(list) != 2 {
			return true
		}
		ka := [2]int{min(list[0][0], list[0][1]), max(list[0][0], list[0][1])}
		kb := [2]int{min(list[1][0], list[1][1]), max(list[1][0], list[1][1])}
		return !samePair(hinges[ka], hinges[kb])
	}
	ukey := func(e [2]int) [2]int { return [2]int{min(e[0], e[1]), max(e[0], e[1])} }

	type chainRec struct {
		verts []int
		edge  *Edge
	}
	chainOf := map[[2]int]int{} // undirected mesh edge → chain index
	posInChain := map[[2]int]int{}
	var chains []chainRec
	usedEdge := map[[2]int]struct{}{}
	otherAt := func(v int, notKey [2]int) [2]int {
		for _, e := range incident[v] {
			if ukey(e) != notKey {
				return e
			}
		}
		return notKey
	}
	commitChain := func(path []int) error {
		info := hinges[ukey([2]int{path[0], path[1]})]
		length := 0.0
		nSegs := 0
		bound := vertexBound[path[0]]
		for k := 0; k+1 < len(path); k++ {
			if err := budget.Step(); err != nil {
				return err
			}
			key := ukey([2]int{path[k], path[k+1]})
			chainOf[key] = len(chains)
			posInChain[key] = k
			length += verts[path[k+1]].Sub(verts[path[k]]).Len()
			bound = max(bound, vertexBound[path[k+1]])
			nSegs++
		}
		// The chain holds nSegs chords, and BOTH endpoints of each move: the
		// error accumulates over N, it is not δ (internal/proofbound/bounds.go, proofbound.ChainLengthBound).
		// It is never zero either — the held length is a float sum of square
		// roots, and the last ulp is not free — so an all-planar rim reports
		// Approximate rather than an Exact it cannot back.
		lengthBound := proofbound.ChainLengthBound(nSegs, bound, length)
		e := &Edge{
			Vertices:    path,
			Bound:       bound,
			Faces:       [2]int{info.fa, info.fb},
			Convex:      info.sign < 0,
			Length:      length,
			LengthBound: lengthBound,
			// A rim between two PLANAR sources is a straight line, so its
			// chord length is honest; any curved source leaves the true
			// curve's length excess over the chords unboundable without
			// curvature knowledge, and Length must refuse rather than
			// understate (never a silent pass).
			LengthUnbounded: !facePlanar[info.fa] || !facePlanar[info.fb],
		}
		chains = append(chains, chainRec{verts: path, edge: e})
		return nil
	}
	walkFrom := func(v int, e [2]int) ([]int, error) {
		path := []int{v}
		cur, curEdge := v, e
		for {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			usedEdge[ukey(curEdge)] = struct{}{}
			nxt := curEdge[0] + curEdge[1] - cur
			path = append(path, nxt)
			if nxt == path[0] || breakAt(nxt) {
				return path, nil
			}
			nextEdge := otherAt(nxt, ukey(curEdge))
			if ukey(nextEdge) == ukey(curEdge) {
				return path, nil
			}
			if _, seen := usedEdge[ukey(nextEdge)]; seen {
				return path, nil
			}
			cur, curEdge = nxt, nextEdge
		}
	}
	// Open chains first: start at every break vertex, in edge order.
	for _, e := range boundaryEdges {
		for _, v := range []int{e[0], e[1]} {
			if err := budget.Step(); err != nil {
				return err
			}
			if !breakAt(v) {
				continue
			}
			if _, ok := usedEdge[ukey(e)]; ok {
				break
			}
			path, err := walkFrom(v, e)
			if err != nil {
				return err
			}
			if err := commitChain(path); err != nil {
				return err
			}
			break
		}
	}
	// The rest are closed uniform cycles: anchor each at its smallest
	// vertex, deterministically.
	for _, e := range boundaryEdges {
		if err := budget.Step(); err != nil {
			return err
		}
		if _, ok := usedEdge[ukey(e)]; ok {
			continue
		}
		cycle, err := walkFrom(e[0], e)
		if err != nil {
			return err
		}
		if cycle[0] != cycle[len(cycle)-1] {
			return fmt.Errorf(`%w: a face boundary chain did not close`, decaderr.ErrBooleanFailed)
		}
		ring := cycle[:len(cycle)-1]
		anchor := 0
		for i, v := range ring {
			if v < ring[anchor] {
				anchor = i
			}
		}
		rotated := make([]int, 0, len(cycle))
		for i := range ring {
			if err := budget.Step(); err != nil {
				return err
			}
			rotated = append(rotated, ring[(anchor+i)%len(ring)])
		}
		rotated = append(rotated, ring[anchor])
		// Re-commit with the rotated ordering (walkFrom already marked the
		// edges used).
		if err := commitChain(rotated); err != nil {
			return err
		}
	}

	// The area-weighted normal of each face's own facets. A face is ONE patch,
	// and a patch of a PLANAR source is coplanar, so this is that plane's
	// outward normal (scaled by twice the patch's area).
	faceNormal := map[int]r3.Vec{}
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		f := facetFace[i]
		faceNormal[f] = faceNormal[f].Add(b.Sub(a).Cross(c.Sub(a)))
	}

	// Face loops: walk each face's directed boundary cycles through the
	// facet fans, then group consecutive halfedges by chain into coedges.
	// loopMoment is each loop's closed-polygon area vector Σ vᵢ × vᵢ₊₁ (twice
	// the signed area), which is what decides outer from hole on a planar face.
	loopMoment := map[*Loop]r3.Vec{}
	faceLoops := map[int][]*Loop{}
	visited := map[[2]int]struct{}{}
	nextBoundary := func(h [2]int) ([2]int, error) {
		cur := h
		for range len(tris)*3 + 3 {
			if err := budget.Step(); err != nil {
				return [2]int{}, err
			}
			f := halfOwner[cur]
			t := tris[f]
			var follow [2]int
			switch {
			case t[0] == cur[0] && t[1] == cur[1]:
				follow = [2]int{t[1], t[2]}
			case t[1] == cur[0] && t[2] == cur[1]:
				follow = [2]int{t[2], t[0]}
			case t[2] == cur[0] && t[0] == cur[1]:
				follow = [2]int{t[0], t[1]}
			default:
				return [2]int{}, fmt.Errorf(`%w: a halfedge lost its facet`, decaderr.ErrBooleanFailed)
			}
			if isBoundary(follow[0], follow[1]) {
				return follow, nil
			}
			cur = [2]int{follow[1], follow[0]}
		}
		return [2]int{}, fmt.Errorf(`%w: a face boundary walk did not close`, decaderr.ErrBooleanFailed)
	}
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			h := [2]int{t[k], t[(k+1)%3]}
			if !isBoundary(h[0], h[1]) {
				continue
			}
			if _, ok := visited[h]; ok {
				continue
			}
			face := facetFace[i]
			var cycle [][2]int
			cur := h
			// A boundary walk must consume each boundary halfedge at most once.
			// Keep the walk bounded even if malformed adjacency causes nextBoundary
			// to cycle without returning to its start.
			for steps := 0; ; steps++ {
				if err := budget.Step(); err != nil {
					return err
				}
				if steps >= len(tris)*3+1 {
					return fmt.Errorf(`%w: a face boundary walk exceeded its halfedges`, decaderr.ErrBooleanFailed)
				}
				visited[cur] = struct{}{}
				cycle = append(cycle, cur)
				nxt, err := nextBoundary(cur)
				if err != nil {
					return err
				}
				if nxt == h {
					break
				}
				cur = nxt
			}
			loop := &Loop{}
			firstChain, lastChain := -1, -1
			for _, he := range cycle {
				if err := budget.Step(); err != nil {
					return err
				}
				key := [2]int{min(he[0], he[1]), max(he[0], he[1])}
				ci, ok := chainOf[key]
				if !ok {
					return fmt.Errorf(`%w: a boundary halfedge belongs to no chain`, decaderr.ErrBooleanFailed)
				}
				if ci == lastChain {
					continue
				}
				lastChain = ci
				if firstChain == -1 {
					firstChain = ci
				}
				forward := chains[ci].verts[posInChain[key]] == he[0]
				loop.Coedges = append(loop.Coedges, Coedge{Edge: chains[ci].edge, Forward: forward})
			}
			// The walk starts at whichever halfedge the facet scan reached
			// first, which can sit in the MIDDLE of a chain — and then that one
			// chain is met twice, once at each end of the cycle, and would be
			// listed as two coedges of one edge. The cycle is closed, so the
			// tail is the head's own chain resumed: drop it. (A chain that is
			// the loop's ONLY one already collapsed to a single coedge above.)
			if n := len(loop.Coedges); n > 1 && lastChain == firstChain {
				loop.Coedges = loop.Coedges[:n-1]
			}
			mom := r3.Vec{}
			for _, he := range cycle {
				if err := budget.Step(); err != nil {
					return err
				}
				mom = mom.Add(verts[he[0]].Cross(verts[he[1]]))
			}
			loopMoment[loop] = mom
			faceLoops[face] = append(faceLoops[face], loop)
		}
	}

	// Pick each face's outer loop. A face is ONE connected patch, so exactly one
	// of its loops bounds it from outside and the rest are holes in it.
	//
	// On a PLANAR patch that is decided, not guessed: the boundary is walked
	// with the material on its left, so about the patch's own outward normal
	// the outer loop turns positive and every hole turns negative — and the
	// outer loop's area vector is the patch's area PLUS its holes', so it is
	// the largest. A longest-perimeter pick would not do: a long serpentine
	// slot can out-measure the boundary it is cut into, and crowning it outer
	// would report the face's true outer boundary as a hole in its own slot.
	//
	// A CURVED patch has no such plane, and no loop of it is a hole in another
	// (a hole wall's two rims bound a tube). There the longest boundary stands
	// as the deterministic bookkeeping choice: validity never reads it, and
	// Faceted faces expose their polygons through Tessellate, not through loop
	// nesting.
	for f := range collectFaces(facetFace) {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(faceLoops[f]) == 0 {
			return fmt.Errorf(`%w: a faceted face has no boundary loop`, decaderr.ErrBooleanFailed)
		}
		li := -1
		if facePlanar[f] {
			n := faceNormal[f]
			best := 0.0
			for idx, l := range faceLoops[f] {
				if err := budget.Step(); err != nil {
					return err
				}
				if s := loopMoment[l].Dot(n); s > best {
					best, li = s, idx
				}
			}
		}
		if li == -1 {
			longest := -1.0
			li = 0
			for idx, l := range faceLoops[f] {
				if err := budget.Step(); err != nil {
					return err
				}
				total := 0.0
				for _, ce := range l.Coedges {
					if err := budget.Step(); err != nil {
						return err
					}
					total += ce.Edge.Length
				}
				if total > longest {
					longest, li = total, idx
				}
			}
		}
		faceLoops[f][0], faceLoops[f][li] = faceLoops[f][li], faceLoops[f][0]
		faceLoops[f][0].Outer = true
	}
	result.FaceLoops = faceLoops
	for _, chain := range chains {
		result.Edges = append(result.Edges, chain.edge)
	}
	return budget.Err()
}

func collectFaces(facetFace []int) map[int]struct{} {
	out := map[int]struct{}{}
	for _, f := range facetFace {
		out[f] = struct{}{}
	}
	return out
}
