// Package facetedtopology groups audited mesh boundaries into face loops and edge chains.
package facetedtopology

import (
	"context"
	"fmt"
	"slices"
	"unsafe"

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
	Coedges      []Coedge
	Outer        bool
	replayMoment r3.Vec
}

type Result struct {
	Edges           []*Edge
	FaceLoops       map[int][]*Loop
	ReplayFaceLoops [][]*Loop
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
	err := build(ctx, proofbound.NewWorkBudget(ctx), verts, tris, facetFace, facePlanar, vertexBound, &result)
	return result, err
}

// BuildWithBudget charges the caller's shared proof counter. The caller
// precharges bounded container storage before this function allocates it.
func BuildWithBudget(ctx context.Context, budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int, facetFace []int, facePlanar map[int]bool,
	vertexBound []float64) (Result, error) {
	var result Result
	if budget != nil && budget.Replay {
		return result, fmt.Errorf("%w: replay topology requires ordered face flags", decaderr.ErrUnsupported)
	}
	err := build(ctx, budget, verts, tris, facetFace, facePlanar, vertexBound, &result)
	return result, err
}

// BuildWithReplay uses ordered face flags with the shared replay counter.
func BuildWithReplay(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, facetFace []int, facePlanar []bool, vertexBound []float64) (Result, error) {
	var result Result
	err := buildWithPlanar(ctx, budget, verts, tris, facetFace, nil, facePlanar, vertexBound, &result)
	return result, err
}

func build(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, facetFace []int, facePlanar map[int]bool, vertexBound []float64, result *Result) error {
	return buildWithPlanar(ctx, budget, verts, tris, facetFace, facePlanar, nil, vertexBound, result)
}

func buildWithPlanar(
	ctx context.Context,
	budget *proofbound.WorkBudget,
	verts []r3.Vec,
	tris [][3]int,
	facetFace []int,
	facePlanar map[int]bool,
	replayPlanar []bool,
	vertexBound []float64,
	result *Result,
) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := budget.Err(); err != nil {
		return err
	}
	if len(vertexBound) != len(verts) {
		return fmt.Errorf(`%w: a faceted body carries %d vertex bounds for %d vertices`, decaderr.ErrBooleanFailed, len(vertexBound), len(verts))
	}

	isPlanar := func(face int) bool {
		if budget.Replay {
			return face >= 0 && face < len(replayPlanar) && replayPlanar[face]
		}
		return facePlanar[face]
	}

	// Directed halfedge → facet, and the boundary predicate: the twin facet
	// belongs to a different face. Each halfedge is named by its ordinal
	// 3i+k. halfOwner holds the map's answers for the directed edge each
	// ordinal carries and for its reverse; every lookup below asks about a
	// halfedge whose ordinal it knows, or the reverse of one, so reading the
	// table returns what the map lookup returned.
	var halfOwner halfedgeOwners
	if budget.Replay {
		var err error
		halfOwner, err = replayHalfedgeOwners(budget, tris)
		if err != nil {
			return err
		}
	} else {
		directed := newDirectedFacets(len(verts), 3*len(tris))
		for i, t := range tris {
			for k := range 3 {
				if err := budget.Step(); err != nil {
					return err
				}
				directed.set(t[k], t[(k+1)%3], i)
			}
		}
		halfOwner = newHalfedgeOwners(directed, tris)
	}
	isBoundary := func(o int) bool {
		twin := halfOwner.twin[o]
		if twin < 0 {
			return false
		}
		return facetFace[halfOwner.owner(o)] != facetFace[twin]
	}

	// Undirected boundary edges with their face pair and exact hinge sign.
	type hingeInfo struct {
		fa, fb int
		sign   int
	}
	var hinges map[[2]int]hingeInfo
	var fixedHinges *replayEdgeTable[hingeInfo]
	if budget.Replay {
		var err error
		fixedHinges, err = newReplayEdgeTable[hingeInfo](budget, 3*len(tris))
		if err != nil {
			return err
		}
	} else {
		hinges = map[[2]int]hingeInfo{}
	}
	getHinge := func(key [2]int) (hingeInfo, error) {
		if !budget.Replay {
			return hinges[key], nil
		}
		v, ok, err := fixedHinges.get(key)
		if err != nil {
			return hingeInfo{}, err
		}
		if !ok {
			return hingeInfo{}, fmt.Errorf("%w: missing replay hinge", decaderr.ErrBooleanFailed)
		}
		return v, nil
	}
	var boundaryEdges [][2]int
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			u, v := t[k], t[(k+1)%3]
			if u > v || !isBoundary(3*i+k) {
				continue
			}
			mine := i
			twin := halfOwner.twin[3*i+k]
			// The hinge: the twin's opposite vertex against this facet's
			// plane — below (negative) is material bending away: convex.
			tw := tris[twin]
			opp := tw[0] + tw[1] + tw[2] - u - v
			var s int
			if budget.Replay {
				var err error
				s, err = replayOrientation(ctx, budget, verts[t[0]], verts[t[1]], verts[t[2]], verts[opp])
				if err != nil {
					return err
				}
			} else {
				s = proof.OrientSign(verts[t[0]], verts[t[1]], verts[t[2]], verts[opp])
			}
			key := [2]int{u, v}
			info := hingeInfo{fa: facetFace[mine], fb: facetFace[twin], sign: s}
			if budget.Replay {
				if err := fixedHinges.set(key, info); err != nil {
					return err
				}
			} else {
				hinges[key] = info
			}
			var err error
			boundaryEdges, err = replayAppend(budget, boundaryEdges, [2]int{u, v})
			if err != nil {
				return err
			}
		}
	}
	if len(boundaryEdges) == 0 {
		// A closed one-face patch, such as a whole sphere, has no B-rep
		// edges or loops. Check closure and edge connectivity before naming
		// that topology; disconnected shells must not collapse to one face.
		if len(tris) == 0 || len(facetFace) != len(tris) || halfOwner.distinct != 3*len(tris) {
			return fmt.Errorf(`%w: a faceted body carries no face boundaries`, decaderr.ErrBooleanFailed)
		}
		face := facetFace[0]
		var seen map[int]struct{}
		var seenFixed []bool
		var queue []int
		seenCount := 1
		if budget.Replay {
			if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(false))); err != nil {
				return err
			}
			if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(int(0)))); err != nil {
				return err
			}
			seenFixed = make([]bool, len(tris))
			seenFixed[0] = true
			queue = make([]int, 0, len(tris))
			queue = append(queue, 0)
		} else {
			seen = map[int]struct{}{0: {}}
			queue = []int{0}
		}
		for len(queue) > 0 {
			if err := budget.Step(); err != nil {
				return err
			}
			i := queue[len(queue)-1]
			queue = queue[:len(queue)-1]
			if facetFace[i] != face {
				return fmt.Errorf("%w: a faceted body carries no face boundaries", decaderr.ErrBooleanFailed)
			}
			for k := range 3 {
				twin := halfOwner.twin[3*i+k]
				if twin < 0 || facetFace[twin] != face {
					return fmt.Errorf("%w: a faceted body carries no face boundaries", decaderr.ErrBooleanFailed)
				}
				present := false
				if budget.Replay {
					present = seenFixed[twin]
				} else {
					_, present = seen[twin]
				}
				if !present {
					if budget.Replay {
						seenFixed[twin] = true
					} else {
						seen[twin] = struct{}{}
					}
					seenCount++
					queue = append(queue, twin)
				}
			}
		}
		if seenCount != len(tris) {
			return fmt.Errorf("%w: a faceted body carries no face boundaries", decaderr.ErrBooleanFailed)
		}
		if budget.Replay {
			if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof([]*Loop{}))); err != nil {
				return err
			}
			result.ReplayFaceLoops = make([][]*Loop, len(tris))
		} else {
			result.FaceLoops = map[int][]*Loop{face: nil}
		}
		return nil
	}

	// Chain the boundary edges: a vertex continues a chain when exactly two
	// boundary edges meet there with the same face pair and hinge sign.
	var incident map[int][][2]int
	var incidentFixed [][][2]int
	if budget.Replay {
		count := len(verts)
		edges := len(boundaryEdges)
		if err := budget.Reserve(uint64(2*count + 2*edges)); err != nil {
			return err
		}
		if err := budget.ReserveBytesMul(uint64(count), uint64(unsafe.Sizeof([][2]int{}))); err != nil {
			return err
		}
		if err := budget.ReserveBytesMul(uint64(count), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return err
		}
		if err := budget.ReserveBytesMul(uint64(2*edges), uint64(unsafe.Sizeof([2]int{}))); err != nil {
			return err
		}
		counts := make([]int, count)
		for _, e := range boundaryEdges {
			if err := budget.Step(); err != nil {
				return err
			}
			counts[e[0]]++
			counts[e[1]]++
		}
		incidentFixed = make([][][2]int, count)
		flat := make([][2]int, 2*edges)
		at := 0
		for v, n := range counts {
			if err := budget.Step(); err != nil {
				return err
			}
			incidentFixed[v] = flat[at : at : at+n]
			at += n
		}
		for _, e := range boundaryEdges {
			if err := budget.Step(); err != nil {
				return err
			}
			incidentFixed[e[0]] = append(incidentFixed[e[0]], e)
			incidentFixed[e[1]] = append(incidentFixed[e[1]], e)
		}
	} else {
		incident = map[int][][2]int{}
		for _, e := range boundaryEdges {
			if err := budget.Step(); err != nil {
				return err
			}
			incident[e[0]] = append(incident[e[0]], e)
			incident[e[1]] = append(incident[e[1]], e)
		}
	}
	incidentOf := func(v int) [][2]int {
		if budget.Replay {
			return incidentFixed[v]
		}
		return incident[v]
	}
	samePair := func(a, b hingeInfo) bool {
		return a.sign == b.sign &&
			(a.fa == b.fa && a.fb == b.fb || a.fa == b.fb && a.fb == b.fa)
	}
	// A chain breaks at any vertex that does not join exactly two boundary
	// edges of the same face pair and hinge sign.
	breakAt := func(v int) (bool, error) {
		list := incidentOf(v)
		if len(list) != 2 {
			return true, nil
		}
		ka := [2]int{min(list[0][0], list[0][1]), max(list[0][0], list[0][1])}
		kb := [2]int{min(list[1][0], list[1][1]), max(list[1][0], list[1][1])}
		a, err := getHinge(ka)
		if err != nil {
			return false, err
		}
		b, err := getHinge(kb)
		if err != nil {
			return false, err
		}
		return !samePair(a, b), nil
	}
	ukey := func(e [2]int) [2]int { return [2]int{min(e[0], e[1]), max(e[0], e[1])} }

	type chainRec struct {
		verts []int
		edge  *Edge
	}
	type edgeState struct {
		chain, pos     int
		hasChain, used bool
	}
	var chainOf, posInChain map[[2]int]int
	var usedEdge map[[2]int]struct{}
	var fixedEdges *replayEdgeTable[edgeState]
	if budget.Replay {
		var err error
		fixedEdges, err = newReplayEdgeTable[edgeState](budget, len(boundaryEdges))
		if err != nil {
			return err
		}
	} else {
		chainOf = map[[2]int]int{}
		posInChain = map[[2]int]int{}
		usedEdge = map[[2]int]struct{}{}
	}
	getEdge := func(key [2]int) (edgeState, error) {
		if budget.Replay {
			v, _, err := fixedEdges.get(key)
			return v, err
		}
		ci, ok := chainOf[key]
		_, used := usedEdge[key]
		return edgeState{chain: ci, pos: posInChain[key], hasChain: ok, used: used}, nil
	}
	setEdge := func(key [2]int, v edgeState) error {
		if budget.Replay {
			return fixedEdges.set(key, v)
		}
		if v.hasChain {
			chainOf[key] = v.chain
			posInChain[key] = v.pos
		}
		if v.used {
			usedEdge[key] = struct{}{}
		}
		return nil
	}
	var chains []chainRec
	otherAt := func(v int, notKey [2]int) [2]int {
		for _, e := range incidentOf(v) {
			if ukey(e) != notKey {
				return e
			}
		}
		return notKey
	}
	commitChain := func(path []int) error {
		info, err := getHinge(ukey([2]int{path[0], path[1]}))
		if err != nil {
			return err
		}
		length := 0.0
		nSegs := 0
		bound := vertexBound[path[0]]
		for k := 0; k+1 < len(path); k++ {
			if err := budget.Step(); err != nil {
				return err
			}
			key := ukey([2]int{path[k], path[k+1]})
			state, err := getEdge(key)
			if err != nil {
				return err
			}
			state.chain, state.pos, state.hasChain = len(chains), k, true
			if err := setEdge(key, state); err != nil {
				return err
			}
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
		if budget.Replay {
			if err := budget.ReserveBytesMul(1, uint64(unsafe.Sizeof(Edge{}))); err != nil {
				return err
			}
		}
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
			LengthUnbounded: !isPlanar(info.fa) || !isPlanar(info.fb),
		}
		chains, err = replayAppend(budget, chains, chainRec{verts: path, edge: e})
		return err
	}
	walkFrom := func(v int, e [2]int) ([]int, error) {
		var path []int
		path, err := replayAppend(budget, path, v)
		if err != nil {
			return nil, err
		}
		cur, curEdge := v, e
		for {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			key := ukey(curEdge)
			state, err := getEdge(key)
			if err != nil {
				return nil, err
			}
			state.used = true
			if err := setEdge(key, state); err != nil {
				return nil, err
			}
			nxt := curEdge[0] + curEdge[1] - cur
			path, err = replayAppend(budget, path, nxt)
			if err != nil {
				return nil, err
			}
			if nxt == path[0] {
				return path, nil
			}
			br, err := breakAt(nxt)
			if err != nil {
				return nil, err
			}
			if br {
				return path, nil
			}
			nextEdge := otherAt(nxt, ukey(curEdge))
			if ukey(nextEdge) == ukey(curEdge) {
				return path, nil
			}
			state, err = getEdge(ukey(nextEdge))
			if err != nil {
				return nil, err
			}
			if state.used {
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
			br, err := breakAt(v)
			if err != nil {
				return err
			}
			if !br {
				continue
			}
			state, err := getEdge(ukey(e))
			if err != nil {
				return err
			}
			if state.used {
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
		state, err := getEdge(ukey(e))
		if err != nil {
			return err
		}
		if state.used {
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
		if budget.Replay {
			if err := budget.Reserve(uint64(len(cycle))); err != nil {
				return err
			}
			if err := budget.ReserveBytesMul(uint64(len(cycle)), uint64(unsafe.Sizeof(int(0)))); err != nil {
				return err
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
	var faceNormal map[int]r3.Vec
	var normalFixed []r3.Vec
	if budget.Replay {
		if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
			return err
		}
		normalFixed = make([]r3.Vec, len(tris))
	} else {
		faceNormal = map[int]r3.Vec{}
	}
	for i, t := range tris {
		if err := budget.Step(); err != nil {
			return err
		}
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		f := facetFace[i]
		if budget.Replay {
			if f < 0 || f >= len(normalFixed) {
				return fmt.Errorf("%w: replay face index", decaderr.ErrBooleanFailed)
			}
			normalFixed[f] = normalFixed[f].Add(b.Sub(a).Cross(c.Sub(a)))
		} else {
			faceNormal[f] = faceNormal[f].Add(b.Sub(a).Cross(c.Sub(a)))
		}
	}

	// Face loops: walk each face's directed boundary cycles through the
	// facet fans, then group consecutive halfedges by chain into coedges.
	// loopMoment is each loop's closed-polygon area vector Σ vᵢ × vᵢ₊₁ (twice
	// the signed area), which is what decides outer from hole on a planar face.
	var loopMoment map[*Loop]r3.Vec
	var faceLoops map[int][]*Loop
	var loopsFixed [][]*Loop
	if budget.Replay {
		if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof([]*Loop{}))); err != nil {
			return err
		}
		loopsFixed = make([][]*Loop, len(tris))
	} else {
		loopMoment = map[*Loop]r3.Vec{}
		faceLoops = map[int][]*Loop{}
	}
	loopsOf := func(id int) []*Loop {
		if budget.Replay {
			return loopsFixed[id]
		}
		return faceLoops[id]
	}
	var visited map[[2]int]struct{}
	var fixedVisited *replayEdgeTable[bool]
	if budget.Replay {
		var err error
		fixedVisited, err = newReplayEdgeTable[bool](budget, 2*len(boundaryEdges))
		if err != nil {
			return err
		}
	} else {
		visited = map[[2]int]struct{}{}
	}
	wasVisited := func(k [2]int) (bool, error) {
		if budget.Replay {
			v, ok, err := fixedVisited.get(k)
			return ok && v, err
		}
		_, ok := visited[k]
		return ok, nil
	}
	markVisited := func(k [2]int) error {
		if budget.Replay {
			return fixedVisited.set(k, true)
		}
		visited[k] = struct{}{}
		return nil
	}
	// nextBoundary walks on from boundary halfedge h, whose ordinal is ho,
	// and returns the next boundary halfedge with its ordinal. f is always the
	// facet the map named for cur's directed edge: h's owner, and for the
	// reverse of a followed halfedge its twin, or the map's zero facet 0 where
	// it has none.
	nextBoundary := func(h [2]int, ho int) ([2]int, int, error) {
		cur, f := h, halfOwner.owner(ho)
		for range len(tris)*3 + 3 {
			if err := budget.Step(); err != nil {
				return [2]int{}, 0, err
			}
			t := tris[f]
			var follow [2]int
			var fo int
			switch {
			case t[0] == cur[0] && t[1] == cur[1]:
				follow, fo = [2]int{t[1], t[2]}, 3*f+1
			case t[1] == cur[0] && t[2] == cur[1]:
				follow, fo = [2]int{t[2], t[0]}, 3*f+2
			case t[2] == cur[0] && t[0] == cur[1]:
				follow, fo = [2]int{t[0], t[1]}, 3*f
			default:
				return [2]int{}, 0, fmt.Errorf(`%w: a halfedge lost its facet`, decaderr.ErrBooleanFailed)
			}
			if isBoundary(fo) {
				return follow, fo, nil
			}
			cur, f = [2]int{follow[1], follow[0]}, max(halfOwner.twin[fo], 0)
		}
		return [2]int{}, 0, fmt.Errorf(`%w: a face boundary walk did not close`, decaderr.ErrBooleanFailed)
	}
	for i, t := range tris {
		for k := range 3 {
			if err := budget.Step(); err != nil {
				return err
			}
			h := [2]int{t[k], t[(k+1)%3]}
			if !isBoundary(3*i + k) {
				continue
			}
			seen, err := wasVisited(h)
			if err != nil {
				return err
			}
			if seen {
				continue
			}
			face := facetFace[i]
			var cycle [][2]int
			cur, co := h, 3*i+k
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
				if err := markVisited(cur); err != nil {
					return err
				}
				cycle, err = replayAppend(budget, cycle, cur)
				if err != nil {
					return err
				}
				nxt, no, err := nextBoundary(cur, co)
				if err != nil {
					return err
				}
				if nxt == h {
					break
				}
				cur, co = nxt, no
			}
			if budget.Replay {
				if err := budget.ReserveBytesMul(1, uint64(unsafe.Sizeof(Loop{}))); err != nil {
					return err
				}
			}
			loop := &Loop{}
			firstChain, lastChain := -1, -1
			for _, he := range cycle {
				if err := budget.Step(); err != nil {
					return err
				}
				key := [2]int{min(he[0], he[1]), max(he[0], he[1])}
				state, err := getEdge(key)
				if err != nil {
					return err
				}
				ci := state.chain
				if !state.hasChain {
					return fmt.Errorf(`%w: a boundary halfedge belongs to no chain`, decaderr.ErrBooleanFailed)
				}
				if ci == lastChain {
					continue
				}
				lastChain = ci
				if firstChain == -1 {
					firstChain = ci
				}
				forward := chains[ci].verts[state.pos] == he[0]
				loop.Coedges, err = replayAppend(budget, loop.Coedges, Coedge{Edge: chains[ci].edge, Forward: forward})
				if err != nil {
					return err
				}
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
			if budget.Replay {
				loop.replayMoment = mom
				list := loopsFixed[face]
				if len(list) == cap(list) {
					capacity := max(1, 2*cap(list))
					if err := budget.Reserve(uint64(capacity)); err != nil {
						return err
					}
					if err := budget.ReserveBytesMul(uint64(capacity), uint64(unsafe.Sizeof((*Loop)(nil)))); err != nil {
						return err
					}
					next := make([]*Loop, len(list), capacity)
					copy(next, list)
					list = next
				}
				loopsFixed[face] = append(list, loop)
			} else {
				loopMoment[loop] = mom
				faceLoops[face] = append(faceLoops[face], loop)
			}
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
	var faceList []int
	if budget.Replay {
		if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return err
		}
		if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof(false))); err != nil {
			return err
		}
		seen := make([]bool, len(tris))
		for _, f := range facetFace {
			if f < 0 || f >= len(seen) {
				return fmt.Errorf("%w: replay face index", decaderr.ErrBooleanFailed)
			}
			seen[f] = true
		}
		faceList = make([]int, 0, len(tris))
		for f, present := range seen {
			if present {
				faceList = append(faceList, f)
			}
		}
	} else {
		faceList = collectFaces(facetFace)
	}
	for _, f := range faceList {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(loopsOf(f)) == 0 {
			return fmt.Errorf(`%w: a faceted face has no boundary loop`, decaderr.ErrBooleanFailed)
		}
		li := -1
		if isPlanar(f) {
			var n r3.Vec
			if budget.Replay {
				n = normalFixed[f]
			} else {
				n = faceNormal[f]
			}
			best := 0.0
			for idx, l := range loopsOf(f) {
				if err := budget.Step(); err != nil {
					return err
				}
				moment := l.replayMoment
				if !budget.Replay {
					moment = loopMoment[l]
				}
				if s := moment.Dot(n); s > best {
					best, li = s, idx
				}
			}
		}
		if li == -1 {
			longest := -1.0
			li = 0
			for idx, l := range loopsOf(f) {
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
		ls := loopsOf(f)
		ls[0], ls[li] = ls[li], ls[0]
		ls[0].Outer = true
	}
	if budget.Replay {
		result.ReplayFaceLoops = loopsFixed
	} else {
		result.FaceLoops = faceLoops
	}
	for _, chain := range chains {
		var err error
		result.Edges, err = replayAppend(budget, result.Edges, chain.edge)
		if err != nil {
			return err
		}
	}
	return budget.Err()
}

// collectFaces lists the distinct face ids in ascending order, so the walk over
// them spends the same budget steps before any refusal on every run.
func collectFaces(facetFace []int) []int {
	seen := make(map[int]struct{}, len(facetFace))
	out := make([]int, 0, len(facetFace))
	for _, f := range facetFace {
		if _, ok := seen[f]; ok {
			continue
		}
		seen[f] = struct{}{}
		out = append(out, f)
	}
	slices.Sort(out)
	return out
}
