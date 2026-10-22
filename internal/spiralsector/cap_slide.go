package spiralsector

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// carried is a vertex the Boolean made on a facet of the proxy: it takes the
// weighted sum of the displacements of the facet's corners.
type carried struct {
	vertex int
	corner [3]int // home slot of each corner, or -1 for a corner that did not move
	weight [3]float64
}

// link finds the vertices the Boolean made on the proxy's facets that moved,
// and answers false when a facet covers too many cells or the search runs past
// its budget.
func (st *restoreState) link(ctx context.Context) (bool, error) {
	m, home := st.m, st.home
	type key [3]int32
	cellOf := func(p r3.Vec) key {
		return key{int32(math.Floor(p.X / carryCell)), int32(math.Floor(p.Y / carryCell)),
			int32(math.Floor(p.Z / carryCell))}
	}
	grid := make(map[key][]int, len(m.Verts))
	for i, v := range m.Verts {
		if st.moved[i] < 0 && !st.onBlank[i] {
			k := cellOf(v)
			grid[k] = append(grid[k], i)
		}
	}
	cornerOf := func(p r3.Vec) int {
		if j, ok := st.slot[p]; ok && st.vertex[j] >= 0 {
			return j
		}
		return -1
	}
	pad := r3.Vec{X: carryPlane, Y: carryPlane, Z: carryPlane}
	done := make([]bool, len(m.Verts))
	visits := 0
	for ti, t := range home.pt {
		if ti%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		a, b, c := home.pv[t[0]], home.pv[t[1]], home.pv[t[2]]
		ca, cb, cc := cornerOf(a), cornerOf(b), cornerOf(c)
		if ca < 0 && cb < 0 && cc < 0 {
			continue
		}
		e1, e2 := b.Sub(a), c.Sub(a)
		nrm := e1.Cross(e2)
		nn := nrm.Dot(nrm)
		if nn == 0 {
			continue
		}
		nl := math.Sqrt(nn)
		lo := r3.Vec{X: min(a.X, b.X, c.X), Y: min(a.Y, b.Y, c.Y), Z: min(a.Z, b.Z, c.Z)}
		hi := r3.Vec{X: max(a.X, b.X, c.X), Y: max(a.Y, b.Y, c.Y), Z: max(a.Z, b.Z, c.Z)}
		k0, k1 := cellOf(lo.Sub(pad)), cellOf(hi.Add(pad))
		cells := int64(k1[0]-k0[0]+1) * int64(k1[1]-k0[1]+1) * int64(k1[2]-k0[2]+1)
		if cells > carryCells {
			return false, nil
		}
		for x := k0[0]; x <= k1[0]; x++ {
			for y := k0[1]; y <= k1[1]; y++ {
				for z := k0[2]; z <= k1[2]; z++ {
					for _, r := range grid[key{x, y, z}] {
						if done[r] {
							continue
						}
						if visits++; visits > carryVisits {
							return false, nil
						}
						p := m.Verts[r].Sub(a)
						if math.Abs(p.Dot(nrm))/nl > carryPlane {
							continue
						}
						w1 := p.Cross(e2).Dot(nrm) / nn
						w2 := e1.Cross(p).Dot(nrm) / nn
						w0 := 1 - w1 - w2
						if w0 < -carryEdge || w1 < -carryEdge || w2 < -carryEdge {
							continue
						}
						st.links = append(st.links, carried{vertex: r,
							corner: [3]int{ca, cb, cc}, weight: [3]float64{w0, w1, w2}})
						done[r] = true
					}
				}
			}
		}
	}
	return true, nil
}

// carry puts the given vertices made on facets of the proxy where the moves of
// their facets' corners take them.
func (st *restoreState) carry(links []carried) {
	for _, l := range links {
		var d r3.Vec
		for k, c := range l.corner {
			if c >= 0 {
				d = d.Add(st.byslot[c].Scale(l.weight[k]))
			}
		}
		st.next[l.vertex] = st.m.Verts[l.vertex].Add(d)
	}
}

// slotMoves refreshes the displacement of every moved vertex.
func (st *restoreState) slotMoves() {
	if st.byslot == nil {
		st.byslot = make([]r3.Vec, len(st.home.moved))
	}
	for j, i := range st.vertex {
		if i >= 0 {
			st.byslot[j] = st.next[i].Sub(st.m.Verts[i])
		}
	}
}

// facing reports whether every listed facet still faces the way it did. A facet
// that stood across its cone, as a wall does, may turn onto the cone, and then
// must face along the cone's outward normal.
func (st *restoreState) facing(facets []int) bool {
	m := st.m
	for _, f := range facets {
		t := m.Tris[f]
		old := m.Verts[t[1]].Sub(m.Verts[t[0]]).Cross(m.Verts[t[2]].Sub(m.Verts[t[0]]))
		now := st.next[t[1]].Sub(st.next[t[0]]).Cross(st.next[t[2]].Sub(st.next[t[0]]))
		lo, ln := math.Sqrt(old.Dot(old)), math.Sqrt(now.Dot(now))
		if lo < thinFacet {
			continue
		}
		if ln == 0 {
			return false
		}
		flat := false
		for _, v := range t {
			if st.moved[v] < 0 || math.Abs(old.Dot(st.normal[v])) >= wallAcross*lo {
				continue
			}
			flat = now.Dot(st.normal[v]) >= wallFlat*ln
			break
		}
		if !flat && old.Dot(now) <= keepTurn*lo*ln {
			return false
		}
	}
	return true
}

// open slides each cap out along its cone, as one piece, by the least distance
// that keeps every facet its vertices hold facing the way it did, then by a
// margin more where that still holds. The cap's own facets keep their shape;
// the distance is the width a flattened wall keeps. A cap no slide serves is
// blamed.
func (st *restoreState) open(ctx context.Context) (bool, error) {
	pieces := map[int][]int{}
	for i := range st.m.Verts {
		if st.moved[i] >= 0 {
			pieces[st.piece[i]] = append(pieces[st.piece[i]], i)
		}
	}
	tied := map[int][]carried{}
	for _, l := range st.links {
		for _, c := range l.corner {
			if c >= 0 {
				tied[st.piece[st.vertex[c]]] = append(tied[st.piece[st.vertex[c]]], l)
				break
			}
		}
	}
	for root, piece := range pieces {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if !st.slide(piece, tied[root]) {
			st.blame(root)
		}
	}
	return len(st.blamed) == 0, nil
}

// slide places one cap: its moved vertices and the vertices tied to them.
func (st *restoreState) slide(piece []int, links []carried) bool {
	var facets []int
	seen := map[int]struct{}{}
	note := func(v int) {
		for _, f := range st.incident[v] {
			if _, ok := seen[f]; !ok {
				seen[f] = struct{}{}
				facets = append(facets, f)
			}
		}
	}
	for _, i := range piece {
		note(i)
	}
	for _, l := range links {
		note(l.vertex)
	}
	base := make([]r3.Vec, len(piece))
	for k, i := range piece {
		base[k] = st.next[i]
	}
	put := func(r float64) bool {
		for k, i := range piece {
			st.next[i] = base[k].Add(st.out[i].Scale(r))
			if !st.withinBound(i) {
				return false
			}
		}
		st.slotMoves()
		st.carry(links)
		return st.facing(facets)
	}
	for _, sign := range [...]float64{1, -1} {
		for r := 0.0; r <= openReach; r += openStep {
			if !put(sign * r) {
				continue
			}
			for _, extra := range [...]float64{2 * openMargin, openMargin, openMargin / 2} {
				if put(sign * (r + extra)) {
					return true
				}
			}
			return put(sign * r)
		}
	}
	return false
}

// withinBound reports whether the restored position of moved vertex i is
// within the vertex's bound of the true surface: the source position is within
// its source bound of it, and the restored position within the distance the
// proxy's bound adds to the source bound of the source position.
func (st *restoreState) withinBound(i int) bool {
	j := st.moved[i]
	d := proofbound.DvLenUpper(proofbound.HeldDelta(st.home.home[j], st.next[i]))
	return !proofbound.IsNonFinite(d) && proofbound.AbsSumUpper(d, st.home.bound[j]) <= st.m.VertexBound[i]
}
