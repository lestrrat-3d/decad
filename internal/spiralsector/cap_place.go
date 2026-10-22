package spiralsector

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// placementProbes are rigid placements the restored mesh has to survive. A
// held mesh that no placement can round without folding is not worth
// publishing, since the caller's next Transform refuses it: the restored cap
// nearest a fold stays at its proxy positions instead. The first probe is a
// quarter turn about Z, whose float matrix is not exactly orthogonal; the
// second is a generic turn about an oblique axis with a translation.
func placementProbes() ([]r3.Transform, error) {
	quarter, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	if err != nil {
		return nil, err
	}
	oblique, err := r3.RotationAround(r3.Vec{X: 3.1, Y: -2.7, Z: 5.3}, r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.7))
	if err != nil {
		return nil, err
	}
	return []r3.Transform{quarter, oblique}, nil
}

// unplaceable returns the facets of the restored mesh that a probe placement
// folds after the float-box repair a Transform runs.
func (st *restoreState) unplaceable(ctx context.Context, changed []bool) ([][2]int, error) {
	probes, err := placementProbes()
	if err != nil {
		return nil, err
	}
	n := len(st.m.Verts)
	ring := st.ring(changed)
	var bad [][2]int
	seen := make(map[int]struct{})
	for _, probe := range probes {
		basis, tr := probe.Basis(), probe.Translation()
		h := meshbool.HeldRounding{
			Verts: make([]r3.Vec, n), Tris: st.m.Tris, Exact: make([]proof.Xpt, n),
			Moved: make([]bool, n), Movable: make([]bool, n),
		}
		for i := range n {
			h.Exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, st.next[i])
			h.Verts[i] = h.Exact[i].Vec()
			if !proof.XptDenotes(h.Exact[i], h.Verts[i]) {
				h.Movable[i] = true
				h.Moved[i] = ring[i]
			}
		}
		_, failing, err := meshbool.RepairHeldEmbedding(ctx, h)
		if err != nil {
			return nil, err
		}
		for _, f := range failing {
			if _, dup := seen[f]; !dup {
				seen[f] = struct{}{}
				bad = append(bad, [2]int{f, f})
			}
		}
	}
	return bad, nil
}

// blameNear marks the caps whose moved vertices lie within reach of a facet of
// a pair as failed.
func (st *restoreState) blameNear(pairs [][2]int) {
	const reach = 1.0 // millimetres
	type key [3]int32
	cellOf := func(p r3.Vec) key {
		return key{int32(math.Floor(p.X / reach)), int32(math.Floor(p.Y / reach)), int32(math.Floor(p.Z / reach))}
	}
	grid := make(map[key][]int)
	for i := range st.m.Verts {
		if st.moved[i] >= 0 {
			k := cellOf(st.next[i])
			grid[k] = append(grid[k], i)
		}
	}
	for _, p := range pairs {
		for _, f := range p {
			for _, v := range st.m.Tris[f] {
				c := cellOf(st.next[v])
				for dx := int32(-1); dx <= 1; dx++ {
					for dy := int32(-1); dy <= 1; dy++ {
						for dz := int32(-1); dz <= 1; dz++ {
							for _, i := range grid[key{c[0] + dx, c[1] + dy, c[2] + dz}] {
								if st.next[i].Sub(st.next[v]).Len() <= reach {
									st.blame(i)
								}
							}
						}
					}
				}
			}
		}
	}
}

// ring marks the vertices of the facets that can meet something new after the
// restoration: every facet that holds a changed vertex, except a cap facet none
// of whose vertices touches another group. A cap's inside keeps its shape, so
// only its edge and what lies beside it need checking by a placement probe.
func (st *restoreState) ring(changed []bool) []bool {
	n := len(st.m.Verts)
	edge := make([]bool, n) // a vertex some other group's facet holds
	first := make([]int, n)
	for i := range first {
		first[i] = -1
	}
	for f, t := range st.m.Tris {
		for _, v := range t {
			if first[v] < 0 {
				first[v] = st.m.Src[f]
			} else if first[v] != st.m.Src[f] {
				edge[v] = true
			}
		}
	}
	ring := make([]bool, n)
	for f, t := range st.m.Tris {
		if !changed[t[0]] && !changed[t[1]] && !changed[t[2]] {
			continue
		}
		if _, isCap := st.home.caps[st.m.Src[f]]; isCap && !edge[t[0]] && !edge[t[1]] && !edge[t[2]] {
			continue
		}
		ring[t[0]], ring[t[1]], ring[t[2]] = true, true, true
	}
	return ring
}
