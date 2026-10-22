package spiralsector

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// finish checks the restored positions and prices the move. A cap whose facets
// meet others, or that a placement would fold, is blamed.
func (st *restoreState) finish(ctx context.Context) (CapRestoration, bool, error) {
	m := st.m
	changed := make([]bool, len(m.Verts))
	delta := 0.0
	for i, v := range m.Verts {
		move := proofbound.DvLenUpper(proofbound.HeldDelta(v, st.next[i]))
		if proofbound.IsNonFinite(move) {
			return CapRestoration{}, false, nil
		}
		delta = max(delta, move)
		changed[i] = move > 0
	}
	if delta == 0 {
		return CapRestoration{}, false, nil
	}
	pairs, err := st.folds(ctx, changed)
	if err != nil {
		return CapRestoration{}, false, err
	}
	if len(pairs) > 0 {
		// A vertex of a folded facet stays at its proxy position; the facets it
		// holds then slope up to it from the restored cap.
		for _, p := range pairs {
			for _, f := range p {
				for _, v := range m.Tris[f] {
					if st.moved[v] >= 0 {
						st.pinned = append(st.pinned, v)
					}
				}
			}
		}
		if len(st.pinned) == 0 {
			st.blameNear(pairs)
		}
		return CapRestoration{}, false, nil
	}
	bad, err := st.unplaceable(ctx, changed)
	if err != nil {
		return CapRestoration{}, false, err
	}
	if len(bad) > 0 {
		st.blameNear(bad)
		return CapRestoration{}, false, nil
	}
	var before [][3]int
	for f, t := range m.Tris {
		if f%256 == 0 {
			if err := ctx.Err(); err != nil {
				return CapRestoration{}, false, err
			}
		}
		if changed[t[0]] || changed[t[1]] || changed[t[2]] {
			before = append(before, t)
		}
	}
	volume := proofbound.SweptVolumeAllow(delta, proofbound.PerturbedAreaUpper(m.Verts, before, delta))
	area := proofbound.AbsSumUpper(facetproof.MeshAreaUpper(m.Verts, before),
		facetproof.MeshAreaUpper(st.next, before))
	if proofbound.IsNonFinite(volume) || proofbound.IsNonFinite(area) {
		return CapRestoration{}, false, nil
	}
	return CapRestoration{Verts: st.next, VolumeCharge: volume, AreaCharge: area}, true, nil
}

// folds returns the pairs of facets that meet other than as neighbours, among
// those that touch a changed vertex. It reads the restored mesh as it is. A
// pair that only misses by an ulp is not counted here: a later placement's
// rounding folds it only if the placement probes (unplaceable) say so.
func (st *restoreState) folds(ctx context.Context, changed []bool) ([][2]int, error) {
	n := len(st.m.Verts)
	h := meshbool.HeldRounding{
		Verts: st.next, Tris: st.m.Tris, Exact: make([]proof.Xpt, n),
		Moved: changed, Movable: make([]bool, n),
	}
	for i, v := range st.next {
		h.Exact[i] = proof.XptOf(v)
	}
	return meshbool.HeldFolds(ctx, h)
}
