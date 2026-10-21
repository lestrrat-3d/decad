package spiralsector

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CapHome pairs each cap vertex a temporary tooth proxy moved with the
// position the source tooth gives it. The proxy exists only so the generic
// mesh Boolean sees the tooth's caps apart from the blank's cones; the
// published result puts the vertices back where the source holds them.
type CapHome struct {
	moved []r3.Vec
	home  []r3.Vec
	bound []float64 // the source vertex bound at home
}

// NewCapHome pairs proxy and home vertices index by index and keeps the pairs
// that differ. bound is the vertex bound of home. The three slices must have
// one entry per vertex.
func NewCapHome(proxy, home []r3.Vec, bound []float64) *CapHome {
	h := new(CapHome)
	if len(proxy) != len(home) || len(bound) != len(home) {
		return h
	}
	for i, p := range proxy {
		if p == home[i] {
			continue
		}
		h.moved = append(h.moved, p)
		h.home = append(h.home, home[i])
		h.bound = append(h.bound, bound[i])
	}
	return h
}

// Append adds the pairs of other. Both must not be used afterwards except
// through the receiver.
func (h *CapHome) Append(other *CapHome) {
	h.moved = append(h.moved, other.moved...)
	h.home = append(h.home, other.home...)
	h.bound = append(h.bound, other.bound...)
}

// Len is the number of moved vertices.
func (h *CapHome) Len() int { return len(h.moved) }

// CapRestoration is a result mesh whose proxy cap vertices are back at their
// source positions, with what the move adds to the result's bounds.
type CapRestoration struct {
	Verts []r3.Vec
	// VolumeCharge is added to the result's symmetric-difference volume bound.
	VolumeCharge float64
	// AreaCharge is added to the result's area slack.
	AreaCharge float64
}

// RestoreCaps returns verts with every vertex that sits at a moved proxy
// position put back at its source position. It answers false when nothing can
// be restored, or when a restoration would be unsound: two moved vertices
// share a position, a result vertex bound is below the source bound at the
// restored position, or a coordinate is not finite. The caller audits the
// restored mesh and keeps the proxy-position mesh when the audit refuses.
//
// The proxy position of a vertex is within its result bound of the true
// solid (OffsetToothMesh raised the bound by the move), and the source
// position is within the source bound, which that bound covers, so the
// vertex and facet bounds stand unchanged. The restored mesh differs from the
// held one only on the facets that touch a restored vertex, where every
// vertex moved by at most the largest restored distance. The volume between
// the two meshes is that distance times the area along the straight path, and
// the area slack adds both meshes' changed-facet areas.
func RestoreCaps(ctx context.Context, home *CapHome, verts []r3.Vec, tris [][3]int,
	vertexBound []float64) (CapRestoration, bool, error) {
	if home == nil || len(home.moved) == 0 || len(vertexBound) != len(verts) {
		return CapRestoration{}, false, nil
	}
	slot := make(map[r3.Vec]int, len(home.moved))
	for i, p := range home.moved {
		if _, dup := slot[p]; dup {
			return CapRestoration{}, false, nil
		}
		slot[p] = i
	}
	next := append([]r3.Vec(nil), verts...)
	changed := make([]bool, len(verts))
	delta, restored := 0.0, 0
	for i, v := range verts {
		j, ok := slot[v]
		if !ok {
			continue
		}
		if !proofbound.FiniteVec(home.home[j]) || vertexBound[i] < home.bound[j] {
			return CapRestoration{}, false, nil
		}
		move := proofbound.DvLenUpper(proofbound.HeldDelta(v, home.home[j]))
		if proofbound.IsNonFinite(move) {
			return CapRestoration{}, false, nil
		}
		delta = max(delta, move)
		next[i], changed[i] = home.home[j], true
		restored++
	}
	if restored == 0 || delta == 0 {
		return CapRestoration{}, false, nil
	}
	var touched [][3]int
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return CapRestoration{}, false, err
			}
		}
		if changed[tri[0]] || changed[tri[1]] || changed[tri[2]] {
			touched = append(touched, tri)
		}
	}
	volume := proofbound.SweptVolumeAllow(delta, proofbound.PerturbedAreaUpper(verts, touched, delta))
	area := proofbound.AbsSumUpper(facetproof.MeshAreaUpper(verts, touched),
		facetproof.MeshAreaUpper(next, touched))
	if proofbound.IsNonFinite(volume) || proofbound.IsNonFinite(area) {
		return CapRestoration{}, false, nil
	}
	return CapRestoration{Verts: next, VolumeCharge: volume, AreaCharge: area}, true, nil
}
