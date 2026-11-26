package brepbandmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestVolumeTermsFollowPatchFaceOrder requires VolumeTerms to list a fillet band's patch terms in PatchFaces order on
// every call. The terms are summed in float, so reading PatchEps in Go's map order changes the sum's last bit from run
// to run.
func TestVolumeTermsFollowPatchFaceOrder(t *testing.T) {
	const patches = 24
	faces := make([]*topo.Face, patches)
	eps := make(map[*topo.Face]float64, patches)
	area := make(map[*topo.Face]float64, patches)
	for i := range faces {
		faces[i] = &topo.Face{}
		eps[faces[i]] = 0.01 * float64(i+1)
		area[faces[i]] = 1 + float64(i)*0.37
	}
	bands := []Band{{fillet: &brepfilletmesh.Rings{PatchFaces: faces, PatchEps: eps}}}
	want := make([]uint64, 0, patches+1)
	want = append(want, math.Float64bits(0))
	for _, f := range faces {
		want = append(want, math.Float64bits(proofbound.ProductUpper(area[f], eps[f])))
	}
	for call := range 30 {
		terms := VolumeTerms(bands, area)
		require.Len(t, terms, len(want), "call %d", call)
		for i, term := range terms {
			require.Equal(t, want[i], math.Float64bits(term), "call %d term %d", call, i)
		}
	}
}
