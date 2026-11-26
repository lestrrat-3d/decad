package brepfilletmesh_test

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// requirePatchFacesMatchEps requires PatchFaces to list exactly PatchEps's faces, each once.
func requirePatchFacesMatchEps(t *testing.T, fr *brepfilletmesh.Rings, want []*topo.Face) {
	t.Helper()
	require.Len(t, fr.PatchFaces, len(fr.PatchEps))
	require.Equal(t, want, fr.PatchFaces)
	for _, f := range fr.PatchFaces {
		_, ok := fr.PatchEps[f]
		require.True(t, ok)
	}
}

// staleRings is a Rings that still holds a previous emit's patch record, so a test sees whether an emit resets it.
func staleRings(patches int) *brepfilletmesh.Rings {
	stale := &topo.Face{}
	return &brepfilletmesh.Rings{
		Count:      1,
		RingV:      [][]int{nil, nil},
		SeamGap:    make([]float64, patches),
		Patches:    patches,
		PatchEps:   map[*topo.Face]float64{stale: 1},
		PatchFaces: []*topo.Face{stale},
	}
}

func rolesToFaces(t *testing.T, band breprecord.LoopBand, n int) (func(string) (*topo.Face, error), []*topo.Face) {
	t.Helper()
	faces := make([]*topo.Face, n)
	byRole := map[string]*topo.Face{}
	for p := range n {
		faces[p] = &topo.Face{}
		byRole[band.PatchRole(p)] = faces[p]
	}
	return func(role string) (*topo.Face, error) {
		f, ok := byRole[role]
		require.True(t, ok, role)
		return f, nil
	}, faces
}

func TestEmitPatchFacesMatchPatchEps(t *testing.T) {
	const n = 3
	band := breprecord.LoopBand{Kind: breprecord.BandFillet}
	faceOfRole, faces := rolesToFaces(t, band, n)
	fr := staleRings(n)
	in := brepfilletmesh.Input{
		Band:  band,
		Start: true,
		Loop:  &capblend.LoopMesh{Walks: make([]survey2d.SideWalk, n)},
	}
	bump := func(*topo.Face, float64) {}
	require.NoError(t, fr.Emit(in, &topo.Mesh{}, faceOfRole, bump))
	requirePatchFacesMatchEps(t, fr, faces)
}

func TestEmitPartialWithLevelPatchFacesMatchPatchEps(t *testing.T) {
	const n = 3
	band := breprecord.LoopBand{Kind: breprecord.BandFillet}
	faceOfRole, faces := rolesToFaces(t, band, n)
	fr := staleRings(0)
	in := brepfilletmesh.Input{Band: band, Start: true}
	partial := &partialband.PartialBandLayout{PatchSag: make([]float64, n)}
	bump := func(*topo.Face, float64) {}
	require.NoError(t, fr.EmitPartialWithLevel(in, partial, &topo.Mesh{}, 0, faceOfRole, bump))
	requirePatchFacesMatchEps(t, fr, faces)
}
