package booleaneval

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftUnionFacetCorrespondence(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	budget := proofbound.NewStrictReplayBudget().WorkBudget(t.Context())
	binding, err := bindLoftUnionFacets(f.body, f.loft, f.loftBool, f.loft, f.loftBool, budget)
	require.NoError(t, err)
	require.NotNil(t, binding)
	unverified := *f.loft
	unverified.SymDiffOK = false
	missing, err := bindLoftUnionFacets(f.body, &unverified, f.loftBool, f.loft, f.loftBool, budget)
	require.NoError(t, err)
	require.Nil(t, missing)
	original, ok := binding.originalEdge(f.loftFacet, f.loftEdge)
	require.True(t, ok)
	require.Equal(t, f.loftFacet, original)
	capFacet := capSide(t, f)
	ok, err = binding.startCapFace(f.loft.Source[capFacet], budget)
	require.NoError(t, err)
	require.True(t, ok)
	ok, err = binding.startCapFace(f.loft.Source[f.loftFacet], budget)
	require.NoError(t, err)
	require.False(t, ok)
	ok, err = binding.startCapFace(new(topo.Face), budget)
	require.NoError(t, err)
	require.False(t, ok)
	joined := *f.loft
	joined.Source = append([]*topo.Face(nil), f.loft.Source...)
	joined.Source[f.loftFacet] = joined.Source[capFacet]
	binding.joined = &joined
	ok, err = binding.startCapFace(joined.Source[capFacet], budget)
	require.NoError(t, err)
	require.False(t, ok)
	binding.joined = f.loft

	changed := *f.loftBool
	changed.Verts = append([]r3.Vec(nil), f.loftBool.Verts...)
	changed.Verts[changed.Tris[f.loftFacet][2]].X += 0.125
	binding.newBool = &changed
	_, ok = binding.originalEdge(f.loftFacet, f.loftEdge)
	require.False(t, ok)
}
