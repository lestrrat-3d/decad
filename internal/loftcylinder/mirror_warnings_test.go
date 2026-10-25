package loftcylinder

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This fixture tests warning transport at the final mirrored-shell builder,
// independently of whether a particular reflection needs positive-reach repair.
func TestBuildMirrorWithWarningsAggregatesFreshHalves(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	allowance := 0.1
	lower := &topo.Body{OperationWarnings: []opdiagnostic.Record{
		{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1, DisplacementBound: 0.001, AllowedReach: 0.01},
		{Code: opdiagnostic.CodeChainAllowanceApplied, Operand: 0, TouchedBound: 0.03,
			PairChordLimit: 0.02, ChainAllowance: &allowance},
	}}
	upper := &topo.Body{OperationWarnings: []opdiagnostic.Record{
		{Code: opdiagnostic.CodeEmbeddingReachApplied, Operand: -1, DisplacementBound: 0.002, AllowedReach: 0.01},
	}}
	body, err := buildMirrorWithWarnings(t.Context(), env, mirrorWarningCube(), lower, upper)
	require.NoError(t, err)
	require.True(t, body.Solid)
	require.Equal(t, 1.0, body.Volume.Value.Base())
	require.InDelta(t, 6, body.Area.Value.Base(), 1e-12)
	require.Len(t, body.OperationWarnings, 2)
	chain, embedding := body.OperationWarnings[0], body.OperationWarnings[1]
	require.Equal(t, opdiagnostic.CodeChainAllowanceApplied, chain.Code)
	require.Equal(t, -1, chain.Operand)
	require.Equal(t, 0.03, chain.TouchedBound)
	require.Equal(t, opdiagnostic.CodeEmbeddingReachApplied, embedding.Code)
	require.Equal(t, -1, embedding.Operand)
	require.GreaterOrEqual(t, embedding.DisplacementBound, 0.003)
	require.Less(t, embedding.DisplacementBound, 0.0031)
	*chain.ChainAllowance = 7
	require.Equal(t, 0.1, allowance)
	require.Equal(t, 0, lower.OperationWarnings[1].Operand)
	require.Equal(t, 0.001, lower.OperationWarnings[0].DisplacementBound)

	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	failed, err := buildMirrorWithWarnings(cancelled, env, mirrorWarningCube(), lower, upper)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, failed)
	require.Equal(t, 0.002, upper.OperationWarnings[0].DisplacementBound)
}

func mirrorWarningCube() facetedrecord.Payload {
	p := facetedrecord.Payload{Xform: r3.Identity(), VertexBound: make([]float64, 8), DPair: 2}
	for i := range 8 {
		p.Verts = append(p.Verts, r3.NewVec(float64(i&1), float64(i>>1&1), float64(i>>2&1)))
	}
	p.Tris = [][3]int{
		{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
		{2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5},
	}
	for range 6 {
		p.Groups = append(p.Groups, facetedrecord.Group{Planar: true})
	}
	for i := range p.Tris {
		p.Src = append(p.Src, i/2)
	}
	return p
}
