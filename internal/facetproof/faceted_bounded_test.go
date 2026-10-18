package facetproof_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAuditFacetedMeshBoundedMatchesLegacy(t *testing.T) {
	for _, tc := range []struct {
		name  string
		build func() ([]r3.Vec, [][3]int)
	}{
		{"tetrahedron", tetraMesh},
		{"box", func() ([]r3.Vec, [][3]int) {
			return testBox(nil, nil, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 5}, false)
		}},
		{"fractional box", func() ([]r3.Vec, [][3]int) {
			return testBox(nil, nil, r3.Vec{X: 0.1, Y: -0.3, Z: 0.7},
				r3.Vec{X: 1.3, Y: 0.9, Z: 2.1}, false)
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			verts, tris := tc.build()
			want, err := facetproof.AuditFacetedMesh(t.Context(), verts, tris)
			require.NoError(t, err)
			steps := 0
			got, admitted, err := facetproof.AuditFacetedMeshBounded(t.Context(), countingBudget(&steps), verts, tris)
			require.NoError(t, err)
			require.True(t, admitted)
			require.LessOrEqual(t, steps, 5788)
			requireAuditsEqual(t, want, got)
			wantFloat, wantBound, wantRat, err := facetproof.VolumeFromAudit(want, 1e-9)
			require.NoError(t, err)
			gotFloat, gotBound, gotRat, admitted, err := facetproof.VolumeFromAuditBounded(
				countingBudget(&steps), got, 1e-9)
			require.NoError(t, err)
			require.True(t, admitted)
			require.LessOrEqual(t, steps, 6044)
			require.Equal(t, math.Float64bits(wantFloat), math.Float64bits(gotFloat))
			require.Equal(t, math.Float64bits(wantBound), math.Float64bits(gotBound))
			require.Zero(t, wantRat.Cmp(gotRat))
		})
	}
}

func TestAuditFacetedMeshBoundedRefusesUnsupportedShells(t *testing.T) {
	verts, tris := testBox(nil, nil, r3.Vec{}, r3.Vec{X: 2, Y: 3, Z: 5}, false)
	otherV, otherT := testBox(nil, nil, r3.Vec{X: 10}, r3.Vec{X: 12, Y: 3, Z: 5}, false)
	twoV, twoT := joinMeshes(verts, tris, otherV, otherT)
	inverted := append([][3]int(nil), tris...)
	for i, tri := range inverted {
		inverted[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	nonfinite := append([]r3.Vec(nil), verts...)
	nonfinite[0].X = math.NaN()
	extreme := append([]r3.Vec(nil), verts...)
	extreme[0].X = math.Ldexp(1, 130)
	tiny := append([]r3.Vec(nil), verts...)
	tiny[0].X = math.Ldexp(1, -461)
	tooManyVertices := make([]r3.Vec, 513)
	copy(tooManyVertices, verts)
	tooManyFacets := make([][3]int, 513)
	copy(tooManyFacets, tris)
	for _, tc := range []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
	}{
		{"open", verts, tris[1:]},
		{"inverted", verts, inverted},
		{"two components", twoV, twoT},
		{"nonfinite coordinate", nonfinite, tris},
		{"arithmetic preflight", extreme, tris},
		{"tiny exponent", tiny, tris},
		{"vertex limit", tooManyVertices, tris},
		{"facet limit", verts, tooManyFacets},
		{"bad index", verts, [][3]int{{0, 1, len(verts)}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			audit, admitted, err := facetproof.AuditFacetedMeshBounded(t.Context(),
				proofbound.NewWallWorkBudget(6044), tc.verts, tc.tris)
			require.NoError(t, err)
			require.False(t, admitted)
			require.Nil(t, audit)
		})
	}
}

func TestAuditFacetedMeshBoundedWorkAndCancellation(t *testing.T) {
	verts, tris := tetraMesh()
	var charged int
	_, admitted, err := facetproof.AuditFacetedMeshBounded(t.Context(),
		countingBudget(&charged), verts, tris)
	require.NoError(t, err)
	require.True(t, admitted)
	_, admitted, err = facetproof.AuditFacetedMeshBounded(t.Context(),
		proofbound.NewWallWorkBudget(uint64(charged)), verts, tris)
	require.NoError(t, err)
	require.True(t, admitted)
	_, admitted, err = facetproof.AuditFacetedMeshBounded(t.Context(),
		proofbound.NewWallWorkBudget(uint64(charged-1)), verts, tris)
	require.NoError(t, err)
	require.False(t, admitted)

	_, admitted, err = facetproof.AuditFacetedMeshBounded(t.Context(),
		proofbound.NewWallWorkBudget(1), verts, tris)
	require.NoError(t, err)
	require.False(t, admitted)

	ctx, cancel := context.WithCancel(t.Context())
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 500 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, admitted, err = facetproof.AuditFacetedMeshBounded(ctx, budget, verts, tris)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Equal(t, 500, steps)
}

func TestAuditFacetedMeshBoundedPrefersCancellationAtWallLimit(t *testing.T) {
	verts, tris := tetraMesh()
	ctx, cancel := context.WithCancel(t.Context())
	wall := proofbound.NewWallWorkBudget(29)
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 30 {
				cancel()
			}
			return wall.Step()
		},
		ErrFn: wall.Err, // This deliberately does not inspect ctx.
	}
	_, admitted, err := facetproof.AuditFacetedMeshBounded(ctx, budget, verts, tris)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Equal(t, 30, steps)
}

func TestVolumeFromAuditBoundedRefusesArithmeticAndWork(t *testing.T) {
	verts, tris := tetraMesh()
	audit, admitted, err := facetproof.AuditFacetedMeshBounded(t.Context(),
		proofbound.NewWallWorkBudget(5788), verts, tris)
	require.NoError(t, err)
	require.True(t, admitted)
	var volumeRat *big.Rat
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(
		proofbound.NewWallWorkBudget(1), audit, 0)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Nil(t, volumeRat)
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(
		proofbound.NewWallWorkBudget(256), audit, 0)
	require.NoError(t, err)
	require.True(t, admitted)
	require.NotNil(t, volumeRat)

	tooLarge := &facetproof.MeshAudit{CompVol: []*big.Rat{
		new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 4096)),
	}}
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(
		proofbound.NewWallWorkBudget(256), tooLarge, 0)
	require.NoError(t, err)
	require.False(t, admitted)
	require.Nil(t, volumeRat)
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(
		proofbound.NewWallWorkBudget(256), audit, math.Inf(1))
	require.NoError(t, err)
	require.False(t, admitted)
	require.Nil(t, volumeRat)

	ctx, cancel := context.WithCancel(t.Context())
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 30 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(budget, audit, 0)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Nil(t, volumeRat)
	require.Equal(t, 30, steps)
}

func TestVolumeFromAuditBoundedRequiresOperationAwareCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := proofbound.NewWallWorkBudgetWithOperation(0, proofbound.NewWorkBudget(ctx))
	_, _, volumeRat, admitted, err := facetproof.VolumeFromAuditBounded(budget, nil, 0)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Nil(t, volumeRat)

	ctx, cancel = context.WithCancel(t.Context())
	wall := proofbound.NewWallWorkBudget(29)
	steps := 0
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 30 {
				cancel()
			}
			return wall.Step()
		},
		ErrFn: func() error {
			if err := ctx.Err(); err != nil {
				return err
			}
			return wall.Err()
		},
	}
	audit := &facetproof.MeshAudit{CompVol: []*big.Rat{big.NewRat(1, 1)}}
	_, _, volumeRat, admitted, err = facetproof.VolumeFromAuditBounded(budget, audit, 0)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, admitted)
	require.Nil(t, volumeRat)
	require.Equal(t, 30, steps)
}
