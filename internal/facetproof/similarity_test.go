package facetproof_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestScaleMeshEmbeddingSummary(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 2}, {Y: 2}, {Z: 2}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	bounds := []float64{0.2, 0.2, 0.2, 0.2}
	anchor := r3.Vec{X: 0.1, Y: 0.2, Z: 0.3}

	t.Run("inherited bounds and ordinary rounding do not warn", func(t *testing.T) {
		t.Parallel()
		ctx := gaptol.With(t.Context(), 0.01)
		scaled, err := facetproof.ScaleMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris, bounds,
			0.1, 0.1, anchor, 1.1)
		require.NoError(t, err)
		require.Nil(t, scaled.EmbeddingReach)
		require.Positive(t, scaled.Round)
		for i, v := range verts {
			want := anchor.Add(v.Sub(anchor).Scale(1.1))
			require.Equal(t, want, scaled.Verts[i])
			require.GreaterOrEqual(t, scaled.VertexBound[i], 1.1*bounds[i])
		}
	})

	t.Run("a canceled image returns no repair", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		scaled, err := facetproof.ScaleMesh(gaptol.With(ctx, 0.01), proofbound.NewWorkBudget(ctx),
			verts, tris, bounds, 0.1, 0.1, anchor, 1.1)
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, scaled.EmbeddingReach)
		require.Nil(t, scaled.Verts)
	})
}
