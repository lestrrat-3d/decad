package facetproof_test

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placementFold loads testdata/placement_fold.json: 118 facets of a restored
// spiral tooth cap, cropped around the four facets that fold when a quarter
// turn about Z is rounded to float64, and the exact image of each vertex under
// that turn. The cap's facets sit 1e-15 mm from their neighbours, so the float
// box of an image vertex cannot clear the fold.
func placementFold(t *testing.T) ([]proof.Xpt, []r3.Vec, [][3]int) {
	t.Helper()
	verts, tris, turn := placementFoldInput(t)
	basis, tr := turn.Basis(), turn.Translation()
	exact := make([]proof.Xpt, len(verts))
	held := make([]r3.Vec, len(verts))
	for i, v := range verts {
		exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v)
		held[i] = exact[i].Vec()
	}
	return exact, held, tris
}

func placementFoldInput(t *testing.T) ([]r3.Vec, [][3]int, r3.Transform) {
	t.Helper()
	raw, err := os.ReadFile("testdata/placement_fold.json")
	require.NoError(t, err)
	var d struct {
		V [][3]float64 `json:"V"`
		T [][3]int     `json:"T"`
	}
	require.NoError(t, json.Unmarshal(raw, &d))
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	verts := make([]r3.Vec, len(d.V))
	for i, v := range d.V {
		verts[i] = r3.Vec{X: v[0], Y: v[1], Z: v[2]}
	}
	return verts, d.T, turn
}

func TestKeepPlacedEmbeddedReach(t *testing.T) {
	t.Parallel()

	t.Run("refuses the fold with no reach", func(t *testing.T) {
		t.Parallel()
		exact, held, tris := placementFold(t)
		_, summary, err := facetproof.KeepPlacedEmbeddedWithSummary(t.Context(), exact, held, tris)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Nil(t, summary)
	})

	t.Run("clears the fold within the reach and charges every move", func(t *testing.T) {
		t.Parallel()
		const reach = 1e-3
		exact, held, tris := placementFold(t)
		first := append([]r3.Vec(nil), held...)
		charge, summary, err := facetproof.KeepPlacedEmbeddedWithSummary(gaptol.With(t.Context(), reach), exact, held, tris)
		require.NoError(t, err)
		require.NotNil(t, charge)
		require.NotNil(t, summary)
		require.Equal(t, reach, summary.AllowedReach)
		records := summary.Diagnostics()
		require.Len(t, records, 1)
		require.Equal(t, opdiagnostic.CodeEmbeddingReachApplied, records[0].Code)
		require.Equal(t, opdiagnostic.ChangeEmbeddingReach, records[0].Change)
		require.Equal(t, summary.DisplacementBound, records[0].DisplacementBound)

		moved := 0
		for i := range held {
			if held[i] == first[i] {
				continue
			}
			moved++
			gap, _ := meshbool.CoordDistance(exact[i], held[i]).Float64()
			require.LessOrEqual(t, gap, reach*1.001, "vertex %d stays within the reach of its exact image", i)
			require.GreaterOrEqual(t, charge[i], gap, "vertex %d is charged at least the distance it moved", i)
			require.GreaterOrEqual(t, summary.DisplacementBound, gap, "the summary covers vertex %d", i)
		}
		require.Positive(t, moved)

		h := meshbool.HeldRounding{
			Verts: held, Tris: tris, Exact: exact,
			Moved: make([]bool, len(held)), Movable: make([]bool, len(held)),
		}
		for i := range held {
			h.Moved[i] = !proof.XptDenotes(exact[i], held[i])
		}
		folds, err := meshbool.HeldFolds(t.Context(), h)
		require.NoError(t, err)
		require.Empty(t, folds, "the placed cap is embedded")
	})

	t.Run("a reach too small to clear it still refuses", func(t *testing.T) {
		t.Parallel()
		exact, held, tris := placementFold(t)
		_, summary, err := facetproof.KeepPlacedEmbeddedWithSummary(gaptol.With(t.Context(), 1e-30), exact, held, tris)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Nil(t, summary)
	})

	t.Run("rigid placement carries its accepted repair", func(t *testing.T) {
		t.Parallel()
		const reach = 1e-3
		verts, tris, turn := placementFoldInput(t)
		ctx := gaptol.With(t.Context(), reach)
		placed, err := facetproof.PlaceMesh(ctx, proofbound.NewWorkBudget(ctx), verts, tris,
			make([]float64, len(verts)), 0, 0, 0, 0, turn)
		require.NoError(t, err)
		require.NotNil(t, placed.EmbeddingReach)
		require.Equal(t, reach, placed.EmbeddingReach.AllowedReach)
		basis, tr := turn.Basis(), turn.Translation()
		for i, v := range verts {
			exact := proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v)
			gap, _ := meshbool.CoordDistance(exact, placed.Verts[i]).Float64()
			require.GreaterOrEqual(t, placed.EmbeddingReach.DisplacementBound, gap, "vertex %d", i)
			require.GreaterOrEqual(t, placed.VertexBound[i], gap, "vertex %d", i)
		}
	})

	t.Run("cancellation publishes no repair summary", func(t *testing.T) {
		t.Parallel()
		exact, held, tris := placementFold(t)
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, summary, err := facetproof.KeepPlacedEmbeddedWithSummary(gaptol.With(ctx, 1e-3), exact, held, tris)
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, summary)
	})
}
