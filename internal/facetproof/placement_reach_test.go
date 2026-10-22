package facetproof_test

import (
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placementFold loads testdata/placement_fold.json: 118 facets of a restored
// spiral tooth cap, cropped around the four facets that fold when a quarter
// turn about Z is rounded to float64, and the exact image of each vertex under
// that turn. The cap's facets sit 1e-15 mm from their neighbours, so the float
// box of an image vertex cannot clear the fold.
func placementFold(t *testing.T) (exact []proof.Xpt, held []r3.Vec, tris [][3]int) {
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
	basis, tr := turn.Basis(), turn.Translation()
	exact = make([]proof.Xpt, len(d.V))
	held = make([]r3.Vec, len(d.V))
	for i, v := range d.V {
		exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, r3.Vec{X: v[0], Y: v[1], Z: v[2]})
		held[i] = exact[i].Vec()
	}
	return exact, held, d.T
}

func TestKeepPlacedEmbeddedReach(t *testing.T) {
	t.Parallel()

	t.Run("refuses the fold with no reach", func(t *testing.T) {
		t.Parallel()
		exact, held, tris := placementFold(t)
		_, err := facetproof.KeepPlacedEmbedded(t.Context(), exact, held, tris)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})

	t.Run("clears the fold within the reach and charges every move", func(t *testing.T) {
		t.Parallel()
		const reach = 1e-3
		exact, held, tris := placementFold(t)
		first := append([]r3.Vec(nil), held...)
		charge, err := facetproof.KeepPlacedEmbedded(gaptol.With(t.Context(), reach), exact, held, tris)
		require.NoError(t, err)
		require.NotNil(t, charge)

		moved := 0
		for i := range held {
			if held[i] == first[i] {
				continue
			}
			moved++
			gap, _ := meshbool.CoordDistance(exact[i], held[i]).Float64()
			require.LessOrEqual(t, gap, reach*1.001, "vertex %d stays within the reach of its exact image", i)
			require.GreaterOrEqual(t, charge[i], gap, "vertex %d is charged at least the distance it moved", i)
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
		_, err := facetproof.KeepPlacedEmbedded(gaptol.With(t.Context(), 1e-30), exact, held, tris)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}
