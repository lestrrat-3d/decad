package booleaneval_test

import (
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// TestGrowFolded holds the retry of a folded result to its rule
// (agent-docs/gap-tolerance-design.md §5.2 item 7): every piece of the second
// operand grows by the same step of the reach, outward for a Union and a Cut
// and inward for an Intersect, smallest step first; a step that folds or meets
// a contact again is passed over; the first other outcome is the result; and a
// document with no reach, or a chain no step answers, keeps the fold.
//
// Shown to fail: growing an Intersect outward, or stopping at the first fold,
// turns the first subtest red. Evaluate reaches this search only for a fold
// of the stitched result, as the large-nine spinning wheel's brake spring
// Union met, never for one met while meshing an operand.
func TestGrowFolded(t *testing.T) {
	t.Parallel()
	const reach = 0.01
	fold := &meshbool.HeldFoldError{}
	ctx := gaptol.With(t.Context(), reach)
	t.Run("first step that does not fold", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			op   meshbool.OperationKind
			sign float64
		}{{meshbool.OpUnion, 1}, {meshbool.OpCut, 1}, {meshbool.OpIntersect, -1}} {
			var seen []float64
			_, err := booleaneval.GrowFolded(ctx, fold, tc.op, 2, func(grows map[int]float64) (booleaneval.Evaluation, error) {
				require.Len(t, grows, 2)
				require.Equal(t, grows[0], grows[1], "every piece grows by the same step")
				seen = append(seen, grows[0])
				if len(seen) == 1 {
					return booleaneval.Evaluation{}, fold
				}
				if len(seen) == 2 {
					return booleaneval.Evaluation{}, &meshbool.TouchContactError{What: meshbool.TouchCoplanar}
				}
				return booleaneval.Evaluation{}, nil
			})
			require.NoError(t, err)
			require.Equal(t, []float64{tc.sign * reach / 4096, tc.sign * reach / 512, tc.sign * reach / 64}, seen)
		}
	})
	t.Run("another refusal ends the search", func(t *testing.T) {
		t.Parallel()
		other := errors.New("empty")
		calls := 0
		_, err := booleaneval.GrowFolded(ctx, fold, meshbool.OpUnion, 1, func(map[int]float64) (booleaneval.Evaluation, error) {
			calls++
			return booleaneval.Evaluation{}, other
		})
		require.ErrorIs(t, err, other)
		require.Equal(t, 1, calls)
	})
	t.Run("no step answers", func(t *testing.T) {
		t.Parallel()
		calls := 0
		_, err := booleaneval.GrowFolded(ctx, fold, meshbool.OpUnion, 1, func(map[int]float64) (booleaneval.Evaluation, error) {
			calls++
			return booleaneval.Evaluation{}, fold
		})
		require.Same(t, fold, err)
		require.Equal(t, 6, calls)
	})
	t.Run("no reach", func(t *testing.T) {
		t.Parallel()
		_, err := booleaneval.GrowFolded(gaptol.With(t.Context(), 0), fold, meshbool.OpUnion, 1, func(map[int]float64) (booleaneval.Evaluation, error) {
			t.Fatal("no attempt runs without a reach")
			return booleaneval.Evaluation{}, nil
		})
		require.Same(t, fold, err)
	})
}
