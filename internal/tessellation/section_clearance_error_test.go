package tessellation_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

func TestRequireWalkClearanceNamesTheChords(t *testing.T) {
	// A flattened rectangle whose two long sides are 0.3 apart: a 0.2 tube on
	// each long side makes chords 0 and 2 collide.
	pts := []tessellation.SectionPoint{{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 0.3}, {U: 0, V: 0.3}}
	loops := [][]int{{0, 1, 2, 3}}

	require.NoError(t, tessellation.RequireWalkClearance(t.Context(), pts, loops, [][]float64{{0.1, 0, 0.1, 0}}))

	err := tessellation.RequireWalkClearance(t.Context(), pts, loops, [][]float64{{0.2, 0, 0.2, 0}})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "chords 0 and 2 of section loop 0")
	require.ErrorContains(t, err, "finer chord tolerance")

	var named *tessellation.SectionClearanceError
	require.True(t, errors.As(err, &named))
	require.Equal(t, 0, named.Loop)
	require.Equal(t, 0, named.A)
	require.Equal(t, 2, named.B)

	var expected *tessellation.ExpectedError
	require.ErrorAs(t, err, &expected)
}

func TestRequireLoopClearanceNamesTheLoops(t *testing.T) {
	pts := []tessellation.SectionPoint{
		{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}, {U: 0, V: 1},
		{U: 1.25, V: 0}, {U: 2.25, V: 0}, {U: 2.25, V: 1}, {U: 1.25, V: 1},
	}
	loops := [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}}

	err := tessellation.RequireLoopClearance(t.Context(), pts, loops, []float64{0.2, 0.2})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "cap boundary loops 0 and 1")
	require.ErrorContains(t, err, "retry with a finer chord tolerance")
	var expected *tessellation.ExpectedError
	require.ErrorAs(t, err, &expected)

	require.NoError(t, tessellation.RequireLoopClearance(t.Context(), pts, loops, []float64{0.1, 0.1}))
}
