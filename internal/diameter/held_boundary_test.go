package diameter_test

import (
	"context"
	"iter"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestHeldPoints(t *testing.T) {
	t.Parallel()
	origin := diameter.HeldPoint{Value: r3.Vec{}, Bound: 0.01}
	far := diameter.HeldPoint{Value: r3.Vec{X: 3, Y: 4}, Bound: 0.02}

	t.Run("shrinks the pair distance by twice the widest bound and allowance", func(t *testing.T) {
		t.Parallel()
		d, ok, err := diameter.HeldPoints(t.Context(), slices.Values([]diameter.HeldPoint{origin, far}), 0.03)
		require.NoError(t, err)
		require.True(t, ok)
		require.LessOrEqual(t, d, 5-2*(0.02+0.03))
		require.InDelta(t, 4.9, d, 1e-9)
	})
	t.Run("withholds on a non-finite point", func(t *testing.T) {
		t.Parallel()
		bad := diameter.HeldPoint{Value: r3.Vec{X: math.NaN()}, Bound: 0}
		_, ok, err := diameter.HeldPoints(t.Context(), slices.Values([]diameter.HeldPoint{origin, bad}), 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("withholds on an unusable bound", func(t *testing.T) {
		t.Parallel()
		bad := diameter.HeldPoint{Value: r3.Vec{X: 1}, Bound: math.Inf(1)}
		_, ok, err := diameter.HeldPoints(t.Context(), slices.Values([]diameter.HeldPoint{origin, bad}), 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("withholds when the shrink reaches zero", func(t *testing.T) {
		t.Parallel()
		_, ok, err := diameter.HeldPoints(t.Context(), slices.Values([]diameter.HeldPoint{origin, far}), 3)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("returns a cancelled context", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, ok, err := diameter.HeldPoints(ctx, slices.Values([]diameter.HeldPoint{origin, far}), 0)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
	})
}

func TestCircularArcs(t *testing.T) {
	t.Parallel()
	seq := func(lengths ...diameter.ArcLength) iter.Seq[diameter.ArcLength] { return slices.Values(lengths) }

	t.Run("full circle reads its diameter from the circumference", func(t *testing.T) {
		t.Parallel()
		d, ok, err := diameter.CircularArcs(t.Context(), seq(diameter.ArcLength{Value: 2 * math.Pi * 5}), 0)
		require.NoError(t, err)
		require.True(t, ok)
		require.LessOrEqual(t, d, 10.0)
		require.InDelta(t, 10, d, 1e-9)
	})
	t.Run("length bound and displacement shrink the reading", func(t *testing.T) {
		t.Parallel()
		d, ok, err := diameter.CircularArcs(t.Context(), seq(diameter.ArcLength{Value: 2 * math.Pi * 5, Bound: math.Pi}), 0.5)
		require.NoError(t, err)
		require.True(t, ok)
		require.InDelta(t, 9-2*0.5, d, 1e-9)
	})
	t.Run("skips an edge whose bound swallows its length", func(t *testing.T) {
		t.Parallel()
		d, ok, err := diameter.CircularArcs(t.Context(),
			seq(diameter.ArcLength{Value: 1, Bound: 2}, diameter.ArcLength{Value: 2 * math.Pi}), 0)
		require.NoError(t, err)
		require.True(t, ok)
		require.InDelta(t, 2, d, 1e-9)
	})
	t.Run("an unbounded length withholds the reference", func(t *testing.T) {
		t.Parallel()
		_, ok, err := diameter.CircularArcs(t.Context(),
			seq(diameter.ArcLength{Unbounded: true}, diameter.ArcLength{Value: 2 * math.Pi}), 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("no edges withholds the reference", func(t *testing.T) {
		t.Parallel()
		_, ok, err := diameter.CircularArcs(t.Context(), seq(), 0)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("returns a cancelled context", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, _, err := diameter.CircularArcs(ctx, seq(diameter.ArcLength{Value: 2 * math.Pi}), 0)
		require.ErrorIs(t, err, context.Canceled)
	})
}
