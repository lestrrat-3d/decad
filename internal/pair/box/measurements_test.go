package box_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestGapMeasurement(t *testing.T) {
	t.Parallel()

	gap := func(a, b, c float64) [3]proofarith.Dyadic {
		return [3]proofarith.Dyadic{proofarith.MustDyOf(a), proofarith.MustDyOf(b), proofarith.MustDyOf(c)}
	}
	// One separated axis is the plain gap along it, exactly.
	got, ok := box.GapMeasurement(gap(-1, 3, 0))
	require.True(t, ok)
	require.Equal(t, measurement.Exact, got.Exactness)
	require.InDelta(t, 3, got.Value.Base(), 0)
	require.Zero(t, got.Bound.Base())
	// Two separated axes give their root-sum-square, enclosed.
	got, ok = box.GapMeasurement(gap(3, 4, -1))
	require.True(t, ok)
	require.InDelta(t, 5, got.Value.Base(), 1e-9)
	require.GreaterOrEqual(t, got.Bound.Base(), 0.0)
	// No separated axis has no gap to publish.
	_, ok = box.GapMeasurement(gap(-1, -2, 0))
	require.False(t, ok)
}

func TestDyadicPointMeasurement(t *testing.T) {
	t.Parallel()

	point := proofarith.DyVec(r3.Vec{X: 1, Y: -2, Z: 0.5})
	got, ok := box.DyadicPointMeasurement(&point)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 1, Y: -2, Z: 0.5}, got.Value)
	require.Equal(t, measurement.Exact, got.Exactness)

	huge := proofarith.DyVec(r3.Vec{X: math.MaxFloat64})
	huge[0] = proofarith.DyMul(huge[0], proofarith.MustDyOf(4))
	_, ok = box.DyadicPointMeasurement(&huge)
	require.False(t, ok, "a coordinate beyond the largest float has no reading")
}

func TestSignedMeasurement(t *testing.T) {
	t.Parallel()

	got, ok := box.SignedMeasurement(proofarith.MustDyOf(-4.25))
	require.True(t, ok)
	require.InDelta(t, -4.25, got.Value.Base(), 0)
	require.Equal(t, measurement.Exact, got.Exactness)

	huge := proofarith.DyMul(proofarith.MustDyOf(math.MaxFloat64), proofarith.MustDyOf(4))
	_, ok = box.SignedMeasurement(huge)
	require.False(t, ok)
}

func TestDyadicNormalMeasurement(t *testing.T) {
	t.Parallel()

	normal, angle, ok := box.DyadicNormalMeasurement(proofarith.DyVec(r3.Vec{Z: 3}))
	require.True(t, ok)
	require.Equal(t, r3.Vec{Z: 1}, normal.Value)
	require.GreaterOrEqual(t, angle.Base(), 0.0)

	// A diagonal axis cannot be normalized exactly, so its angle error is positive.
	diagonal, angle, ok := box.DyadicNormalMeasurement(proofarith.DyVec(r3.Vec{X: 1, Y: 1, Z: 1}))
	require.True(t, ok)
	require.InDelta(t, 1/math.Sqrt(3), diagonal.Value.X, 1e-15)
	require.Positive(t, angle.Base())

	_, _, ok = box.DyadicNormalMeasurement(proofarith.DyVec(r3.Vec{}))
	require.False(t, ok, "a zero axis has no direction")
}
