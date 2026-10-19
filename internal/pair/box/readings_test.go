package box_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointMeasurement(t *testing.T) {
	t.Parallel()
	t.Run("dyadic point is exact", func(t *testing.T) {
		t.Parallel()
		got, ok := box.PointMeasurement([3]*big.Rat{big.NewRat(1, 2), big.NewRat(-3, 1), new(big.Rat)})
		require.True(t, ok)
		require.Equal(t, r3.Vec{X: 0.5, Y: -3, Z: 0}, got.Value)
		require.Equal(t, measurement.Exact, got.Exactness)
		require.Zero(t, got.Bound.Base())
	})
	t.Run("rounded point carries an enclosing ball", func(t *testing.T) {
		t.Parallel()
		third := big.NewRat(1, 3)
		got, ok := box.PointMeasurement([3]*big.Rat{third, big.NewRat(2, 1), big.NewRat(-1, 3)})
		require.True(t, ok)
		require.Equal(t, measurement.Approximate, got.Exactness)
		bound := proof.FloatRat(got.Bound.Base())
		for axis, exact := range [3]*big.Rat{third, big.NewRat(2, 1), big.NewRat(-1, 3)} {
			held := proof.FloatRat([3]float64{got.Value.X, got.Value.Y, got.Value.Z}[axis])
			deviation := new(big.Rat).Abs(new(big.Rat).Sub(exact, held))
			require.LessOrEqual(t, deviation.Cmp(bound), 0, "axis %d lies outside the ball", axis)
		}
	})
	t.Run("unrepresentable point is refused", func(t *testing.T) {
		t.Parallel()
		huge := new(big.Rat).SetInt(new(big.Int).Exp(big.NewInt(10), big.NewInt(400), nil))
		_, ok := box.PointMeasurement([3]*big.Rat{huge, new(big.Rat), new(big.Rat)})
		require.False(t, ok)
	})
}

func TestPlanarNormalMeasurement(t *testing.T) {
	t.Parallel()
	dy := proof.MustDyOf
	t.Run("axis direction is exact", func(t *testing.T) {
		t.Parallel()
		got, angle, ok := box.PlanarNormalMeasurement(proof.DyV3{dy(0), dy(0), dy(2)})
		require.True(t, ok)
		require.Equal(t, r3.Vec{Z: 1}, got.Value)
		require.Equal(t, measurement.Exact, got.Exactness)
		require.Zero(t, got.Bound.Base())
		require.Zero(t, angle.Base())
	})
	t.Run("diagonal direction is bounded and reverses exactly", func(t *testing.T) {
		t.Parallel()
		got, angle, ok := box.PlanarNormalMeasurement(proof.DyV3{dy(1), dy(1), dy(0)})
		require.True(t, ok)
		require.Equal(t, measurement.Approximate, got.Exactness)
		require.Positive(t, got.Bound.Base())
		require.Positive(t, angle.Base())
		reversed, reversedAngle, ok := box.PlanarNormalMeasurement(proof.DyV3{dy(-1), dy(-1), dy(0)})
		require.True(t, ok)
		require.Equal(t, r3.Vec{X: -got.Value.X, Y: -got.Value.Y, Z: -got.Value.Z}, reversed.Value)
		require.Equal(t, got.Bound, reversed.Bound)
		require.Equal(t, angle, reversedAngle)
	})
	t.Run("zero direction is refused", func(t *testing.T) {
		t.Parallel()
		_, _, ok := box.PlanarNormalMeasurement(proof.DyV3{dy(0), dy(0), dy(0)})
		require.False(t, ok)
	})
}
