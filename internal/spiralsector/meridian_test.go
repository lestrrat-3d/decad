package spiralsector

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRationalDirection(t *testing.T) {
	t.Parallel()
	exact := func(name string, sumY, sumZ float64, wantY, wantZ int64) {
		t.Helper()
		y, z, ok := rationalDirection(sumY, sumZ)
		require.True(t, ok, name)
		require.Zero(t, y.Cmp(big.NewRat(wantY, 1)), "%s: y = %s", name, y)
		require.Zero(t, z.Cmp(big.NewRat(wantZ, 1)), "%s: z = %s", name, z)
		norm := new(big.Rat).Add(new(big.Rat).Mul(y, y), new(big.Rat).Mul(z, z))
		require.Zero(t, norm.Cmp(big.NewRat(1, 1)), "%s: y^2+z^2 = %s", name, norm)
	}
	exact("+y", 1, 0, 1, 0)
	exact("+z", 0, 1, 0, 1)
	exact("-y", -1, 0, -1, 0)

	_, _, ok := rationalDirection(math.NaN(), 0)
	require.False(t, ok)
}

func TestTriangleMeridianError(t *testing.T) {
	t.Parallel()
	m, b := new(big.Rat), big.NewRat(1, 1)
	limitOf := func(a, b2, c r3.Vec) (float64, bool) {
		limit, ok := TriangleMeridianError([3]r3.Vec{a, b2, c}, m, b)
		if !ok {
			return 0, false
		}
		f, _ := limit.Float64()
		return f, true
	}

	t.Run("radius above the meridian", func(t *testing.T) {
		limit, ok := limitOf(r3.Vec{X: 0, Y: 2}, r3.Vec{X: 1, Y: 2}, r3.Vec{X: 0.5, Y: 2})
		require.True(t, ok)
		require.GreaterOrEqual(t, limit, 1.0)
		require.LessOrEqual(t, limit, 1+1e-12)
	})
	t.Run("radius just below the meridian", func(t *testing.T) {
		limit, ok := limitOf(r3.Vec{X: 0, Y: 1}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: 0.5, Y: 0.999, Z: 0.04})
		require.True(t, ok)
		require.Greater(t, limit, 0.0002)
		require.Less(t, limit, 0.001)
	})
	t.Run("angular spread of pi", func(t *testing.T) {
		_, ok := limitOf(r3.Vec{Y: 1}, r3.Vec{Y: -1, Z: 0.1}, r3.Vec{Y: -1, Z: -0.1})
		require.False(t, ok)
	})
}
