package survey2d_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestWidenPullBracketCoversTurnedNormals widens the bracket of the
// component of the in-plane normal (1, 0) along a pull, then turns that
// normal by sampled angles up to the widening's turn (unit-vector distance):
// every turned component stays inside the widened bracket, and a pull along
// F's normal widens nothing.
func TestWidenPullBracketCoversTurnedNormals(t *testing.T) {
	t.Parallel()
	m := identityFrameMap(t)
	pull := r3.Vec{X: 3, Y: -4, Z: 2}
	base := new(big.Rat).SetFloat64(3) // (1, 0)·pull
	b := survey2d.PullBracket{MinLo: base, MinHi: base, MaxLo: base, MaxHi: base}
	turn := big.NewRat(1, 4)
	got, ok := survey2d.WidenPullBracket(b, m, pull, turn)
	require.True(t, ok)
	lo, _ := got.MinLo.Float64()
	hi, _ := got.MaxHi.Float64()
	for i := -100; i <= 100; i++ {
		// A unit vector a chord 1/4 from (1, 0) turns by 2·asin(1/8).
		a := 2 * math.Asin(0.125) * float64(i) / 100
		c := 3*math.Cos(a) - 4*math.Sin(a)
		require.GreaterOrEqual(t, c, lo-1e-12)
		require.LessOrEqual(t, c, hi+1e-12)
	}
	axial, ok := survey2d.WidenPullBracket(b, m, r3.Vec{Z: 1}, turn)
	require.True(t, ok)
	require.Zero(t, axial.MinLo.Cmp(base))
	require.Zero(t, axial.MaxHi.Cmp(base))
}
