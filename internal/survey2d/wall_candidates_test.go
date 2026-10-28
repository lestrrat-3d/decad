package survey2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// TestArcArcCandsSkipsCarrierPair pins the carrier-pair rule: two halves of
// one circle that share a nonzero Carrier and a material side emit no
// concentric candidate, while every other concentric pair keeps its family.
func TestArcArcCandsSkipsCarrierPair(t *testing.T) {
	arc := func(r, th0, th1 float64, carrier int) survey2d.SurveyElem {
		t.Helper()
		e, ok := survey2d.ArcElem(0, 0, r, th0, th1, false)
		require.True(t, ok)
		e.Carrier = carrier
		return e
	}
	radii := func(a, b survey2d.SurveyElem) []float64 {
		t.Helper()
		kernel := survey2d.NewWallKernel([]survey2d.SurveyElem{a, b}, nil, math.Inf(1))
		var got []float64
		kernel.PairCands(a, b, func(_, _, r, _ float64) { got = append(got, r) })
		return got
	}
	upper, lower := arc(8, 0, math.Pi, 1), arc(8, -math.Pi, 0, 1)
	require.True(t, upper.MatInside)
	require.True(t, lower.MatInside)

	t.Run("same carrier emits no concentric candidate", func(t *testing.T) {
		require.Empty(t, radii(upper, lower))
	})
	t.Run("no carrier keeps the zero-width family", func(t *testing.T) {
		upper, lower := upper, lower
		upper.Carrier, lower.Carrier = 0, 0
		require.Equal(t, []float64{0, 0}, radii(upper, lower))
	})
	t.Run("distinct carriers keep the zero-width family", func(t *testing.T) {
		other := lower
		other.Carrier = 2
		require.Equal(t, []float64{0, 0}, radii(upper, other))
	})
	t.Run("opposite material sides keep the zero-width family", func(t *testing.T) {
		hole := arc(8, 0, -math.Pi, 1)
		require.False(t, hole.MatInside)
		require.Equal(t, []float64{0, 0}, radii(upper, hole))
	})
	t.Run("concentric skins of two carriers read the shell", func(t *testing.T) {
		// A hollow sphere's inner (r 8, material outside) and outer (r 10)
		// skins: their family is the 1 mm-radius disk between them.
		inner := arc(8, math.Pi, 0, 1)
		outer := arc(10, 0, math.Pi, 2)
		require.Equal(t, []float64{1, 1}, radii(inner, outer))
	})
}
