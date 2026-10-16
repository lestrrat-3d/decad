package obliqueboss_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/stretchr/testify/require"
)

// TestDiameterBoundTakesTheLargestWidenedExtent checks that the bound is
// sqrt(3) times the largest of the three widened extents: the plate width, the
// plate depth, and plate height plus boss length and radius, each plus twice
// the boundary bound.
func TestDiameterBoundTakesTheLargestWidenedExtent(t *testing.T) {
	cases := []struct {
		name   string
		rec    facetedrecord.ObliqueBossRecord
		bound  float64
		extent float64
	}{
		{"width", facetedrecord.ObliqueBossRecord{
			Rect: momentinput.AxisRect{U0: 0, V0: 0, U1: 20, V1: 10}, Bottom: 0, Top: 5, Length: 6, Radius: 4},
			0.5, 21},
		{"depth", facetedrecord.ObliqueBossRecord{
			Rect: momentinput.AxisRect{U0: 0, V0: -3, U1: 10, V1: 17}, Bottom: 0, Top: 5, Length: 6, Radius: 4},
			0.25, 20.5},
		{"height", facetedrecord.ObliqueBossRecord{
			Rect: momentinput.AxisRect{U0: 0, V0: 0, U1: 10, V1: 10}, Bottom: 1, Top: 6, Length: 30, Radius: 4},
			0.5, 40},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := obliqueboss.DiameterBound(&tc.rec, tc.bound)
			want := math.Sqrt(3) * tc.extent
			require.GreaterOrEqual(t, got, want)
			require.Less(t, got, want*(1+1e-9))
		})
	}
}
