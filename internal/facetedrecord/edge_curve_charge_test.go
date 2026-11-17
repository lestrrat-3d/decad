package facetedrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestRetainedEdgeCurveCylinderAxialDelta admits the circle of a plane and a
// cylinder that states a finite non-negative displacement, which only the
// retained edge blend reads; a negative or non-finite one keeps the chord chain.
func TestRetainedEdgeCurveCylinderAxialDelta(t *testing.T) {
	t.Parallel()
	const n = 32
	var chain []r3.Vec
	var bounds []float64
	for i := 0; i <= n; i++ {
		a := 2 * math.Pi * float64(i%n) / n
		chain = append(chain, r3.NewVec(3*math.Cos(a), 3*math.Sin(a), 0))
		bounds = append(bounds, 1e-12)
	}
	cases := []struct {
		name  string
		set   bool
		delta float64
		ok    bool
	}{
		{"unset", false, 0, true},
		{"within allowance", true, 1e-14, true},
		{"a stated displacement", true, 1e-6, true},
		{"negative", true, -1, false},
		{"NaN", true, math.NaN(), false},
		{"infinite", true, math.Inf(1), false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
			wall := retained(surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(3)})
			wall.HasAxialDelta, wall.AxialDelta = tc.set, tc.delta
			curve, ok := facetedrecord.RetainedEdgeCurve(floor, wall, chain, bounds, true)
			require.Equal(t, tc.ok, ok)
			if !tc.ok {
				require.Nil(t, curve)
				return
			}
			require.IsType(t, surfacegeom.Circle3{}, curve)
		})
	}
}
