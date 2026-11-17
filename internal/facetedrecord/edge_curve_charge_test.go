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
// cylinder that states a displacement: any finite non-negative one beside a wall
// the stacked view marks, and only one within the chain's arithmetic allowance
// beside an unmarked (published) wall. A negative or non-finite one keeps the
// chord chain either way.
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
		name   string
		set    bool
		marked bool
		delta  float64
		ok     bool
	}{
		{"unset", false, false, 0, true},
		{"within allowance", true, false, 1e-14, true},
		{"a stated displacement beside a published wall", true, false, 1e-6, false},
		{"a stated displacement beside a marked wall", true, true, 1e-6, true},
		{"a marked wall within allowance", true, true, 1e-14, true},
		{"negative", true, true, -1, false},
		{"NaN", true, true, math.NaN(), false},
		{"infinite", true, true, math.Inf(1), false},
		{"infinite beside a published wall", true, false, math.Inf(1), false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			floor := retained(planeAt(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
			wall := retained(surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(3)})
			wall.HasAxialDelta, wall.AxialDelta, wall.ViewCharge = tc.set, tc.delta, tc.marked
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

// TestRetainedEdgeCurvePublishedWall checks that a published charged wall
// declines the line rule too, not the circle rule alone.
func TestRetainedEdgeCurvePublishedWall(t *testing.T) {
	t.Parallel()
	// A plane through the cylinder's tangent line meets it in a Line3.
	line := []r3.Vec{r3.NewVec(3, 0, 0), r3.NewVec(3, 0, 2), r3.NewVec(3, 0, 5)}
	lineBounds := []float64{1e-12, 1e-12, 1e-12}
	for _, tc := range []struct {
		name   string
		marked bool
		ok     bool
	}{{"published", false, false}, {"marked", true, true}} {
		t.Run("a plane-parallel cylinder line, "+tc.name, func(t *testing.T) {
			t.Parallel()
			side := retained(planeAt(t, r3.NewVec(3, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
			wall := retained(surfacegeom.Cylinder{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(3)})
			wall.HasAxialDelta, wall.AxialDelta, wall.ViewCharge = true, 1e-6, tc.marked
			curve, ok := facetedrecord.RetainedEdgeCurve(side, wall, line, lineBounds, false)
			require.Equal(t, tc.ok, ok)
			if tc.ok {
				require.IsType(t, surfacegeom.Line3{}, curve)
			}
		})
	}
}
