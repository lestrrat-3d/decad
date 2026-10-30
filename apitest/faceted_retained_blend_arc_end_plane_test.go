package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFacetedRetainedBlendArcEndPlane fillets arcs whose rows end on a plane
// parallel to the axis and offset from it, nearly tangent to the rows'
// circles (agent-docs/retained-edge-blend-design.md §4.4, §6.3). The torus
// meets that plane in a curve that is no conic, and the end face takes the
// chords between the rows' points on it. c is the sine of the angle between
// the plane and the circle of the row nearest tangency. The true end curve is
// sampled from the torus and must lie within the mesh's published Bound.
func TestFacetedRetainedBlendArcEndPlane(t *testing.T) {
	t.Parallel()

	// A Ø8 bore along z with its axis at (h, 10) breaks out through the
	// plate's face x = 0, h from the axis, so the bore's top rim is an arc
	// whose ends lie on x = 0 (the arc route). The fillet's wall row, the
	// circle of radius 4, meets x = 0 at the sine c.
	for _, c := range []float64{0.4, 0.2} {
		t.Run(fmt.Sprintf("the arc route at a hole mouth, c = %g", c), func(t *testing.T) {
			t.Parallel()
			const bore, r = 4.0, 4.0
			h := bore * math.Sqrt(1-c*c)
			sc := newRetentionScene()
			bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, h, 10, bore, -30, 30))
			require.NoError(t, err)
			out, err := decad.Fillet(t.Context(), bored, arcSeedAt(t, bored, 20), units.Millimeters(r))
			require.NoError(t, err)
			for _, side := range []float64{1, -1} {
				requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
					rho := bore + r - r*math.Sin(alpha)
					return r3.NewVec(0, 10+side*math.Sqrt(rho*rho-h*h), 20-r*(1-math.Cos(alpha)))
				})
			}
		})
	}

	// The box's corner rounded to radius 3 about (3, 3), cut back to x = gx,
	// has a top outline that starts with an arc ending on x = gx: a tangent
	// chain with an arc end (the chain route). The fillet's plane row, the
	// circle of radius 3 − r, meets x = gx at the sine c = 0.1.
	t.Run("the chain route at a boss's corner", func(t *testing.T) {
		t.Parallel()
		const r, c = 2.5, 0.1
		h := (openBoxR - r) * math.Sqrt(1-c*c)
		gx := openBoxR - h
		sc := newRetentionScene()
		block := placed(t, rectBox(t, sc.w, sc.doc, -5, -5, gx, 25, 30), -5)
		trimmed, err := decad.Cut(t.Context(), roundedCornerBox(t, sc), block)
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), trimmed, arcSeedAt(t, trimmed, 20), units.Millimeters(r),
			decad.WithTangentChain())
		require.NoError(t, err)
		requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
			rho := openBoxR - r + r*math.Sin(alpha)
			return r3.NewVec(gx, openBoxR-math.Sqrt(rho*rho-h*h), 20-r*(1-math.Cos(alpha)))
		})
	})
}
