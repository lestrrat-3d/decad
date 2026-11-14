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

	// The same hole mouth chamfered: the cone's strip triangles join rows
	// whose ends on x = 0 lie at different azimuths, and every point of every
	// facet of the chamfer, the band beside the wall's cut included, must lie
	// within Bound of the cone.
	for _, c := range []float64{0.4, 0.2} {
		for _, d := range []float64{1, 2, 4} {
			t.Run(fmt.Sprintf("the chamfer's strip at a hole mouth, d = %g, c = %g", d, c), func(t *testing.T) {
				t.Parallel()
				const bore = 4.0
				h := bore * math.Sqrt(1-c*c)
				sc := newRetentionScene()
				bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, h, 10, bore, -30, 30))
				require.NoError(t, err)
				old := originsOf(bored)
				out, err := decad.Chamfer(t.Context(), bored, arcSeedAt(t, bored, 20), units.Millimeters(d))
				require.NoError(t, err)
				made := madeFaces(old, out)
				// The cone's generator in the meridian plane about (h, 10),
				// from (bore + d, 20) to (bore, 20 − d); the chamfer removes
				// the bore's edge at (bore, 20).
				cone := blendProfile{cx: h, cy: 10, seg: true, p0: [2]float64{bore + d, 20},
					p1: [2]float64{bore, 20 - d}, corner: [2]float64{bore, 20}}
				requireBlendFacetsCovered(t, out, func(f *decad.Face) bool { return made[f] }, cone.dist,
					cone.outward, math.Ldexp(d, -10), true)
			})
		}
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

	// A plate's pocket open through x = 0, its two corners at x1 rounded to
	// radius 3, so the corner axes lie 3 − x1 from x = 0. The mouth's outline
	// runs arc, line, arc with both arcs ending on x = 0: the chain route with
	// arc ends on the hole side, κ = +1. At x1 = 0.5 the wall's row meets
	// x = 0 at the sine √(1 − (2.5/3)²) ≈ 0.55; at x1 = 0.25 the sine is
	// ≈ 0.40, and the held chain's short steps near the arcs' ends no longer
	// decline the arc (retention E7), so the walk reads the arc end there too.
	for _, c := range []struct{ r, x1 float64 }{{2.0, 0.5}, {1.0, 0.25}} {
		t.Run(fmt.Sprintf("the chain route at a pocket mouth, x1 = %g", c.x1), func(t *testing.T) {
			t.Parallel()
			h := openBoxR - c.x1
			body := pocketMouth(t, c.x1)
			out, err := decad.Fillet(t.Context(), body, arcSeedAt(t, body, 10),
				units.Millimeters(c.r), decad.WithTangentChain())
			require.NoError(t, err)
			for _, end := range []struct{ y, dir float64 }{{13, -1}, {27, 1}} {
				requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
					rho := openBoxR + c.r - c.r*math.Sin(alpha)
					return r3.NewVec(0, end.y+end.dir*math.Sqrt(rho*rho-h*h), 10-c.r*(1-math.Cos(alpha)))
				})
			}
		})
	}
}

// pocketMouth is a 40×40×10 plate with a placed tool x ∈ [−10, x1],
// y ∈ [10, 30] cut 5 deep, the tool's two corners at x1 rounded to radius 3.
func pocketMouth(t *testing.T, x1 float64) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	plate := rectBox(t, sc.w, sc.doc, 0, 0, 40, 40, 10)
	tool := rectBox(t, sc.w, sc.doc, -10, 10, x1, 30, 20)
	up := decad.ParallelTo(r3.NewVec(0, 0, 1))
	corners := decad.Edges(up, decad.EndpointAt(r3.NewVec(x1, 10, 0))).
		Or(up, decad.EndpointAt(r3.NewVec(x1, 30, 0))).Exactly(2)
	tool, err := decad.Fillet(t.Context(), tool, corners, units.Millimeters(openBoxR))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), plate, placed(t, tool, 5))
	require.NoError(t, err)
	return out
}
