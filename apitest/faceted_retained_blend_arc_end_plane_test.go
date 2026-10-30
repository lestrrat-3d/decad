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
	// strip triangle must lie within Bound of the cone.
	for _, c := range []float64{0.4, 0.2} {
		for _, d := range []float64{1, 2, 4} {
			t.Run(fmt.Sprintf("the chamfer's strip at a hole mouth, d = %g, c = %g", d, c), func(t *testing.T) {
				t.Parallel()
				const bore = 4.0
				h := bore * math.Sqrt(1-c*c)
				sc := newRetentionScene()
				bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, h, 10, bore, -30, 30))
				require.NoError(t, err)
				out, err := decad.Chamfer(t.Context(), bored, arcSeedAt(t, bored, 20), units.Millimeters(d))
				require.NoError(t, err)
				// The cone's generator in the meridian plane about (h, 10),
				// from (bore + d, 20) to (bore, 20 − d).
				coneDist := func(p r3.Vec) float64 {
					rho := math.Hypot(p.X-h, p.Y-10)
					a, b := [2]float64{bore + d, 20}, [2]float64{bore, 20 - d}
					ux, uz := b[0]-a[0], b[1]-a[1]
					s := math.Min(1, math.Max(0, ((rho-a[0])*ux+(p.Z-a[1])*uz)/(ux*ux+uz*uz)))
					return math.Hypot(rho-a[0]-s*ux, p.Z-a[1]-s*uz)
				}
				requireStripInteriorsCovered(t, out, coneDist, func(p r3.Vec) bool {
					return math.Abs(p.X) < 1e-9 || math.Abs(p.Z-20) < 1e-9
				})
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

	// A plate's pocket open through x = 0, its two corners at x = x1 rounded
	// to radius 3, so the corner axes lie 3 − x1 from x = 0. The mouth's
	// outline runs arc, line, arc with both arcs ending on x = 0: the chain
	// route with arc ends on the hole side, κ = +1. At x1 = 0.5 the wall's row
	// meets x = 0 at the sine √(1 − (2.5/3)²) ≈ 0.55.
	t.Run("the chain route at a pocket mouth", func(t *testing.T) {
		t.Parallel()
		const r, x1 = 2.0, 0.5
		h := openBoxR - x1
		body := pocketMouth(t, x1)
		out, err := decad.Fillet(t.Context(), body, arcSeedAt(t, body, 10),
			units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		for _, end := range []struct{ y, dir float64 }{{13, -1}, {27, 1}} {
			requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
				rho := openBoxR + r - r*math.Sin(alpha)
				return r3.NewVec(0, end.y+end.dir*math.Sqrt(rho*rho-h*h), 10-r*(1-math.Cos(alpha)))
			})
		}
	})

	// At x1 = 0.25 the rim meets x = 0 at the sine ≈ 0.40, and the walk that
	// expands the seed refuses (modify-reach SX2) before any row is built
	// (agent-docs/missing-features.md).
	t.Run("the chain walk refuses a pocket mouth cut near its corners' axes", func(t *testing.T) {
		t.Parallel()
		body := pocketMouth(t, 0.25)
		_, err := decad.Fillet(t.Context(), body, arcSeedAt(t, body, 10), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the tangent chain branches or cannot be decided here")
		require.ErrorContains(t, err, "(modify-reach SX2)")
	})
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

// requireStripInteriorsCovered samples every strip triangle of out's mesh, a
// triangle whose three vertices lie on the blend surface (dist at most
// 1e-9) and not all on one face's plane (onFace), on a barycentric grid, and
// requires every sample within the mesh's published Bound of the surface.
func requireStripInteriorsCovered(t *testing.T, out *decad.Body, dist func(r3.Vec) float64,
	onFace func(r3.Vec) bool) {
	t.Helper()
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	verts := mesh.Vertices()
	worst, strip := 0.0, 0
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		if dist(a) > 1e-9 || dist(b) > 1e-9 || dist(c) > 1e-9 || (onFace(a) && onFace(b) && onFace(c)) {
			continue
		}
		strip++
		const g = 8
		for i := 0; i <= g; i++ {
			for j := 0; i+j <= g; j++ {
				p := a.Add(b.Sub(a).Scale(float64(i) / g)).Add(c.Sub(a).Scale(float64(j) / g))
				worst = math.Max(worst, dist(p))
			}
		}
	}
	t.Logf("strip interiors within %.6g mm of the surface, Bound %.6g mm, %d strip facets of %d", worst,
		mesh.Bound().Base(), strip, len(mesh.Triangles()))
	require.Positive(t, strip)
	require.LessOrEqual(t, worst, mesh.Bound().Base())
}
