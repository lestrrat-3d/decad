package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// xzBox extrudes the rectangle x ∈ [−3.5, 3.5], z ∈ [z0, z1], sketched on the
// plane y = y0, 3.6 along +y.
func xzBox(t *testing.T, y0, z0, z1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	f, err := r3.NewFrame(r3.Vec{Y: y0}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	r := s.CreateRectangle(-3.5, z0, 3.5, z1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), decad.New(), s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(3.6), Dir: decad.Against})
	require.NoError(t, err)
	return box
}

// TestBrepRouteEFilletEndFacesAgree fillets each edge along z of a box
// extruded along y, one at a time, through route E. The edge's two end faces,
// the box's floor and top, hold its corner through mirrored frames, and each
// face's walk along the cap runs to a different far end. Both read the corner
// with the same walk arriving, through the corner itself, so their feet agree
// bit for bit and SB9 passes (agent-docs/modify-design.md Part B §5.3 step 2).
// Read through the far ends in each face's own loop order, the feet on the
// edges at the sketch plane y = y0 part by an ulp and SB9 refused them.
func TestBrepRouteEFilletEndFacesAgree(t *testing.T) {
	t.Parallel()
	const r = 0.3
	for _, box := range []struct{ y0, z0, z1 float64 }{{0, 0, 10}, {9.2, 42.5, 60.5}} {
		for _, x := range []float64{-3.5, 3.5} {
			for _, y := range []float64{box.y0, box.y0 + 3.6} {
				body := xzBox(t, box.y0, box.z0, box.z1)
				out, err := decad.Fillet(t.Context(), body, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
					decad.EndpointAt(r3.NewVec(x, y, box.z0))).Exactly(1), units.Millimeters(r))
				require.NoError(t, err, "edge at x = %g, y = %g", x, y)

				// The box loses the corner region over the edge's length.
				want := 7*(box.z1-box.z0)*3.6 - (box.z1-box.z0)*r*r*(1-math.Pi/4)
				vol, err := out.Volume()
				require.NoError(t, err)
				require.LessOrEqual(t, math.Abs(vol.Value.Base()-want), vol.Bound.Base()+1e-9)

				// The blend is a cylinder about the line r in from both faces.
				cx := x - math.Copysign(r, x)
				cy := y + r
				if y != box.y0 {
					cy = y - r
				}
				cylinders := 0
				for _, f := range out.Faces() {
					c, ok := f.Surface().(decad.Cylinder)
					if !ok {
						continue
					}
					cylinders++
					require.InDelta(t, r, c.Radius.Base(), 1e-15)
					require.InDelta(t, 1, math.Abs(c.Axis.Z), 1e-15)
					require.InDelta(t, cx, c.Origin.X, 1e-12)
					require.InDelta(t, cy, c.Origin.Y, 1e-12)
				}
				require.Equal(t, 1, cylinders)
				mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
			}
		}
	}
}

// xyPrism extrudes the polygon pts, sketched on the plane z = z0, along +z by
// height.
func xyPrism(t *testing.T, doc *decad.Document, z0, height float64, pts [][2]float64) *decad.Body {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return framePrism(t, doc, f, height, decad.Along, pts)
}

// framePrism extrudes the polygon pts, sketched with fixed corners on the
// plane f, by depth in direction dir.
func framePrism(t *testing.T, doc *decad.Document, f r3.Frame, depth float64, dir decad.Direction,
	pts [][2]float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(depth), Dir: dir})
	require.NoError(t, err)
	return body
}

// TestBrepRouteEFilletObliqueBossEdges fillets, one at a time, the vertical
// edges of trapezoid bosses united with a plate. Each edge lies between two
// walls, one or both oblique, so its carriers are not axis-aligned, and its
// end faces, the boss's top and the plate's top, hold its corner in opposite
// senses. Both read the corner with the same walk arriving; read in each
// face's own loop order instead, some of these edges refuse SB9.
func TestBrepRouteEFilletObliqueBossEdges(t *testing.T) {
	t.Parallel()
	const r = 0.3
	for _, lean := range []float64{0.7, 1.3, 2.9, 3.1} {
		for _, top := range []float64{4.9, 6.2, 7.3} {
			boss := [][2]float64{{-5, -5}, {5, -5}, {5 - lean, top}, {-5 + 1.7*lean, top}}
			for _, corner := range boss {
				doc := decad.New()
				plate := xyPrism(t, doc, 0, 10, [][2]float64{{-20, -20}, {20, -20}, {20, 20}, {-20, 20}})
				body, err := decad.Union(plate, xyPrism(t, doc, 10, 15, boss)).Do(t.Context())
				require.NoError(t, err)
				before, err := body.Volume()
				require.NoError(t, err)
				out, err := decad.Fillet(t.Context(), body, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
					decad.EndpointAt(r3.NewVec(corner[0], corner[1], 25))).Exactly(1), units.Millimeters(r))
				require.NoError(t, err, "lean %g, top %g, corner %v", lean, top, corner)
				after, err := out.Volume()
				require.NoError(t, err)
				// A convex corner loses material along the 15 mm edge.
				require.Less(t, after.Value.Base(), before.Value.Base())
				require.Less(t, before.Value.Base()-after.Value.Base(), 15*r*r+before.Bound.Base()+after.Bound.Base())
			}
		}
	}
}
