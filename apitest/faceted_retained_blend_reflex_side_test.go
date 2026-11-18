package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// steppedArm extrudes the outline, given in the XZ plane, 6 mm along y from
// y = 0 and joins it to a block along z that crosses its top, so the Union
// takes the mesh path. The outline's vertex (10, 10) is reflex: there the
// right side x = 10 turns onto the overhang's underside.
func steppedArm(t *testing.T, outline [][2]float64) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XZ())
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(outline))
	for i, c := range outline {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	arm, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(6)})
	require.NoError(t, err)
	box, err := arm.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 0, math.Min(math.Abs(box.Min.Y), math.Abs(box.Max.Y)), 1e-12)
	if box.Min.Y < -1e-9 {
		arm = translated(t, arm, 0, 6, 0)
	}
	cs, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	rect := cs.CreateRectangle(-3, -3, 4, 9)
	cs.Fix(rect.A)
	_, err = cs.Solve(t.Context())
	require.NoError(t, err)
	block, err := decad.Extrude(t.Context(), sc.doc, cs, cs.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), arm, translated(t, block, 0, 0, 12))
	require.NoError(t, err)
	return joined
}

// TestFacetedRetainedBlendReflexSide fillets a convex edge whose face beside
// it is reflex at one end: the edge x = 10, y = 0 rises from the floor z = 0
// to the vertex (10, 0, 10), where the face y = 0 turns onto an overhang. The
// blend ends on the overhang's underside, which takes the blend's end cap,
// and the face y = 0 runs on from the setback point along its meeting with the
// underside (agent-docs/modify-design.md Part E §5.0, "A face that is
// reflex at an end").
func TestFacetedRetainedBlendReflexSide(t *testing.T) {
	t.Parallel()
	const r = 1.0
	area := r * r * (1 - math.Pi/4)
	// The corner region's centroid lies cornerCentroid(r) from x = 10.
	xBar := 10 - cornerCentroid(r)

	for _, tc := range []struct {
		name    string
		outline [][2]float64
		// removed is the volume the blend takes: the corner region times its
		// height up to the underside, at the region's centroid where the
		// underside slopes.
		removed float64
		// grown is the area change: the face y = 0 loses the quadrilateral
		// V0, V1, P_A1, P_A0 and the face x = 10 a 1 × 10 strip; the floor
		// loses the corner region and the underside gains it over |d·n|; the
		// cylinder adds r times the integral of its generators' lengths.
		grown float64
		// setback is P_A1, where the face y = 0 now meets the underside.
		setback r3.Vec
		// capArea is the area of the end cap the underside takes.
		capArea float64
	}{
		{"a flat underside", [][2]float64{{0, 0}, {10, 0}, {10, 10}, {20, 10}, {20, 14}, {0, 14}}, area * 10,
			-20 + 10*math.Pi/2, r3.NewVec(9, 0, 10), area},
		{"an underside at 45 degrees", [][2]float64{{0, 0}, {10, 0}, {10, 10}, {20, 20}, {20, 24}, {16, 24},
			{2, 10}, {0, 10}}, area * xBar, -9.5 - 10 + (math.Sqrt2-1)*area + 9*math.Pi/2 + 1, r3.NewVec(9, 0, 9),
			math.Sqrt2 * area},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := steppedArm(t, tc.outline)
			before, err := body.Volume()
			require.NoError(t, err)
			out, err := decad.Fillet(t.Context(), body, straightEdge(t, body, r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1)),
				units.Millimeters(r))
			require.NoError(t, err)
			after, err := out.Volume()
			require.NoError(t, err)
			requireVolumeChange(t, before, after, tc.removed)
			requireCapsTaken(t, body, out, tc.grown, tc.capArea)
			requireCapCharged(t, body, out, tc.capArea)

			cylinders := 0
			for _, f := range out.Faces() {
				c, ok := f.Surface().(decad.Cylinder)
				if !ok || math.Abs(c.Radius.Base()-r) > 1e-12 {
					continue
				}
				cylinders++
				require.InDelta(t, 10-r, c.Origin.X, 1e-9)
				require.InDelta(t, r, c.Origin.Y, 1e-9)
				require.InDelta(t, 1, math.Abs(c.Axis.Z), 1e-12)
				// The blend removes material: its normal points away from the axis.
				p := firstVertex(t, f)
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				require.Negative(t, n.Value.Dot(c.Origin.Sub(p)))
			}
			require.Equal(t, 1, cylinders)

			// The old corner (10, 0, 10) now lies inside the face y = 0's
			// straight meeting with the underside, which starts at P_A1.
			corners := 0
			for _, v := range out.Vertices() {
				p := v.Position().Value
				if p.Sub(r3.NewVec(10, 0, 10)).Len() < 1e-9 {
					corners++
				}
				if p.Sub(tc.setback).Len() < 1e-9 {
					corners += 2
				}
			}
			require.Equal(t, 2, corners)

			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}

	// The end curve on the 45° underside x − z = 0 is the ellipse the blend's
	// cylinder cuts there: at the cross-section point (10 − r + r·cos α,
	// r − r·sin α) it lies at z = x.
	t.Run("the end curve on the sloped underside", func(t *testing.T) {
		t.Parallel()
		body := steppedArm(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {20, 20}, {20, 24}, {16, 24}, {2, 10}, {0, 10}})
		out, err := decad.Fillet(t.Context(), body, straightEdge(t, body, r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(r))
		require.NoError(t, err)
		requireEndCurveCovered(t, out, func(alpha float64) r3.Vec {
			x := 10 - r + r*math.Cos(alpha)
			return r3.NewVec(x, r-r*math.Sin(alpha), x)
		})
	})

	// A cavity straddles the flat underside z = 10 beside the corner. Its
	// corner nearest the blend's axis (9, 1) lies 1.13 mm from it, outside the
	// r = 1 blend, so it meets no strip facet; its lower half lies in the
	// corner the fillet removes, and it crosses the cap the underside takes.
	t.Run("a cavity crossing the cap", func(t *testing.T) {
		t.Parallel()
		body := withCavity(t, steppedArm(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {20, 10}, {20, 14}, {0, 14}}),
			9.8, 0.05, 9.95, 0.2, 9, 11)
		_, err := decad.Fillet(t.Context(), body, straightEdge(t, body, r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(r))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the cap an end face takes meets another facet of the body at")
	})

	// An underside that rises steeply from the corner, z = 10 + k·(x − 10),
	// meets the setback line on the face y = 0, x = 9, at z = 10 − k. At k = 10
	// that is the floor, where the strip's first row has no length; at k = 12 it
	// lies below the floor and the row runs backwards. Both refuse.
	for _, k := range []float64{10, 12} {
		t.Run(fmt.Sprintf("an underside of slope %g refuses", k), func(t *testing.T) {
			t.Parallel()
			body := steppedArm(t, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {11, 10 + k}, {11, 12 + k}, {0, 12 + k}})
			_, err := decad.Fillet(t.Context(), body, straightEdge(t, body, r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1)),
				units.Millimeters(r))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "a row of the blend's strip is no longer than its bounds along the edge")
		})
	}
}

// requireCapCharged checks that the area bound grew by less than the cap an
// end face takes. A cap charged as zero, neither gained nor given up, puts the
// exact area change one cap off the held one, and the bound would have to
// grow past it.
func requireCapCharged(t *testing.T, before, after *decad.Body, capArea float64) {
	t.Helper()
	beforeArea, err := before.Area()
	require.NoError(t, err)
	afterArea, err := after.Area()
	require.NoError(t, err)
	require.Less(t, afterArea.Bound.Mag()-beforeArea.Bound.Mag(), capArea)
}
