package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The placement entry points a test places a body through.
const (
	placeViaTransform = "Transform"
	placeViaCopy      = "Copy"
)

// curvedLoftRadiusFace returns the Cylinder-tagged face of radius 8 mm.
func curvedLoftRadiusFace(t *testing.T, body *Body) *Face {
	t.Helper()
	var found *Face
	for _, f := range body.Faces() {
		c, ok := f.n.Surface.(surfacegeom.Cylinder)
		if !ok {
			continue
		}
		if r, _ := c.Radius.In(units.Millimeter); math.Abs(r-8) < 1e-9 {
			found = f
		}
	}
	require.NotNil(t, found)
	return found
}

// TestFacetedPlacementMovesCurvedTags places a smooth LoftSections body whose
// Cylinder tag carries no denoted patch. The moved tag must be the placed
// source surface exactly, and the published NormalAt bound must cover the true
// rotated normal for a rotation and for a pure translation.
func TestFacetedPlacementMovesCurvedTags(t *testing.T) {
	body := curvedLoftBody(t)
	src := curvedLoftRadiusFace(t, body)
	srcCyl := src.n.Surface.(surfacegeom.Cylinder)
	p0 := r3.NewVec(8, 0, 5)
	n0, err := src.NormalAt(p0)
	require.NoError(t, err)

	rotY, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(0, 5, 0))
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		move r3.Transform
	}{{"rotate30Y", rotY}, {"translateY5", trans}} {
		for _, mode := range []string{placeViaTransform, placeViaCopy} {
			t.Run(tc.name+"/"+mode, func(t *testing.T) {
				var placed *Body
				if mode == placeViaTransform {
					// Transform consumes its input; copy first so the source stays live.
					cp, cerr := Copy(t.Context(), body)
					require.NoError(t, cerr)
					placed, err = Transform(t.Context(), cp, tc.move)
				} else {
					placed, err = Copy(t.Context(), body, WithCopyPlacement(tc.move))
				}
				require.NoError(t, err)
				mv := curvedLoftRadiusFace(t, placed)
				require.IsType(t, Cylinder{}, mv.Surface())
				cyl := mv.n.Surface.(surfacegeom.Cylinder)
				require.Equal(t, tc.move.Apply(srcCyl.Origin), cyl.Origin)
				require.Equal(t, tc.move.ApplyDir(srcCyl.Axis), cyl.Axis)

				// The cylinder is rigidly moved, so its moved normal is the rotated source normal.
				want := tc.move.ApplyDir(n0.Value)
				got, err := mv.NormalAt(tc.move.Apply(p0))
				require.NoError(t, err)
				diff := got.Value.Sub(want).Len()
				// A moved tag is a float-image stand-in, charged the largest distance
				// of two unit normals (2), the limit of any enclosure of its departure.
				require.GreaterOrEqual(t, got.Bound.Base(), 2.0)
				require.LessOrEqual(t, diff, got.Bound.Base(),
					"published normal bound does not cover the true error")
			})
		}
	}
}

// placeBody places body by move through Transform on a copy and through Copy
// with a placement, so both entry points are exercised.
func placeBody(t *testing.T, body *Body, mode string, move r3.Transform) *Body {
	t.Helper()
	if mode == placeViaTransform {
		// Transform consumes its input; copy first so the source stays live.
		cp, err := Copy(t.Context(), body)
		require.NoError(t, err)
		placed, err := Transform(t.Context(), cp, move)
		require.NoError(t, err)
		return placed
	}
	placed, err := Copy(t.Context(), body, WithCopyPlacement(move))
	require.NoError(t, err)
	return placed
}

// TestFacetedPlacementReflectsPlaneTag reflects a smooth LoftSections body
// whose top cap carries a Plane tag. The moved tag's frame normal is negated by
// the reflection, so the group's orientation flips and the placed face's
// NormalAt direction is the reflected outward normal, covered by its bound.
func TestFacetedPlacementReflectsPlaneTag(t *testing.T) {
	body := curvedLoftBody(t)
	var top *Face
	for _, f := range body.Faces() {
		if pl, ok := f.n.Surface.(surfacegeom.Plane); ok && math.Abs(pl.Frame.Origin().Z-10) < 1e-9 {
			top = f
		}
	}
	require.NotNil(t, top)
	// A source point on the top cap: a body vertex at the cap's height.
	var p0 r3.Vec
	for _, v := range body.Vertices() {
		if pos := v.Position().Value; math.Abs(pos.Z-10) < 1e-9 {
			p0 = pos
		}
	}
	require.InDelta(t, 10, p0.Z, 1e-9)
	n0, err := top.NormalAt(p0)
	require.NoError(t, err)
	require.Greater(t, n0.Value.Z, 0.5, "the source top cap faces up")

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirror)
	require.NoError(t, err)
	require.True(t, reflection.IsReflection())
	for _, mode := range []string{placeViaTransform, placeViaCopy} {
		t.Run(mode, func(t *testing.T) {
			placed := placeBody(t, body, mode, reflection)
			var mv *Face
			for _, f := range placed.Faces() {
				// The mirror plane z = 20 sends the cap at z = 10 to z = 30.
				if pl, ok := f.n.Surface.(surfacegeom.Plane); ok && math.Abs(pl.Frame.Origin().Z-30) < 1e-9 {
					mv = f
				}
			}
			require.NotNil(t, mv, "the reflected top cap")

			// The image of a solid's outward normal under a reflection is the
			// linear part applied to it.
			want := reflection.ApplyDir(n0.Value)
			got, err := mv.NormalAt(reflection.Apply(p0))
			require.NoError(t, err)
			require.LessOrEqual(t, got.Value.Sub(want).Len(), 1e-9, "the reflected normal points the wrong way")
			require.GreaterOrEqual(t, got.Bound.Base(), 2.0)
			require.LessOrEqual(t, got.Value.Sub(want).Len(), got.Bound.Base())
		})
	}
}

// placementConeCut cuts a fitted-spline tooth with a point-apex cone tool. The
// result's Cone face is a faceted group that keeps its denoted surface record.
func placementConeCut(t *testing.T) *Body {
	t.Helper()
	doc := New()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), 8)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(3, -1), s.CreatePoint(4, -1.5), s.CreatePoint(5, -1),
		s.CreatePoint(5, 1), s.CreatePoint(3, 1),
	}
	for _, p := range pts {
		s.Fix(p)
	}
	_, err = s.CreateFitSpline(pts[0], pts[1], pts[2])
	require.NoError(t, err)
	for i := 2; i < len(pts); i++ {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)

	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.XY())
	require.NoError(t, err)
	tp := []*sketch.Point{ts.CreatePoint(-2, 0), ts.CreatePoint(-2, 7), ts.CreatePoint(5, 0)}
	for _, p := range tp {
		ts.Fix(p)
	}
	for i := range tp {
		ts.CreateLine(tp[i], tp[(i+1)%len(tp)])
	}
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Revolve(t.Context(), doc, ts, ts.Profiles()[0],
		SketchLine{Start: Point2{}, End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	cut, err := Cut(t.Context(), tooth, tool)
	require.NoError(t, err)
	return cut
}

func placementConeFace(body *Body) *Face {
	for _, f := range body.Faces() {
		if _, ok := f.n.Surface.(surfacegeom.Cone); ok {
			return f
		}
	}
	return nil
}

// TestFacetedPlacementKeepsDenotedProof places a faceted body whose Cone
// group carries a denoted patch. The placed group keeps the transformed
// denoted record, so NormalAt is proven against it and covers the rotated
// normal with a bound far below the unit-normal limit of 2 that a group
// without a denoted patch is charged.
func TestFacetedPlacementKeepsDenotedProof(t *testing.T) {
	body := placementConeCut(t)
	src := placementConeFace(body)
	require.NotNil(t, src)
	require.NotNil(t, src.n.Denoted)
	p0 := r3.NewVec(3, 2, 0)
	n0, err := src.NormalAt(p0)
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(0, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		move r3.Transform
	}{{"rotate30Y", rot}, {"translate", trans}} {
		for _, mode := range []string{placeViaTransform, placeViaCopy} {
			t.Run(tc.name+"/"+mode, func(t *testing.T) {
				placed := placeBody(t, body, mode, tc.move)
				mv := placementConeFace(placed)
				require.NotNil(t, mv)
				require.NotNil(t, mv.n.Denoted, "the group keeps its denoted proof")

				want := tc.move.ApplyDir(n0.Value)
				got, err := mv.NormalAt(tc.move.Apply(p0))
				require.NoError(t, err)
				require.LessOrEqual(t, got.Value.Sub(want).Len(), got.Bound.Base(),
					"published normal bound does not cover the true error")
				// The denoted proof does not read NormalBound, which the
				// placement set to 2 for a group without a denoted patch. A
				// rigid placement moves the denoted record to its exact image,
				// so only rounding separates the two normals. A stale record
				// would instead charge the whole rotation.
				require.Less(t, got.Bound.Base(), 1e-9)
			})
		}
	}
}
