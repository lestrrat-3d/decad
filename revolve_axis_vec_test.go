package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// axisVecSketch draws a quadrilateral on plane whose points all lie on one side
// of every axis the tests use, and returns its sketch and profile.
func axisVecSketch(t *testing.T, world *sketch.World, plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	outline := [][2]float64{{0, 2}, {0, 4}, {0.5, 4}, {0.5, 2.5}}
	points := make([]*sketch.Point, len(outline))
	for i, xy := range outline {
		points[i] = s.CreatePoint(xy[0], xy[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func revolvedAxisVec(t *testing.T, plane func(*sketch.World) *sketch.Plane, axis Axis) r3.Vec {
	t.Helper()
	world := sketch.NewWorld()
	s, profile := axisVecSketch(t, world, plane(world))
	body, err := Revolve(t.Context(), New(), s, profile, axis, FullRevolution{})
	require.NoError(t, err)
	return body.n.Payload.(revolvePayload).AxisVec
}

func xyPlane(w *sketch.World) *sketch.Plane { return w.XY() }

// TestRevolveRecordsAxisVec shows which Revolves record the exact axis vector
// (agent-docs/revolve-axis-design.md §3.6): only a frame whose origin is the
// world origin and whose U and V are signed basis vectors, an anchor at the
// plane origin, and a direction read from stored floats without subtraction.
func TestRevolveRecordsAxisVec(t *testing.T) {
	t.Parallel()
	t.Run("sketch line", func(t *testing.T) {
		require.Equal(t, r3.Vec{X: 1, Y: 2},
			revolvedAxisVec(t, xyPlane, SketchLine{End: Point2{U: 1, V: 2}}))
		// The direction is the stored end point, not a normalized copy.
		require.Equal(t, r3.Vec{X: 3, Y: 6},
			revolvedAxisVec(t, xyPlane, SketchLine{End: Point2{U: 3, V: 6}}))
	})
	t.Run("construction axis", func(t *testing.T) {
		require.Equal(t, r3.Vec{X: 1, Y: 2},
			revolvedAxisVec(t, xyPlane, ConstructionAxis{Dir: r3.Vec{X: 1, Y: 2}}))
		// An out-of-plane component within the plane gate's tolerance is dropped.
		require.Equal(t, r3.Vec{X: 1, Y: 2},
			revolvedAxisVec(t, xyPlane, ConstructionAxis{Dir: r3.Vec{X: 1, Y: 2, Z: 1e-12}}))
	})
	t.Run("side flip", func(t *testing.T) {
		// The region lies on the far side of this axis, so the Revolve flips the
		// axis frame and the record carries the flipped direction.
		require.Equal(t, r3.Vec{X: 1, Y: 2},
			revolvedAxisVec(t, xyPlane, SketchLine{End: Point2{U: -1, V: -2}}))
	})
	t.Run("canonical", func(t *testing.T) {
		world := sketch.NewWorld()
		s, profile := axisVecSketch(t, world, world.XY())
		body, err := Revolve(t.Context(), New(), s, profile, SketchLine{End: Point2{U: 1}}, FullRevolution{})
		require.NoError(t, err)
		rp := body.n.Payload.(revolvePayload)
		require.Equal(t, r3.Vec{X: 1}, rp.AxisVec)
		require.True(t, coneaxis.FrameCanonical(rp.Frame, rp.Ax))
	})
	t.Run("other coordinate planes", func(t *testing.T) {
		for name, plane := range map[string]func(*sketch.World) *sketch.Plane{
			"XZ": func(w *sketch.World) *sketch.Plane { return w.XZ() },
			"YZ": func(w *sketch.World) *sketch.Plane { return w.YZ() },
		} {
			world := sketch.NewWorld()
			s, profile := axisVecSketch(t, world, plane(world))
			body, err := Revolve(t.Context(), New(), s, profile, SketchLine{End: Point2{U: 1, V: 2}}, FullRevolution{})
			require.NoError(t, err, name)
			rp := body.n.Payload.(revolvePayload)
			// The recorded vector is the plane direction read in the frame's axes.
			require.Equal(t, rp.Frame.U().Add(rp.Frame.V().Scale(2)), rp.AxisVec, name)
			require.NotEqual(t, r3.Vec{}, rp.AxisVec, name)
			axis, ok := coneaxis.OfRevolve(rp.Frame, rp.Ax, rp.AxisVec)
			require.True(t, ok, name)
			require.Equal(t, rp.AxisVec, axis.Vec(), name)
		}
	})
	t.Run("chain", func(t *testing.T) {
		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		a, b := s.CreatePoint(0, 2), s.CreatePoint(0.5, 4)
		s.Fix(a)
		s.Fix(b)
		s.CreateLine(a, b)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Chains(), 1)
		body, err := Revolve(t.Context(), New(), s, s.Chains()[0], SketchLine{End: Point2{U: 1, V: 2}}, FullRevolution{})
		require.NoError(t, err)
		require.Equal(t, r3.Vec{X: 1, Y: 2}, body.n.Payload.(chainRevolvePayload).AxisVec)
	})
	t.Run("copy keeps the record", func(t *testing.T) {
		world := sketch.NewWorld()
		s, profile := axisVecSketch(t, world, world.XY())
		doc := New()
		body, err := Revolve(t.Context(), doc, s, profile, SketchLine{End: Point2{U: 1, V: 2}}, FullRevolution{})
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Z: 3})
		require.NoError(t, err)
		placed, err := Copy(t.Context(), body, WithCopyPlacement(move))
		require.NoError(t, err)
		require.Equal(t, r3.Vec{X: 1, Y: 2}, placed.n.Payload.(revolvePayload).AxisVec)
	})
	t.Run("refusals leave the vector zero", func(t *testing.T) {
		rotated, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0.6, 0.8, 0), r3.NewVec(-0.8, 0.6, 0))
		require.NoError(t, err)
		for name, tc := range map[string]struct {
			plane func(*sketch.World) *sketch.Plane
			axis  Axis
		}{
			"start is not the origin": {xyPlane, SketchLine{Start: Point2{U: 1}, End: Point2{U: 2, V: 2}}},
			"construction origin off": {xyPlane, ConstructionAxis{Origin: r3.Vec{X: 0.5}, Dir: r3.Vec{X: 1, Y: 2}}},
			"construction origin on the plane but moved": {xyPlane,
				ConstructionAxis{Origin: r3.Vec{Y: 0.25}, Dir: r3.Vec{X: 1, Y: 2}}},
			"non-coordinate frame": {func(w *sketch.World) *sketch.Plane {
				p, err := w.CreatePlaneFromFrame(rotated)
				require.NoError(t, err)
				return p
			}, SketchLine{End: Point2{U: 1, V: 2}}},
			"frame origin moved": {func(w *sketch.World) *sketch.Plane {
				p, err := w.CreateOffsetPlane(w.XY(), 1)
				require.NoError(t, err)
				return p
			}, SketchLine{End: Point2{U: 1, V: 2}}},
		} {
			got := revolvedAxisVec(t, tc.plane, tc.axis)
			require.Equal(t, r3.Vec{}, got, name)
		}
	})
}
