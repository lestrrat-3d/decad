package decad

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFaceDepartureReader runs the reader the STEP writer's tag gate calls on
// face nodes that state no departure, a NormalBound, a retained denoted
// face's HeldNormalBound, and on a value that is not a face. Only the first
// reads false. Shown to fail with each of the two bound legs deleted (that
// face read false) and with the non-face case returning false.
func TestFaceDepartureReader(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		face any
		want bool
	}{
		{"exact tag", bornFace(&Face{}), false},
		{"normal bound", bornFace(&Face{n: topo.Face{NormalBound: 1e-16}}), true},
		{"held normal bound", bornFace(&Face{n: topo.Face{HeldNormalBound: 1e-16}}), true},
		{"not a face", bornEdge(&Edge{}), true},
		{"nil face", (*Face)(nil), true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, facedeparture.Stated(tc.face))
		})
	}
}

// departureRevolve fully revolves the closed polygon pts about the sketch's u
// axis, or, when circle is set, the circle of radius circle about pts[0].
func departureRevolve(t *testing.T, pts [][2]float64, circle float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	if circle > 0 {
		c := s.CreatePoint(pts[0][0], pts[0][1])
		s.Fix(c)
		s.CreateCircle(c, circle)
	} else {
		points := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			points[i] = s.CreatePoint(p[0], p[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	body, err := Revolve(t.Context(), New(), s, s.Profiles()[0], axis, FullRevolution{})
	require.NoError(t, err)
	return body
}

// departureSphere fully revolves the half disk of radius 5 about (5, 0).
func departureSphere(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	end, c := s.CreatePoint(10, 0), s.CreatePoint(5, 0)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	body, err := Revolve(t.Context(), New(), s, s.Profiles()[0], axis, FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestFaceTagDepartsOnRevolveRecords reads TagDeparts on the faces of full
// revolutions, every one of which carries a denoted surface and a zero
// NormalBound. A tube's cylinders and annular planes, a ball's sphere and a
// ring's torus have tags exactly equal to their records, so none departs. A
// frustum's Cone tag is built from a float apex and an atan2 half-angle and
// is never admitted, so it departs. The ball's sphere with its tag radius
// moved off the record's departs too. Shown to fail with TagDeparts' Denoted
// leg deleted (the cone read false), with tagIsDenoted always false (the
// exact faces departed), with the Cone tag decided by stitchflux.TagIsDenoted
// (the cone read false), and with sphereRadiusIsRecord deleted (the moved
// sphere read false).
func TestFaceTagDepartsOnRevolveRecords(t *testing.T) {
	t.Parallel()
	kinds := func(t *testing.T, body *Body, want bool) map[string]int {
		t.Helper()
		out := map[string]int{}
		for _, f := range body.Faces() {
			require.NotNil(t, f.n.Denoted, "a full revolution's face carries its record")
			require.Zero(t, f.n.NormalBound)
			require.Equal(t, want, facedeparture.TagDeparts(&f.n), "%T", f.n.Surface)
			out[fmt.Sprintf("%T", f.n.Surface)]++
		}
		return out
	}
	t.Run("tube", func(t *testing.T) {
		t.Parallel()
		body := departureRevolve(t, [][2]float64{{0, 2}, {10, 2}, {10, 5}, {0, 5}}, 0)
		require.Equal(t, map[string]int{"surfacegeom.Cylinder": 2, "surfacegeom.Plane": 2}, kinds(t, body, false))
	})
	t.Run("ring", func(t *testing.T) {
		t.Parallel()
		body := departureRevolve(t, [][2]float64{{0, 8}}, 2)
		require.Equal(t, map[string]int{"surfacegeom.Torus": 1}, kinds(t, body, false))
	})
	t.Run("ball", func(t *testing.T) {
		t.Parallel()
		body := departureSphere(t)
		require.Equal(t, map[string]int{"surfacegeom.Sphere": 1}, kinds(t, body, false))
		moved := body.Faces()[0].n
		sphere, ok := moved.Surface.(surfacegeom.Sphere)
		require.True(t, ok)
		sphere.Radius = units.Millimeters(6)
		moved.Surface = sphere
		require.True(t, facedeparture.TagDeparts(&moved), "a sphere tag off its recorded radius")
	})
	t.Run("frustum", func(t *testing.T) {
		t.Parallel()
		body := departureRevolve(t, [][2]float64{{0, 0}, {10, 0}, {10, 3}, {0, 5}}, 0)
		for _, f := range body.Faces() {
			_, cone := f.n.Surface.(surfacegeom.Cone)
			require.NotNil(t, f.n.Denoted)
			require.Equal(t, cone, facedeparture.TagDeparts(&f.n), "%T", f.n.Surface)
		}
	})
}
