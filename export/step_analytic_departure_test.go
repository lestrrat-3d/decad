package export

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// departureLoft lofts a 10 x 10 square with a radius-1 hole at center between
// the plane of frame and its offset by 2 (agent-docs/loft-design.md §18): ten
// planes and one bore cylinder rimmed by two whole circles.
func departureLoft(t *testing.T, frame r3.Frame, center r3.Vec) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	lower, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(lower, 2)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{lower, upper} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		c := s.CreatePoint(center.X, center.Y)
		s.Fix(c)
		s.CreateCircle(c, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := decad.Loft(t.Context(), decad.New(), sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	return body
}

// departureFrame is a frame at origin with x along u and y along v.
func departureFrame(t *testing.T, origin, u r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}

// TestAnalyticSTEPRefusesADepartedTag writes bodies whose every face and edge
// the analytic face admission takes, but one or more of whose faces departs
// from its tag: a Loft bore whose held center rounds (1e9 + 0.427 has no
// float) and a Loft bore on a frame map that is not exactly orthonormal
// (loft-design §18), each stating a nonzero NormalBound on its bore; the
// rounded box's top chamfer, whose four corner cones are cap-blend band
// patches (modify-reach §8.3) stating a nonzero NormalBound; and a revolved
// frustum, whose Cone tag is not proven to be the cone its record denotes.
// The tilted loft's end cap states the loft's vertex displacement as its
// AxialDelta, since its tag and held points are not exactly on its section
// plane (internal/loftrecord CapOnRecord); every other loft
// cap here lies on its section plane exactly and states none. Each body is
// written faceted: one PLANE per ADVANCED_FACE and no analytic surface. A
// revolved tube, whose cylinders and annular caps are exactly their records,
// and the same square lofted on the world frame with its hole at the origin,
// whose bore and caps are exact, are written analytic. Shown to fail with
// supportsAnalyticSTEP's tagsUndeparted gate deleted (every departed body was
// written analytic), with TagDeparts' Denoted leg deleted (the frustum was
// written analytic), with tagIsDenoted always false (the tube was written
// faceted), and with the loft build's clearExactCapLevels call deleted (the
// exact loft was written faceted).
func TestAnalyticSTEPRefusesADepartedTag(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		build    func(t *testing.T) *decad.Body
		departed int
	}{
		{"loft bore with a rounded center", func(t *testing.T) *decad.Body {
			return departureLoft(t, departureFrame(t, r3.NewVec(1e9, 0, 0), r3.NewVec(1, 0, 0)), r3.NewVec(.427, 0, 0))
		}, 1},
		{"loft bore on a tilted frame map", func(t *testing.T) *decad.Body {
			return departureLoft(t, departureFrame(t, r3.Vec{}, r3.NewVec(1, 0, 4)), r3.Vec{})
		}, 2},
		{"loop chamfer corner cones", func(t *testing.T) *decad.Body {
			return patchRoundedBoxChamfer(t, 0)
		}, 4},
		{"revolved frustum", func(t *testing.T) *decad.Body {
			return coneFrustum(t, 5, 3)
		}, 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := tc.build(t)
			departed := 0
			for _, face := range body.Faces() {
				if facedeparture.Stated(face) {
					departed++
				}
			}
			require.Equal(t, tc.departed, departed)
			require.False(t, tagsUndeparted(body))
			ok, err := supportsAnalyticFaces(t.Context(), body)
			require.NoError(t, err)
			require.True(t, ok, "the face admission takes the body, so only the tag gate refuses it")
			ok, err = supportsAnalyticSTEP(t.Context(), body)
			require.NoError(t, err)
			require.False(t, ok)
			f, err := NewSTEPFile(t.Context(), body, units.Millimeters(0.1), bandHeader())
			require.NoError(t, err)
			data, err := f.Marshal()
			require.NoError(t, err)
			require.Contains(t, string(data), "faceted decad solid")
			byName := bandEntities(f)
			require.Equal(t, len(byName["ADVANCED_FACE"]), len(byName["PLANE"]), "one planar face per mesh triangle")
			require.Empty(t, byName["CYLINDRICAL_SURFACE"])
			require.Empty(t, byName["CONICAL_SURFACE"])
		})
	}
	t.Run("revolved tube", func(t *testing.T) {
		t.Parallel()
		body := departureTube(t)
		require.True(t, tagsUndeparted(body))
		f, err := NewSTEPFile(t.Context(), body, units.Millimeters(0.1), bandHeader())
		require.NoError(t, err)
		data, err := f.Marshal()
		require.NoError(t, err)
		require.Contains(t, string(data), "analytic decad solid")
		byName := bandEntities(f)
		require.Len(t, byName["ADVANCED_FACE"], 4)
		require.Equal(t, []float64{2, 5}, bandReals(byName["CYLINDRICAL_SURFACE"], 2))
		require.Len(t, byName["PLANE"], 2)
	})
	t.Run("exact loft bore", func(t *testing.T) {
		t.Parallel()
		body := departureLoft(t, departureFrame(t, r3.Vec{}, r3.NewVec(1, 0, 0)), r3.Vec{})
		require.True(t, tagsUndeparted(body))
		f, err := NewSTEPFile(t.Context(), body, units.Millimeters(0.1), bandHeader())
		require.NoError(t, err)
		data, err := f.Marshal()
		require.NoError(t, err)
		require.Contains(t, string(data), "analytic decad solid")
		byName := bandEntities(f)
		require.Len(t, byName["ADVANCED_FACE"], 11)
		require.Equal(t, []float64{1}, bandReals(byName["CYLINDRICAL_SURFACE"], 2))
		require.Len(t, byName["PLANE"], 10)
	})
}

// departureTube fully revolves the rectangle 0 <= u <= 10, 2 <= v <= 5 about
// the sketch's u axis: two cylinders, of radius 2 and 5, and two annular caps.
func departureTube(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 2, 10, 5)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}
