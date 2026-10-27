package export

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/facedeparture"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
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
// the analytic face admission takes, but one or more of whose faces states a
// nonzero departure from its tag: a Loft bore whose held center rounds
// (1e9 + 0.427 has no float), a Loft bore on a frame map that is not exactly
// orthonormal (loft-design §18), and the rounded box's top chamfer, whose four
// corner cones are cap-blend band patches (modify-reach §8.3). Each is written
// faceted: one PLANE per ADVANCED_FACE and no analytic surface. The same
// square lofted on the world frame with its hole at the origin states no
// departure and is written analytic, its bore a CYLINDRICAL_SURFACE of radius
// 1 placed on the z axis. Shown to fail with supportsAnalyticSTEP's
// tagsUndeparted gate deleted (every departed body was written analytic).
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
		}, 1},
		{"loop chamfer corner cones", func(t *testing.T) *decad.Body {
			return patchRoundedBoxChamfer(t, 0)
		}, 4},
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
		require.Len(t, byName["ADVANCED_FACE"], len(body.Faces()))
		require.Len(t, byName["CYLINDRICAL_SURFACE"], 1)
		require.Equal(t, []float64{1}, bandReals(byName["CYLINDRICAL_SURFACE"], 2))
		byID := map[uint64]step.Entity{}
		for _, e := range f.Entities {
			byID[e.ID] = e
		}
		ref := func(v step.Value) step.Entity { return byID[uint64(v.(step.Reference))] }
		cylinder := byName["CYLINDRICAL_SURFACE"][0]
		placement := ref(cylinder.Parameters[1])
		location := ref(placement.Parameters[1]).Parameters[1].(step.List)
		axis := ref(placement.Parameters[2]).Parameters[1].(step.List)
		require.Equal(t, step.List{step.Real(0), step.Real(0)}, location[:2], "the bore's axis passes through the origin")
		require.Equal(t, step.List{step.Real(0), step.Real(0), step.Real(1)}, axis)
	})
}
