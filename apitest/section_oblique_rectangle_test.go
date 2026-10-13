package apitest_test

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func obliqueSectionCube(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	body, err := decad.Extrude(t.Context(), decad.New(), s, profiles[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

func obliqueSectionFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: 5}, r3.Vec{Y: 1}, r3.Vec{X: 5, Z: 1})
	require.NoError(t, err)
	return frame
}

func TestSectionObliqueRectangleFeedsVerifiedExtrude(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	frame := obliqueSectionFrame(t)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	_, err = decad.MeasureProfile(section, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)

	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.Positive(t, volume.Bound.Base())
	slope := frame.V().Z / frame.V().X
	trueVolume := 20 * math.Hypot(10, 10*slope)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-trueVolume), volume.Bound.Base()+1e-10)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueVerticalRectangleFeedsVerifiedExtrude(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	frame, err := r3.NewFrame(r3.Vec{X: 5}, r3.Vec{X: 1, Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionObliqueRectangleTriangleThroughHexagon(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
		edges  int
	}{
		{"triangle", r3.Vec{X: 5}, r3.Vec{X: 1, Y: -1}, r3.Vec{X: 1, Y: 1, Z: -2}, 3},
		{"quadrilateral", r3.Vec{Z: 5}, r3.Vec{Y: 1}, r3.Vec{X: 5, Z: 1}, 4},
		{"pentagon", r3.Vec{Z: 7.5}, r3.Vec{X: 1, Y: -1}, r3.Vec{X: 1, Y: 1, Z: -1}, 5},
		{"hexagon", r3.Vec{X: 5, Y: 5, Z: 5}, r3.Vec{X: 1, Y: -1}, r3.Vec{X: 1, Y: 1, Z: -2}, 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			section, profiles, err := body.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			require.Len(t, profiles[0].Outer, tc.edges)
			child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
				D: units.Millimeters(2), Dir: decad.Along,
			})
			require.NoError(t, err)
			mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestSectionObliqueRectanglePlacementAndAxisSense(t *testing.T) {
	t.Parallel()
	source := obliqueSectionCube(t)
	placement, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{X: -1}, EZ: r3.Vec{Z: 1},
	}, r3.Vec{X: 20, Y: 30, Z: 7})
	require.NoError(t, err)
	body, err := decad.Transform(t.Context(), source, placement)
	require.NoError(t, err)
	for _, u := range []r3.Vec{{X: -1}, {X: 1}} {
		frame, err := r3.NewFrame(r3.Vec{X: 20, Y: 30, Z: 12}, u, r3.Vec{Y: 5, Z: 1})
		require.NoError(t, err)
		section, profiles, err := body.Section(t.Context(), frame)
		require.NoError(t, err)
		require.Len(t, profiles, 1)
		child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
			D: units.Millimeters(2), Dir: decad.Along,
		})
		require.NoError(t, err)
		mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	}
}

func TestSectionObliqueRectangleCertificateRefusesEditsAndCopies(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		change func(*testing.T, *sketch.Sketch) *sketch.Sketch
	}{
		{name: "stale reference", change: func(_ *testing.T, s *sketch.Sketch) *sketch.Sketch {
			s.MarkStale(s.Points()[0].Source())
			return s
		}},
		{name: "deleted marker", change: func(t *testing.T, s *sketch.Sketch) *sketch.Sketch {
			for _, entity := range s.Entities() {
				require.True(t, s.RemoveEntity(entity))
			}
			for _, point := range s.Points() {
				require.True(t, s.RemovePoint(point))
			}
			return s
		}},
		{name: "shallow copy", change: func(_ *testing.T, s *sketch.Sketch) *sketch.Sketch {
			copySketch := *s
			return &copySketch
		}},
		{name: "JSON reload", change: func(t *testing.T, s *sketch.Sketch) *sketch.Sketch {
			data, err := json.Marshal(s)
			require.NoError(t, err)
			require.NoError(t, json.Unmarshal(data, s))
			return s
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := obliqueSectionCube(t)
			section, profiles, err := body.Section(t.Context(), obliqueSectionFrame(t))
			require.NoError(t, err)
			section = tc.change(t, section)
			_, err = decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
				D: units.Millimeters(2), Dir: decad.Along,
			})
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

func TestSectionObliqueRectangleMarkerCannotSupplyCertificate(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	bounded, _, err := body.Section(t.Context(), obliqueSectionFrame(t))
	require.NoError(t, err)
	marker := bounded.Points()[0].Source()
	require.NotEmpty(t, marker)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	coordinates := [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}
	points := make([]*sketch.Point, len(coordinates))
	for i, xy := range coordinates {
		points[i] = s.CreateReferencePoint(xy[0], xy[1], marker)
	}
	for i := range points {
		_, err := s.CreateReferenceLine(points[i], points[(i+1)%len(points)], marker)
		require.NoError(t, err)
	}
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	_, err = decad.MeasureProfile(s, profiles[0])
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Extrude(t.Context(), decad.New(), s, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSectionObliqueRectangleOtherRegionCannotBypassBound(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	section, profiles, err := body.Section(t.Context(), obliqueSectionFrame(t))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	coordinates := [][2]float64{{30, 30}, {40, 30}, {40, 40}, {30, 40}}
	points := make([]*sketch.Point, len(coordinates))
	for i, xy := range coordinates {
		points[i] = section.CreateReferencePoint(xy[0], xy[1], "ordinary")
	}
	for i := range points {
		_, err := section.CreateReferenceLine(points[i], points[(i+1)%len(points)], "ordinary")
		require.NoError(t, err)
	}
	require.Len(t, section.Profiles(), 2)
	for _, profile := range section.Profiles() {
		if !profile.Valid {
			continue
		}
		_, err = decad.MeasureProfile(section, profile)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		_, err = decad.Extrude(t.Context(), decad.New(), section, profile, decad.Distance{
			D: units.Millimeters(2), Dir: decad.Along,
		})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
}

func TestSectionObliqueRectangleRefusesUnsupportedExtrudeOptions(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	section, profiles, err := body.Section(t.Context(), obliqueSectionFrame(t))
	require.NoError(t, err)
	profile := profiles[0]
	_, err = decad.Extrude(t.Context(), decad.New(), section, profile,
		decad.Symmetric{D: units.Millimeters(2)})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Extrude(t.Context(), decad.New(), section, profile,
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along}, decad.WithTaper(units.Degrees(5)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Extrude(t.Context(), decad.New(), section, profile,
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along}, decad.WithSurfaceResult())
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestSectionObliqueRectangleRefusesContact(t *testing.T) {
	t.Parallel()
	body := obliqueSectionCube(t)
	for _, origin := range []r3.Vec{
		{X: 10, Y: 0, Z: 10}, // a box vertex
		{X: 0, Y: 0, Z: 10},  // a cap-edge contact
		{X: 0, Y: 0, Z: 20},  // outside
	} {
		frame, err := r3.NewFrame(origin, r3.Vec{Y: 1}, r3.Vec{X: 5, Z: 1})
		require.NoError(t, err)
		section, profiles, err := body.Section(t.Context(), frame)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Nil(t, section)
		require.Empty(t, profiles)
	}
}
