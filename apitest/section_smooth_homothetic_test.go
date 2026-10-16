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

func homotheticTriangleSections(t *testing.T, origin r3.Vec) [3]decad.LoftSection {
	t.Helper()
	return homotheticPolygonSections(t, origin, [][2]float64{{-2, -1}, {2, -1}, {0, 2}})
}

func homotheticPolygonSections(t *testing.T, origin r3.Vec, coords [][2]float64) [3]decad.LoftSection {
	t.Helper()
	world := sketch.NewWorld()
	base, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	basePlane, err := world.CreatePlaneFromFrame(base)
	require.NoError(t, err)
	var sections [3]decad.LoftSection
	for i, scale := range [...]float64{1, 1.5, 1} {
		plane := basePlane
		if i != 0 {
			plane, err = world.CreateOffsetPlane(basePlane, float64(2*i))
			require.NoError(t, err)
		}
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		points := make([]*sketch.Point, len(coords))
		for j, p := range coords {
			points[j] = s.CreatePoint(scale*p[0], scale*p[1])
			s.Fix(points[j])
		}
		for j := range points {
			s.CreateLine(points[j], points[(j+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		profiles := s.Profiles()
		require.Len(t, profiles, 1)
		sections[i] = decad.LoftSection{Sketch: s, Profile: profiles[0]}
	}
	return sections
}

func TestSectionHomotheticConcavePolygonFeedsVerifiedExtrude(t *testing.T) {
	sections := homotheticPolygonSections(t, r3.Vec{},
		[][2]float64{{-2, -2}, {2, -2}, {2, 2}, {0, 1}, {-2, 2}})
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-31.5), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-31.5), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionHomotheticThreeSectionLoftFeedsExtrude(t *testing.T) {
	sections := homotheticTriangleSections(t, r3.Vec{})
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-11.34375), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-22.6875), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionHomotheticThreeSectionLoftKeepsTranslatedSourceAndMiddleLevel(t *testing.T) {
	sections := homotheticTriangleSections(t, r3.NewVec(10, 20, 3))
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(10, 20, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-13.5), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-27), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionHomotheticThreeSectionLoftRefusesUnprovedInputs(t *testing.T) {
	sections := homotheticTriangleSections(t, r3.Vec{})
	body, err := decad.New().LoftSections(t.Context(), sections[:]...)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"lower cap", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"upper cap", r3.NewVec(0, 0, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"outside", r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted origin", r3.NewVec(1, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"rotated axes", r3.NewVec(0, 0, 1), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0)},
		{"inexact scale", r3.NewVec(0, 0, 0.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			_, _, err = body.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
	copied, err := decad.Copy(t.Context(), body)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	_, _, err = copied.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
