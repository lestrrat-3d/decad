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

func TestSectionMatchingRectangularHoleLoftFeedsExtrude(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftHoledSquares(t)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	require.Len(t, profiles[0].Holes, 1)
	require.Len(t, profiles[0].Holes[0], 4)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.Equal(t, 1500.0, area.Value.Base())
	require.Zero(t, area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.Equal(t, 3000.0, volume.Value.Base())
	require.Zero(t, volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionMatchingTwoCircleHolesLoftFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := sectionTwoCircleProfile(t, world, world.XY(), 2)
	top, topProfile := sectionTwoCircleProfile(t, world, topPlane, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 2)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-2*math.Pi)), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-4*math.Pi)), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionMatchingLoftRefusesDifferentHoleRecords(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := sectionTwoCircleProfile(t, world, world.XY(), 2)
	top, topProfile := sectionTwoCircleProfile(t, world, topPlane, math.Nextafter(2, 3))
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionMatchingLoftRefusesFacetedCutHole(t *testing.T) {
	origin := r3.NewVec(0, 0, 0.1)
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, origin, 5, 5, 2)
	doc := decad.New()
	blank, err := decad.Loft(t.Context(), doc, bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	world := sketch.NewWorld()
	toolFrame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	toolPlane, err := world.CreatePlaneFromFrame(toolFrame)
	require.NoError(t, err)
	toolSketch, err := world.CreateSketch(toolPlane)
	require.NoError(t, err)
	center := toolSketch.CreatePoint(0, 0)
	toolSketch.Fix(center)
	toolSketch.CreateCircle(center, 1)
	_, err = toolSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, toolSketch, toolSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	sectionFrame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := cut.Section(t.Context(), sectionFrame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionMatchingLoftRefusesFilletRewrite(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(-5, -5, 0))).Exactly(1)
	rounded, err := decad.Fillet(t.Context(), source, corner, units.Millimeters(1))
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := rounded.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func sectionTwoCircleProfile(t *testing.T, world *sketch.World, plane *sketch.Plane,
	secondCenterX float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-5, -5, 5, 5)
	s.Fix(rect.A)
	for _, x := range []float64{-2, secondCenterX} {
		center := s.CreatePoint(x, 0)
		s.Fix(center)
		s.CreateCircle(center, 1)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Outer) == 4 && len(profile.Holes) == 2 {
			return s, profile
		}
	}
	t.Fatal("no matching two-hole region")
	return nil, nil
}
