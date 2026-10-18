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

func TestSectionRuledSquareLoftFeedsExtrude(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 4, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-81), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-162), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionRuledNonplanarWallsKeepDiagonalKinks(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	build := func(plane *sketch.Plane, points [4][2]float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		vertices := make([]*sketch.Point, len(points))
		for i, point := range points {
			vertices[i] = s.CreatePoint(point[0], point[1])
			s.Fix(vertices[i])
		}
		for i := range vertices {
			s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	lower := [4][2]float64{{-5, -5}, {5, -5}, {5, 5}, {-5, 5}}
	upper := [4][2]float64{{-4, -4}, {4, -3}, {4, 4}, {-4, 4}}
	bottom, bottomProfile := build(world.XY(), lower)
	top, topProfile := build(topPlane, upper)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 8)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-77.5), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-155), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionRuledLoftWithChangingHoleFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	build := func(plane *sketch.Plane, outerHalf, holeHalf float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		outer := s.CreateRectangle(-outerHalf, -outerHalf, outerHalf, outerHalf)
		s.Fix(outer.A)
		s.CreateRectangle(-holeHalf, -holeHalf, holeHalf, holeHalf)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 1 {
				return s, profile
			}
		}
		t.Fatal("no square region with a hole")
		return nil, nil
	}
	bottom, bottomProfile := build(world.XY(), 5, 1)
	top, topProfile := build(topPlane, 4, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-72), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-144), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionRuledLoftRefusesNonexactCrossing(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 4, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0.2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionRuledLoftRefusesShiftedCutOrigin(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 4, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
	}{
		{name: "X", origin: r3.NewVec(1, 0, 1)},
		{name: "Y", origin: r3.NewVec(0, 1, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
}

func TestSectionRuledLoftRefusesNonzeroAlignment(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	points := [5][2]float64{{0, 0}, {10, 0}, {14, 5}, {9, 11}, {1, 8}}
	build := func(plane *sketch.Plane, shift int) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		vertices := make([]*sketch.Point, len(points))
		for i := range points {
			point := points[(i+shift)%len(points)]
			vertices[i] = s.CreatePoint(point[0], point[1])
			s.Fix(vertices[i])
		}
		for i := range vertices {
			s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	bottom, bottomProfile := build(world.XY(), 0)
	top, topProfile := build(topPlane, 1)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile,
		decad.WithLoftAlignment(4))
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionRuledLoftRefusesDisplacedVertices(t *testing.T) {
	origin := r3.NewVec(1e9, 0, 0)
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, origin, 5, 5+math.Ldexp(1, -23), 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(origin.X, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}
