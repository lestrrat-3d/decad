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

func TestSectionDirectHoleFreeLoftFeedsExtrude(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 4)
	require.Empty(t, profiles[0].Holes)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-100), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-200), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectHoleFreeCircleLoftFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := loftCircleProfile(t, world, world.XY(), 2)
	top, topProfile := loftCircleProfile(t, world, topPlane, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 1)
	require.Empty(t, profiles[0].Holes)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-8*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectHoleFreeLoftRefusesUnprovedCases(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"bottom cap", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"top cap", r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted origin", r3.NewVec(1, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"tilted frame", r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 0)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, move)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, placed, r3.NewVec(0, 0, 1))
}

func TestSectionDirectHoleFreeLoftRefusesTwistedCorrespondence(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 10)
	require.NoError(t, err)
	points := [5][2]float64{{0, 0}, {10, 0}, {14, 5}, {9, 11}, {1, 8}}
	makeProfile := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		vertices := make([]*sketch.Point, len(points))
		for i, point := range points {
			vertices[i] = s.CreatePoint(point[0], point[1])
			s.Fix(vertices[i])
		}
		for i := range points {
			s.CreateLine(vertices[i], vertices[(i+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	bottom, bottomProfile := makeProfile(world.XY())
	top, topProfile := makeProfile(topPlane)
	twisted, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile,
		decad.WithLoftAlignment(1))
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, twisted, r3.NewVec(0, 0, 5))
}

func TestSectionDirectHoleFreeLoftRefusesShiftedSourceX(t *testing.T) {
	source := holeFreeLoftWithShiftedTop(t, r3.NewVec(1, 0, 2))
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 1))
}

func TestSectionDirectHoleFreeLoftRefusesShiftedSourceY(t *testing.T) {
	source := holeFreeLoftWithShiftedTop(t, r3.NewVec(0, 1, 2))
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 1))
}

func TestSectionDirectHoleFreeLoftRefusesShiftedRequestedY(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 1, 1))
}

func holeFreeLoftWithShiftedTop(t *testing.T, topOrigin r3.Vec) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	makeSquare := func(origin r3.Vec) (*sketch.Sketch, *sketch.Profile) {
		frame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		plane, err := world.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	bottom, bottomProfile := makeSquare(r3.NewVec(0, 0, 0))
	top, topProfile := makeSquare(topOrigin)
	source, err := decad.Loft(t.Context(), decad.New(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	return source
}
