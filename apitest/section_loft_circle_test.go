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

func TestSectionDirectLoftAnnulusFeedsExtrude(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	require.NoError(t, decad.Remove(source))
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 1)
	require.Len(t, profiles[0].Holes, 1)
	require.Len(t, profiles[0].Holes[0], 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-13.75*math.Pi), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-27.5*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftRectangularOuterFeedsExtrude(t *testing.T) {
	source := directLoftCircles(t, false, 1, 2, 0, 0)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 4)
	require.Len(t, profiles[0].Holes, 1)
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(100-2.25*math.Pi)), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(200-4.5*math.Pi)), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftVaryingCircleFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := loftCircleProfile(t, world, world.XY(), 1)
	top, topProfile := loftCircleProfile(t, world, topPlane, 3)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 1)
	require.Equal(t, 2.0, profiles[0].Outer[0].Entity.(*sketch.Circle).R())
	area, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	areaMeasure, err := area.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(areaMeasure.Value.Base()-4*math.Pi), areaMeasure.Bound.Base()+1e-12)
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

func TestSectionDirectLoftVaryingCircleTranslatedFrameFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	bottomFrame, err := r3.NewFrame(r3.NewVec(10, 20, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	bottomPlane, err := world.CreatePlaneFromFrame(bottomFrame)
	require.NoError(t, err)
	topPlane, err := world.CreateOffsetPlane(bottomPlane, 2)
	require.NoError(t, err)
	bottom, bottomProfile := loftCircleProfile(t, world, bottomPlane, 1)
	top, topProfile := loftCircleProfile(t, world, topPlane, 3)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(10, 20, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Equal(t, 2.0, profiles[0].Outer[0].Entity.(*sketch.Circle).R())
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

func TestSectionDirectLoftMovingCircleTracksCenter(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 3)
	require.NoError(t, err)
	build := func(plane *sketch.Plane, centerX float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(centerX, 0)
		s.Fix(center)
		s.CreateCircle(center, 4)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	bottom, bottomProfile := build(world.XY(), 0)
	top, topProfile := build(topPlane, 1)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1.5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	circle := profiles[0].Outer[0].Entity.(*sketch.Circle)
	require.Equal(t, 4.0, circle.R())
	require.Equal(t, 0.5, circle.Center.X())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-32*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 0.2))
}

func TestSectionDirectLoftVaryingAnnulusFeedsExtrude(t *testing.T) {
	source := directLoftCirclesWithOuterRadii(t, true, 4, 5, 1, 2, 1, 0)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Outer, 1)
	require.Len(t, profiles[0].Holes, 1)
	require.Equal(t, 4.5, profiles[0].Outer[0].Entity.(*sketch.Circle).R())
	hole := profiles[0].Holes[0][0].Entity.(*sketch.Circle)
	require.Equal(t, 1.5, hole.R())
	require.Equal(t, 0.5, hole.Center.X())
	area, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	areaMeasure, err := area.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(areaMeasure.Value.Base()-18*math.Pi), areaMeasure.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-36*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftVaryingCircleWithTwoHolesFeedsExtrude(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	build := func(plane *sketch.Plane, outerRadius, holeRadius float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		outer := s.CreatePoint(0, 0)
		s.Fix(outer)
		s.CreateCircle(outer, outerRadius)
		for _, x := range []float64{-3, 3} {
			center := s.CreatePoint(x, 0)
			s.Fix(center)
			s.CreateCircle(center, holeRadius)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			if profile.Valid && len(profile.Holes) == 2 {
				return s, profile
			}
		}
		t.Fatal("no circular profile with two holes")
		return nil, nil
	}
	bottom, bottomProfile := build(world.XY(), 8, 1)
	top, topProfile := build(topPlane, 9, 1.5)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := source.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 2)
	require.Equal(t, 8.5, profiles[0].Outer[0].Entity.(*sketch.Circle).R())
	for _, hole := range profiles[0].Holes {
		require.Equal(t, 1.25, hole[0].Entity.(*sketch.Circle).R())
	}
	measured, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-69.125*math.Pi), area.Bound.Base()+1e-12)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-138.25*math.Pi), volume.Bound.Base()+1e-12)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSectionDirectLoftVaryingCircleRefusesUnprovedPlanes(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := loftCircleProfile(t, world, world.XY(), 1)
	top, topProfile := loftCircleProfile(t, world, topPlane, 3)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"bottom cap", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"top cap", r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"nonexact radius", r3.NewVec(0, 0, 0.2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted X", r3.NewVec(1, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"shifted Y", r3.NewVec(0, 1, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"tilted", r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 0)},
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
}

func TestSectionDirectLoftVaryingCircleRefusesPlacement(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 2)
	require.NoError(t, err)
	bottom, bottomProfile := loftCircleProfile(t, world, world.XY(), 1)
	top, topProfile := loftCircleProfile(t, world, topPlane, 3)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), source, move)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, placed, r3.NewVec(0, 0, 1))
}

func TestSectionDirectLoftVaryingCircleRefusesShiftedSourceOrigin(t *testing.T) {
	for _, tc := range []struct {
		name  string
		shift r3.Vec
	}{
		{name: "X", shift: r3.NewVec(1, 0, 2)},
		{name: "Y", shift: r3.NewVec(0, 1, 2)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			world := sketch.NewWorld()
			topFrame, err := r3.NewFrame(tc.shift, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			topPlane, err := world.CreatePlaneFromFrame(topFrame)
			require.NoError(t, err)
			bottom, bottomProfile := loftCircleProfile(t, world, world.XY(), 1)
			top, topProfile := loftCircleProfile(t, world, topPlane, 3)
			source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
			require.NoError(t, err)
			requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 1))
		})
	}
}

func TestSectionDirectLoftMovingCircleRefusesNonexactY(t *testing.T) {
	world := sketch.NewWorld()
	topPlane, err := world.CreateOffsetPlane(world.XY(), 3)
	require.NoError(t, err)
	build := func(plane *sketch.Plane, centerY float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(0, centerY)
		s.Fix(center)
		s.CreateCircle(center, 4)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	bottom, bottomProfile := build(world.XY(), 0)
	top, topProfile := build(topPlane, 1)
	source, err := decad.New().Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 0.2))
}

func TestSectionDirectLoftAnnulusRefusesCapAndTilt(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	for _, tc := range []struct {
		name   string
		origin r3.Vec
		u, v   r3.Vec
	}{
		{"bottom cap", r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"top cap", r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"nonexact circle interpolation", r3.NewVec(0, 0, 0.2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)},
		{"tilted section", r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 0)},
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
}

func requireDirectLoftSectionUnsupported(t *testing.T, body *decad.Body, origin r3.Vec) {
	t.Helper()
	frame, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	section, profiles, err := body.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, section)
	require.Nil(t, profiles)
}

func TestSectionDirectLoftRefusesPlacementAndShiftedOrigin(t *testing.T) {
	source := directLoftCircles(t, true, 1, 2, 1, 0)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(1, 0, 1))
	constant := directLoftCircles(t, false, 1, 1, 0, 0)
	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), constant, move)
	require.NoError(t, err)
	requireDirectLoftSectionUnsupported(t, placed, r3.NewVec(0, 0, 1))
}

func TestSectionDirectLoftRefusesNonExactCenter(t *testing.T) {
	source := directLoftCircles(t, false, 1, 1, 3, 0)
	requireDirectLoftSectionUnsupported(t, source, r3.NewVec(0, 0, 0.2))
}

func TestSectionDirectLoftRefusesCutSource(t *testing.T) {
	bottom, bottomProfile, top, topProfile := loftSquaresAt(t, r3.NewVec(0, 0, 0), 5, 5, 2)
	doc := decad.New()
	blank, err := doc.Loft(t.Context(), bottom, bottomProfile, top, topProfile)
	require.NoError(t, err)
	tool := verticalCylinder(t, doc, 0, 0, 1)
	cut, err := decad.Cut(t.Context(), blank, tool)
	require.NoError(t, err)
	faces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(cut))).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	require.Len(t, faces[0].Loops(), 2)
	require.IsType(t, decad.Circle3{}, faces[0].Loops()[1].Edges()[0].Curve())
	requireDirectLoftSectionUnsupported(t, cut, r3.NewVec(0, 0, 1))
}
