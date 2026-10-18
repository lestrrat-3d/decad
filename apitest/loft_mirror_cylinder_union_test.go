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

// This generic radial tab has the six-curve outline admitted by the circular
// Join: two root connectors, two fitted walls, an outer arc, and an inner arc.
func loftMirrorTabSection(t *testing.T, world *sketch.World, plane *sketch.Plane,
	turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	polar := func(radius, angle float64) *sketch.Point {
		point := s.CreatePoint(radius*math.Cos(angle+turn), radius*math.Sin(angle+turn))
		s.Fix(point)
		return point
	}
	rightRoot := polar(3, -0.18)
	leftRoot := polar(3, 0.18)
	right := []*sketch.Point{polar(3.6, -0.18), polar(4.0, -0.15), polar(4.4, -0.11)}
	left := []*sketch.Point{polar(3.6, 0.18), polar(4.0, 0.15), polar(4.4, 0.11)}
	s.CreateLine(rightRoot, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[2], left[2])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(center, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	require.Len(t, profiles[0].Outer, 6)
	return s, profiles[0]
}

func TestLoftMirrorCylinderUnionJoinsRepeatedRadialTabs(t *testing.T) {
	doc := decad.New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorTabSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := loftMirrorTabSection(t, world, middlePlane, 0.1)
	lower, err := decad.Loft(t.Context(), doc, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	// Moving the top blank cap off the reflection height must refuse the
	// specialized route instead of accepting a near-coincident middle seam.
	otherBlank := circleBodyAtZ(t, doc, 0, 3, 0, 2.1)
	_, err = decad.Union(t.Context(), otherBlank, pair)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	quarterTurn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	turnedPair, err := decad.Copy(t.Context(), pair, decad.WithCopyPlacement(quarterTurn))
	require.NoError(t, err)
	blank := circleBodyAtZ(t, doc, 0, 3, 0, 2)
	joined, err := decad.Union(t.Context(), blank, pair)
	require.NoError(t, err)
	joined, err = decad.Union(t.Context(), joined, turnedPair)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, role := range []decad.FeatureRef{
		decad.CapStart(blank), decad.CapEnd(blank), decad.CapStart(pair), decad.CapEnd(pair),
		decad.CapStart(turnedPair), decad.CapEnd(turnedPair),
		decad.CapStart(joined), decad.CapEnd(joined),
	} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(joined)
		require.NoError(t, err)
		require.IsType(t, decad.Plane{}, faces[0].Surface())
	}
	centroid, err := joined.Centroid()
	require.NoError(t, err)
	require.Greater(t, centroid.Bound.Base(), 0.0)
}

func TestUnionMirroredRadialTabsWithNearAxialCopy(t *testing.T) {
	doc := decad.New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorTabSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := loftMirrorTabSection(t, world, middlePlane, 0.2)
	lower, err := decad.Loft(t.Context(), doc, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	angle := 2 * math.Pi / 17
	// These two copies are 0.45 radians apart; the 0.2-radian twist
	// requires a height-dependent separator for their full source walls.
	firstMotion, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(10*angle-0.45))
	require.NoError(t, err)
	firstCopy, err := decad.Copy(t.Context(), pair, decad.WithCopyPlacement(firstMotion))
	require.NoError(t, err)
	nearMotion, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(10*angle))
	require.NoError(t, err)
	require.Equal(t, math.Nextafter(1, 0), nearMotion.Basis().EZ.Z)
	nearCopy, err := decad.Copy(t.Context(), pair, decad.WithCopyPlacement(nearMotion))
	require.NoError(t, err)
	blank := circleBodyAtZ(t, doc, 0, 3, 0, 2)
	joined, err := decad.Union(t.Context(), blank, pair, firstCopy, nearCopy)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, source := range []*decad.Body{blank, pair, firstCopy, nearCopy, joined} {
		bottom, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(source))).Exactly(1).SelectFaces(joined)
		require.NoError(t, err)
		require.IsType(t, decad.Plane{}, bottom[0].Surface())
		top, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(source))).Exactly(1).SelectFaces(joined)
		require.NoError(t, err)
		require.IsType(t, decad.Faceted{}, top[0].Surface())
	}
	volume, err := joined.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base(), 0.0)
	require.Greater(t, volume.Bound.Base(), 0.0)
	centroid, err := joined.Centroid()
	require.NoError(t, err)
	require.Greater(t, centroid.Bound.Base(), 0.0)
}

func loftMirrorCylinderTwoTabs(t *testing.T, nearAxial bool) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorTabSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := loftMirrorTabSection(t, world, middlePlane, 0.1)
	lower, err := decad.Loft(t.Context(), doc, bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, decad.MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	angle := units.Degrees(90)
	if nearAxial {
		angle = units.Radians(10 * 2 * math.Pi / 17)
	}
	quarterTurn, err := r3.Rotation(r3.Vec{Z: 1}, angle)
	require.NoError(t, err)
	turnedPair, err := decad.Copy(t.Context(), pair, decad.WithCopyPlacement(quarterTurn))
	require.NoError(t, err)
	blank := circleBodyAtZ(t, doc, 0, 3, 0, 2)
	if nearAxial {
		require.Equal(t, math.Nextafter(1, 0), quarterTurn.Basis().EZ.Z)
		joined, err := decad.Union(t.Context(), blank, pair, turnedPair)
		require.NoError(t, err)
		return doc, joined, blank
	}
	joined, err := decad.Union(t.Context(), blank, pair)
	require.NoError(t, err)
	joined, err = decad.Union(t.Context(), joined, turnedPair)
	require.NoError(t, err)
	return doc, joined, blank
}

func TestLoftMirrorCylinderBoreCutsRepeatedRadialTabs(t *testing.T) {
	doc, joined, blank := loftMirrorCylinderTwoTabs(t, false)
	before, err := joined.Volume()
	require.NoError(t, err)
	bore := circleBodyAtZ(t, doc, 0, 0.5, -1, 4)
	cut, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	after, err := cut.Volume()
	require.NoError(t, err)
	removed := math.Pi * 0.5 * 0.5 * 2
	require.LessOrEqual(t, math.Abs(before.Value.Base()-after.Value.Base()-removed),
		before.Bound.Base()+after.Bound.Base()+1e-10)
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, role := range []decad.FeatureRef{decad.CapStart(blank), decad.CapEnd(blank),
		decad.CapStart(cut), decad.CapEnd(cut)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(cut)
		require.NoError(t, err)
		require.Len(t, faces[0].Loops(), 2)
	}
	require.Len(t, doc.Bodies(), 1)
}

func TestLoftMirrorCylinderBoreRejectsToolEndingInsideUpperHalf(t *testing.T) {
	doc, joined, _ := loftMirrorCylinderTwoTabs(t, false)
	tool := circleBodyAtZ(t, doc, 0, 0.5, -1, 2.5)
	_, err := decad.Cut(t.Context(), joined, tool)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "tool ends inside the upper half")
	require.Contains(t, doc.Bodies(), joined)
}

func TestLoftMirrorCylinderBorePreservesNearAxialBridge(t *testing.T) {
	doc, joined, _ := loftMirrorCylinderTwoTabs(t, true)
	bore := circleBodyAtZ(t, doc, 0, 0.5, -1, 4)
	cut, err := decad.Cut(t.Context(), joined, bore)
	require.NoError(t, err)
	bottom, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(cut))).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	require.IsType(t, decad.Plane{}, bottom[0].Surface())
	top, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(cut))).Exactly(1).SelectFaces(cut)
	require.NoError(t, err)
	require.IsType(t, decad.Faceted{}, top[0].Surface())
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
