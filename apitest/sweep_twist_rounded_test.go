package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func roundedTwistProfile(t *testing.T, offsetX float64) (*sketch.Sketch, *sketch.Profile) {
	return roundedTwistProfileSized(t, offsetX, 4, 2, 0.5)
}

func roundedTwistProfileSized(t *testing.T, offsetX, width, height, radius float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x+offsetX, y)
		s.Fix(p)
		return p
	}
	left, right := -width/2, width/2
	bottom, top := -height/2, height/2
	ends := [8]*sketch.Point{
		point(left+radius, bottom), point(right-radius, bottom),
		point(right, bottom+radius), point(right, top-radius),
		point(right-radius, top), point(left+radius, top),
		point(left, top-radius), point(left, bottom+radius),
	}
	for i := 0; i < 8; i += 2 {
		s.CreateLine(ends[i], ends[(i+1)%8])
	}
	s.CreateArc(point(right-radius, bottom+radius), ends[1], ends[2])
	s.CreateArc(point(right-radius, top-radius), ends[3], ends[4])
	s.CreateArc(point(left+radius, top-radius), ends[5], ends[6])
	s.CreateArc(point(left+radius, bottom+radius), ends[7], ends[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 8 && len(candidate.Holes) == 0 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile)
	return s, profile
}

func TestSweepTwistRoundedProfileVerifyUsesHeldDiameter(t *testing.T) {
	s, profile := roundedTwistProfileSized(t, 0, 8, 6, 1.5)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	require.Len(t, report.Bodies, 1)
	row := report.Bodies[0]
	require.Equal(t, decad.ValidityValid, row.Validity.Outcome)
	require.Equal(t, decad.ToleranceExceeded, row.Area.Tolerance.State)
	require.Equal(t, decad.ToleranceExceeded, row.Bounds.Tolerance.State)
	require.NotNil(t, row.Region)
	require.Equal(t, decad.ToleranceSatisfied, row.Region.Volume.Tolerance.State)
	require.Len(t, row.Diagnostics, 2)
	require.Equal(t, decad.DiagMeasurementBeyondTolerance, row.Diagnostics[0].Code)
	require.Equal(t, decad.ReadingArea, row.Diagnostics[0].Reading)
	require.Equal(t, decad.DiagMeasurementBeyondTolerance, row.Diagnostics[1].Code)
	require.Equal(t, decad.ReadingBounds, row.Diagnostics[1].Reading)
}

// A solved Sketch supplies the mixed boundary; the real Sweep and tessellator
// must carry its circular corners through the public producer-consumer path.
func TestSweepTwistRoundedProfileVerifyAll(t *testing.T) {
	s, profile := roundedTwistProfile(t, 0)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 10)
	roles := map[string]struct{}{}
	for _, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		_, exists := roles[origins[0].Role]
		require.False(t, exists)
		roles[origins[0].Role] = struct{}{}
	}
	for edge := range 8 {
		require.Contains(t, roles, fmt.Sprintf("side(0,0,%d)", edge))
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-5*(7+math.Pi/4)), volume.Bound.Base()+1e-12)
	require.Less(t, volume.Bound.Base(), 1e-6)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 2.5), centroid.Value)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	require.Greater(t, len(mesh.Triangles()), 16)
	live := map[*decad.Face]struct{}{}
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())

	negative, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(-30)))
	require.NoError(t, err)
	negativeMesh, err := negative.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, negativeMesh.BoundaryVerified())
	require.True(t, negativeMesh.VolumeVerified())
}

func TestSweepTwistRoundedProfileAxisRefusalIsAtomic(t *testing.T) {
	s, profile := roundedTwistProfile(t, 1)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	_, err = decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}

func TestSweepTwistRoundedProfileUnionKeepsHeldProof(t *testing.T) {
	doc, swept, box := roundedTwistUnionOperands(t, 1, 2.5)
	joined, err := decad.Union(t.Context(), swept, box)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	volume, err := joined.Volume()
	require.NoError(t, err)
	// This checks the published bound against possible true occupancy. The
	// rounded section's x support stays below 2.2 at z=2..3, so at least
	// 0.24 mm³ of the box is exterior. Its x=1..1.2, y=-0.1..0.1 slab lies
	// inside the rotated 3×1 mm inner rectangle, so at least 0.04 mm³ overlaps.
	// The broad source bound cannot measure the exact overlap from these limits.
	sweptVolume := 5 * (7 + math.Pi/4)
	lower := volume.Value.Base() - volume.Bound.Base()
	upper := volume.Value.Base() + volume.Bound.Base()
	require.LessOrEqual(t, lower, sweptVolume+1.16)
	require.GreaterOrEqual(t, upper, sweptVolume+0.24)
	require.Less(t, volume.Bound.Base(), 10.0)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.2),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, mesh.Bound().Base(), 0.1)
	require.Less(t, mesh.Bound().Base(), 0.2)
	_, err = joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
}

func TestSweepTwistRoundedProfileUnionRefusesNearContact(t *testing.T) {
	doc, swept, box := roundedTwistUnionOperands(t, 2.1, 3.6)
	_, err := decad.Union(t.Context(), swept, box)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "held facets come within the chord tolerance")
	var booleanErr *decad.BooleanError
	require.ErrorAs(t, err, &booleanErr)
	require.Equal(t, decad.BooleanUnsupportedContact, booleanErr.Code)
	require.Len(t, doc.Bodies(), 2)
}

func roundedTwistUnionOperands(t *testing.T, boxMinX, boxMaxX float64) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	s, profile := roundedTwistProfile(t, 0)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	swept, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)

	boxWorld := sketch.NewWorld()
	boxSketch, err := boxWorld.CreateSketch(boxWorld.XY())
	require.NoError(t, err)
	boxProfile := boxSketch.CreateRectangle(boxMinX, -0.4, boxMaxX, 0.4)
	boxSketch.Fix(boxProfile.A)
	_, err = boxSketch.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), doc, boxSketch, boxSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0, 0, 2))
	require.NoError(t, err)
	box, err = decad.Transform(t.Context(), box, shift)
	require.NoError(t, err)

	return doc, swept, box
}
