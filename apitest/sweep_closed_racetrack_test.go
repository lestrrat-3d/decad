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

func racetrackSection(t *testing.T, x, radius float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(x, 0)
	s.CreateCircle(center, radius)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func racetrackPath(t *testing.T, length, major float64) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, length)},
		decad.ArcThrough{Through: r3.NewVec(major, 0, length+major),
			End: r3.NewVec(2*major, 0, length)},
		decad.LineTo{End: r3.NewVec(2*major, 0, 0)},
		decad.ArcThrough{Through: r3.NewVec(major, 0, -major), End: r3.Vec{}},
	)
	require.NoError(t, err)
	return path
}

func TestSweepClosedRacetrackBuildsVerifiedTubeAndReplaysPlacement(t *testing.T) {
	t.Parallel()
	const radius, length, major = 1.0, 5.0, 5.0
	s, profile := racetrackSection(t, 0, radius)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, racetrackPath(t, length, major))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), 4)
	require.Len(t, body.Edges(), 4)
	require.Len(t, body.Shells(), 1)
	for i, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		require.Equal(t, fmt.Sprintf("side(%d,0,0)", i), origins[0].Role)
		for _, edge := range face.Edges() {
			require.Len(t, edge.Faces(), 2)
		}
	}
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	pathLength := 2*length + 2*math.Pi*major
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, math.Pi*radius*radius*pathLength, volume.Value.Base(), 1e-10+volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	require.InDelta(t, 2*math.Pi*radius*pathLength, area.Value.Base(), 1e-10+area.Bound.Base())
	centroid, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, major, centroid.Value.X, 1e-10+centroid.Bound.Base())
	require.InDelta(t, 0, centroid.Value.Y, 1e-10+centroid.Bound.Base())
	require.InDelta(t, length/2, centroid.Value.Z, 1e-10+centroid.Bound.Base())
	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -1, bounds.Min.X, 1e-10+bounds.Bound.Base())
	require.InDelta(t, 11, bounds.Max.X, 1e-10+bounds.Bound.Base())
	require.InDelta(t, -6, bounds.Min.Z, 1e-10+bounds.Bound.Base())
	require.InDelta(t, 11, bounds.Max.Z, 1e-10+bounds.Bound.Base())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	for _, source := range mesh.SourceFaces() {
		require.Contains(t, body.Faces(), source)
	}

	translation, err := r3.Translation(r3.NewVec(3, 4, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, translation)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	require.InDelta(t, volume.Value.Base(), placedVolume.Value.Base(),
		volume.Bound.Base()+placedVolume.Bound.Base()+1e-10)
	rotation, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	turned, err := decad.Transform(t.Context(), placed, rotation)
	require.NoError(t, err)
	turnedMesh, err := turned.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, turnedMesh.BoundaryVerified())
	require.True(t, turnedMesh.VolumeVerified())
	turnedCentroid, err := turned.Centroid()
	require.NoError(t, err)
	wantCentroid := rotation.Apply(translation.Apply(r3.NewVec(major, 0, length/2)))
	require.InDelta(t, wantCentroid.X, turnedCentroid.Value.X, 1e-10+turnedCentroid.Bound.Base())
	require.InDelta(t, wantCentroid.Y, turnedCentroid.Value.Y, 1e-10+turnedCentroid.Bound.Base())
	require.InDelta(t, wantCentroid.Z, turnedCentroid.Value.Z, 1e-10+turnedCentroid.Bound.Base())
}

func TestSweepClosedRacetrackRefusesAxisContact(t *testing.T) {
	t.Parallel()
	s, profile := racetrackSection(t, 0, 1)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, racetrackPath(t, 5, 1))
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "touching the revolve axis")
	require.Empty(t, doc.Bodies())
}

// A short straight leg and an off-centre section are admitted when the
// cyclic contact audit proves the result simple. Pappus fixes both volumes.
func TestSweepClosedRacetrackAdmitsShortLegsAndOffCentreSections(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                     string
		x, radius, length, major float64
		pathLength               float64
	}{
		{name: "short legs", radius: 1, length: 2, major: 5, pathLength: 2*2 + 2*math.Pi*5},
		// The centroid sits 0.25 toward each arc axis, so both half turns
		// sweep it at radius major-0.25.
		{name: "off-centre section", x: 0.25, radius: 1, length: 5, major: 5,
			pathLength: 2*5 + 2*math.Pi*(5-0.25)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, profile := racetrackSection(t, tc.x, tc.radius)
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, racetrackPath(t, tc.length, tc.major))
			require.NoError(t, err)
			volume, err := body.Volume()
			require.NoError(t, err)
			require.InDelta(t, math.Pi*tc.radius*tc.radius*tc.pathLength, volume.Value.Base(),
				1e-9+volume.Bound.Base())
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.True(t, report.Passed())
		})
	}
}
