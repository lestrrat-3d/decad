package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepArcMatchesQuarterRevolveAndReplaysPlacement(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.NewVec(0, 5, 0),
		decad.ArcThrough{
			Through: r3.NewVec(0, 3, 4),
			End:     r3.NewVec(0, 0, 5),
		},
	)
	require.NoError(t, err)

	sweepDoc := decad.New()
	swept, err := sweepDoc.Sweep(s, profile, path)
	require.NoError(t, err)
	revolveDoc := decad.New()
	revolved, err := revolveDoc.Revolve(
		s,
		profile,
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 100, V: 0}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along},
	)
	require.NoError(t, err)
	requireSameBodyReadings(t, revolved, swept)

	roles := make([]string, 0, len(swept.Faces()))
	for _, face := range swept.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		roles = append(roles, origins[0].Role)
	}
	require.ElementsMatch(t, []string{
		"capStart",
		"capEnd",
		"side(0,0,1)",
		"side(0,0,2)",
		"side(0,0,3)",
	}, roles)
	for _, edge := range swept.Edges() {
		require.Len(t, edge.Faces(), 2)
	}

	report, err := sweepDoc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Equal(t, decad.Sound, report.Status)

	motion, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placedSweep, err := swept.Placed(motion)
	require.NoError(t, err)
	placedRevolve, err := revolved.Placed(motion)
	require.NoError(t, err)
	requireSameBodyReadings(t, placedRevolve, placedSweep)

	report, err = sweepDoc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Equal(t, decad.Sound, report.Status)
}

func TestSweepArcUsesPathEndpointCapRolesWhenAxisGateFlips(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.NewVec(20, 110, 0),
		decad.ArcThrough{
			Through: r3.NewVec(20, 106, 8),
			End:     r3.NewVec(20, 100, 10),
		},
	)
	require.NoError(t, err)
	body, err := decad.New().Sweep(s, profile, path)
	require.NoError(t, err)

	startCap := faceByRole(t, body, "capStart")
	for _, edge := range startCap.Edges() {
		require.Zero(t, edge.Start().Position().Value.Z)
		require.Zero(t, edge.End().Position().Value.Z)
	}
	endCap := faceByRole(t, body, "capEnd")
	for _, edge := range endCap.Edges() {
		require.Equal(t, 100.0, edge.Start().Position().Value.Y)
		require.Equal(t, 100.0, edge.End().Position().Value.Y)
	}
}

func TestSweepArcBuildsGeneralAngleWithRationalCenter(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	// These dyadic input points lie on a circle whose exact center is
	// (0, -25/3, 0). The directed angle is not a quadrantal turn.
	path, err := decad.NewPath(
		r3.NewVec(0, -10, 0),
		decad.ArcThrough{
			Through: r3.NewVec(0, -7, 1),
			End:     r3.NewVec(0, -7, -1),
		},
	)
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Sweep(s, profile, path)
	require.NoError(t, err)
	require.True(t, body.IsSolid())

	volume, err := body.Volume()
	require.NoError(t, err)
	theta := math.Pi + math.Atan(3.0/4.0)
	wantVolume := 230000 * theta
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base())
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.True(t, volume.Bound.Base() > 0)
	area, err := body.Area()
	require.NoError(t, err)
	wantArea := 12000 + (36800.0/3.0)*theta
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base())
	require.True(t, area.Bound.Base() > 0)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	require.True(t, centroid.Bound.Base() > 0)
	bounds, err := body.Bounds()
	require.NoError(t, err)
	require.True(t, bounds.Bound.Base() > 0)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	require.Equal(t, decad.Sound, report.Status)
}

func TestSweepArcMatchesRevolveFromObliqueSketchPlane(t *testing.T) {
	t.Parallel()

	w := sketch.NewWorld()
	frame, err := r3.NewFrame(
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 3, 4),
	)
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 10, 10, 6)
	s.Fix(rect.A)
	_, err = s.Solve(context.Background())
	require.NoError(t, err)

	path, err := decad.NewPath(
		frame.V(),
		decad.ArcThrough{
			Through: frame.N(),
			End:     frame.V().Scale(-1),
		},
	)
	require.NoError(t, err)

	swept, err := decad.New().Sweep(s, s.Profiles()[0], path)
	require.NoError(t, err)
	revolved, err := decad.New().Revolve(
		s,
		s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: -10, V: 0}, End: decad.Point2{U: 10, V: 0}},
		decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along},
	)
	require.NoError(t, err)
	requireSameHeldBodyReadingsWithBounds(t, revolved, swept)
}

func TestSweepArcRefusalsLeaveDocumentUnchanged(t *testing.T) {
	t.Parallel()

	s, profile := plateSketch(t)
	tests := []struct {
		name    string
		start   r3.Vec
		through r3.Vec
		end     r3.Vec
		want    error
	}{
		{
			name:    "axis crosses profile",
			start:   r3.NewVec(20, 40, 0),
			through: r3.NewVec(20, 36, 8),
			end:     r3.NewVec(20, 30, 10),
			want:    decad.ErrDegenerate,
		},
		{
			name:    "reversed initial tangent",
			start:   r3.NewVec(20, 10, 0),
			through: r3.NewVec(20, 6, -8),
			end:     r3.NewVec(20, 0, -10),
			want:    decad.ErrDegenerate,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			path, err := decad.NewPath(
				test.start,
				decad.ArcThrough{Through: test.through, End: test.end},
			)
			require.NoError(t, err)
			doc := decad.New()
			_, err = doc.Sweep(s, profile, path)
			require.ErrorIs(t, err, test.want)
			require.Empty(t, doc.Bodies())
		})
	}
}

func requireSameBodyReadings(t *testing.T, want, got *decad.Body) {
	t.Helper()
	wantVolume, err := want.Volume()
	require.NoError(t, err)
	gotVolume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, wantVolume, gotVolume)
	wantArea, err := want.Area()
	require.NoError(t, err)
	gotArea, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, wantArea, gotArea)
	wantCentroid, err := want.Centroid()
	require.NoError(t, err)
	gotCentroid, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, wantCentroid, gotCentroid)
	wantBounds, err := want.Bounds()
	require.NoError(t, err)
	gotBounds, err := got.Bounds()
	require.NoError(t, err)
	require.Equal(t, wantBounds, gotBounds)
}

func requireSameHeldBodyReadingsWithBounds(t *testing.T, want, got *decad.Body) {
	t.Helper()
	wantVolume, err := want.Volume()
	require.NoError(t, err)
	gotVolume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, wantVolume.Value, gotVolume.Value)
	require.Equal(t, decad.Approximate, gotVolume.Exactness)
	require.Positive(t, gotVolume.Bound.Base())

	wantArea, err := want.Area()
	require.NoError(t, err)
	gotArea, err := got.Area()
	require.NoError(t, err)
	require.Equal(t, wantArea.Value, gotArea.Value)
	require.Equal(t, decad.Approximate, gotArea.Exactness)
	require.Positive(t, gotArea.Bound.Base())

	wantCentroid, err := want.Centroid()
	require.NoError(t, err)
	gotCentroid, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, wantCentroid.Value, gotCentroid.Value)
	require.Equal(t, decad.Approximate, gotCentroid.Exactness)
	require.Positive(t, gotCentroid.Bound.Base())

	wantBounds, err := want.Bounds()
	require.NoError(t, err)
	gotBounds, err := got.Bounds()
	require.NoError(t, err)
	require.Equal(t, wantBounds.Min, gotBounds.Min)
	require.Equal(t, wantBounds.Max, gotBounds.Max)
	require.Equal(t, decad.Approximate, gotBounds.Exactness)
	require.Positive(t, gotBounds.Bound.Base())
}
