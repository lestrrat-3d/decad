package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func closedCircleSection(t *testing.T, minX, maxX float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(minX, -1, maxX, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func closedCirclePath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.NewVec(10, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(6, 0, 8), End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(-10, 0, 0), End: r3.NewVec(10, 0, 0)},
	)
	require.NoError(t, err)
	require.Equal(t, path.Start(), path.End())
	return path
}

func TestSweepClosedCircleBuildsVerifiedSolidAndReplaysPlacement(t *testing.T) {
	t.Parallel()
	s, profile := closedCircleSection(t, 9, 11)
	path := closedCirclePath(t)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), 4)
	require.Len(t, body.Shells(), 1)
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindCylinder: 2,
		decad.KindPlane:    2,
	})
	for _, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 2)
		require.True(t, strings.HasPrefix(origins[0].Role, "side(0,"))
		require.Equal(t, "side(1,"+origins[0].Role[len("side(0,"):], origins[1].Role)
		for _, origin := range origins {
			selected, err := decad.Faces(decad.FaceCreatedBy(origin)).Exactly(1).SelectFaces(body)
			require.NoError(t, err)
			require.Equal(t, face, selected[0])
		}
	}
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(body))).SelectFaces(body)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 80*math.Pi, volume.Value.Base(), 1e-10)
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(160*math.Pi))
	decadtest.MeasuresCentroid(t, body, r3.Vec{})
	decadtest.MeasuresBounds(t, body, r3.NewVec(-11, -1, -11), r3.NewVec(11, 1, 11))
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: -1}, End: decad.Point2{U: 0, V: 1}}
	reference, err := decad.New().Revolve(s, profile, axis, decad.FullRevolution{})
	require.NoError(t, err)
	refVolume, err := reference.Volume()
	require.NoError(t, err)
	require.Equal(t, refVolume, volume)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed())
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.NotEmpty(t, mesh.Triangles())
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, body.Faces(), face)
	}

	move, err := r3.Translation(r3.NewVec(3, 4, 5))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())
	for _, face := range placed.Faces() {
		require.Len(t, face.Origins(), 2)
	}
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, volume, placedVolume)
	placedReport, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, placedReport.Passed())
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestSweepClosedCircleRefusesUnsupportedVariantsWithoutCommit(t *testing.T) {
	t.Parallel()
	s, profile := closedCircleSection(t, 9, 11)
	full := closedCirclePath(t)
	fourArcs, err := decad.NewPath(r3.NewVec(10, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(6, 0, 8), End: r3.NewVec(0, 0, 10)},
		decad.ArcThrough{Through: r3.NewVec(-6, 0, 8), End: r3.NewVec(-10, 0, 0)},
		decad.ArcThrough{Through: r3.NewVec(-6, 0, -8), End: r3.NewVec(0, 0, -10)},
		decad.ArcThrough{Through: r3.NewVec(6, 0, -8), End: r3.NewVec(10, 0, 0)},
	)
	require.NoError(t, err)
	for _, test := range []struct {
		name string
		path *decad.Path
		opts []decad.SweepOption
	}{
		{name: "four tangent arcs", path: fourArcs},
		{name: "surface result", path: full, opts: []decad.SweepOption{decad.WithSurfaceResult()}},
		{name: "nonzero twist", path: full, opts: []decad.SweepOption{decad.WithSweepTwist(units.Degrees(10))}},
	} {
		t.Run(test.name, func(t *testing.T) {
			doc := decad.New()
			body, err := doc.Sweep(t.Context(), s, profile, test.path, test.opts...)
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, doc.Bodies())
		})
	}

	crossing, crossingProfile := closedCircleSection(t, -1, 1)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), crossing, crossingProfile, full)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())
}
