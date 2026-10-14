package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExtrudeToFullRevolveAxialFace(t *testing.T) {
	t.Parallel()
	s, profile := solidSketch(t)
	doc := decad.New()
	host, err := decad.Revolve(t.Context(), doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	world := s.World()
	stopSketch, err := world.CreateSketch(world.YZ())
	require.NoError(t, err)
	rect := stopSketch.CreateRectangle(-1, -1, 1, 1)
	stopSketch.Fix(rect.A)
	_, err = stopSketch.Solve(t.Context())
	require.NoError(t, err)

	pin, err := decad.Extrude(t.Context(), doc, stopSketch, stopSketch.Profiles()[0], decad.ToFace{
		Body: host,
		Face: decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0))),
	})
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, pin, units.CubicMillimeters(40))
	mesh, err := pin.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestExtrudeToNearlyPlanarRevolveFaceRefuses(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(10, 8)
	d := s.CreatePoint(1e-10, 8)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	s.Fix(a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	host, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)

	plane, err := world.CreateOffsetPlane(world.YZ(), -1)
	require.NoError(t, err)
	stopSketch, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := stopSketch.CreateRectangle(-1, -1, 1, 1)
	stopSketch.Fix(rect.A)
	_, err = stopSketch.Solve(t.Context())
	require.NoError(t, err)
	_, err = decad.Extrude(t.Context(), doc, stopSketch, stopSketch.Profiles()[0], decad.ToFace{
		Body: host,
		Face: decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0))),
	})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "one plane")
}

func stopSketchBehindRevolveFace(t *testing.T, host *decad.Body, query *decad.FaceQuery) *sketch.Sketch {
	t.Helper()
	faces, err := query.SelectFaces(host)
	require.NoError(t, err)
	require.Len(t, faces, 1)
	plane, ok := faces[0].Surface().(decad.Plane)
	require.True(t, ok)
	world := sketch.NewWorld()
	base, err := world.CreatePlaneFromFrame(plane.Frame)
	require.NoError(t, err)
	behind, err := world.CreateOffsetPlane(base, -1)
	require.NoError(t, err)
	s, err := world.CreateSketch(behind)
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s
}

func TestExtrudeToRevolveFaceWithUncertainAxisRefuses(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0.1, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	// The distant axis anchor gives the recorded axial level a nonzero
	// interval even though the displayed end face still looks planar.
	host, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.ConstructionAxis{
		Origin: r3.NewVec(1e9, 0, 0), Dir: r3.NewVec(1, 0, 0),
	}, decad.FullRevolution{})
	require.NoError(t, err)
	query := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))
	stopSketch := stopSketchBehindRevolveFace(t, host, query)
	_, err = decad.Extrude(t.Context(), doc, stopSketch, stopSketch.Profiles()[0], decad.ToFace{
		Body: host, Face: query,
	})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "one plane")
}

func TestExtrudeToPlacedRevolveFaceWithTiltedRecordRefuses(t *testing.T) {
	t.Parallel()
	s, profile := solidSketch(t)
	doc := decad.New()
	original, err := decad.Revolve(t.Context(), doc, s, profile, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	// At this placement the displayed face normal passes the held-normal
	// check, while its recorded plane is tilted against that same direction.
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(41))
	require.NoError(t, err)
	rotated, err := decad.Transform(t.Context(), original, turn)
	require.NoError(t, err)
	var farFrame r3.Frame
	for _, face := range rotated.Faces() {
		plane, ok := face.Surface().(decad.Plane)
		if !ok {
			continue
		}
		if plane.Frame.N().X > 0 {
			farFrame = plane.Frame
		}
	}
	require.NotEqual(t, r3.Frame{}, farFrame)
	query := decad.Faces(decad.Planar(), decad.Facing(farFrame.N()))
	stopSketch := stopSketchBehindRevolveFace(t, rotated, query)
	_, err = decad.Extrude(t.Context(), doc, stopSketch, stopSketch.Profiles()[0], decad.ToFace{
		Body: rotated, Face: query,
	})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "one plane")
}
