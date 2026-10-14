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
