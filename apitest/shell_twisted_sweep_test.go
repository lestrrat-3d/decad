package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func twistedShellRoundedSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	ends := [8]*sketch.Point{
		point(-2.5, -3), point(2.5, -3), point(4, -1.5), point(4, 1.5),
		point(2.5, 3), point(-2.5, 3), point(-4, 1.5), point(-4, -1.5),
	}
	s.CreateLine(ends[0], ends[1])
	s.CreateArc(point(2.5, -1.5), ends[1], ends[2])
	s.CreateLine(ends[2], ends[3])
	s.CreateArc(point(2.5, 1.5), ends[3], ends[4])
	s.CreateLine(ends[4], ends[5])
	s.CreateArc(point(-2.5, 1.5), ends[5], ends[6])
	s.CreateLine(ends[6], ends[7])
	s.CreateArc(point(-2.5, -1.5), ends[7], ends[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Outer) == 8 && len(p.Holes) == 0 {
			return s, p
		}
	}
	t.Fatal("Sketch did not produce the rounded section")
	return nil, nil
}

func twistedShellReceiver(t *testing.T, angle units.Value) (*decad.Document, *decad.Body) {
	t.Helper()
	s, profile := twistedShellRoundedSketch(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, path, decad.WithSweepTwist(angle))
	require.NoError(t, err)
	return doc, body
}

func TestTwistedSweepRoundedShellVerifyAll(t *testing.T) {
	doc, swept := twistedShellReceiver(t, units.Degrees(30))
	cup, err := swept.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(swept))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{cup}, doc.Bodies())
	require.Len(t, cup.Lumps(), 1)
	require.Len(t, cup.Faces(), 5)
	volume, err := cup.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	require.Less(t, volume.Bound.Base(), 20.0)
	mesh, err := cup.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	require.Len(t, mesh.Triangles(), 3456)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	_, err = cup.Tessellate(t.Context(), units.Millimeters(0.01),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestTwistedSweepRoundedShellUnsupportedCallsAreAtomic(t *testing.T) {
	doc, swept := twistedShellReceiver(t, units.Degrees(30))
	end := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(swept))).Exactly(1)
	start := decad.Faces(decad.FaceCreatedBy(decad.CapStart(swept))).Exactly(1)
	for _, call := range []func() error{
		func() error {
			_, err := swept.Shell(t.Context(), start, units.Millimeters(1))
			return err
		},
		func() error {
			_, err := swept.Shell(t.Context(), end, units.Millimeters(1),
				decad.WithShellSense(decad.Outward))
			return err
		},
		func() error {
			_, err := swept.Shell(t.Context(), end, units.Millimeters(3))
			return err
		},
	} {
		require.ErrorIs(t, call(), decad.ErrUnsupported)
		require.Equal(t, []*decad.Body{swept}, doc.Bodies())
	}
}
