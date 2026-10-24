package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func twistedShellReceiver(t *testing.T, angle units.Value) (*decad.Document, *decad.Body) {
	t.Helper()
	s, profile := roundedLargeTwistProfile(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, path, decad.WithSweepTwist(angle))
	require.NoError(t, err)
	return doc, body
}

func TestTwistedSweepRoundedShellVerifyAll(t *testing.T) {
	doc, swept := twistedShellReceiver(t, units.Degrees(30))
	cup, err := decad.Shell(t.Context(), swept, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(swept))).Exactly(1),
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
			_, err := decad.Shell(t.Context(), swept, start, units.Millimeters(1))
			return err
		},
		func() error {
			_, err := decad.Shell(t.Context(), swept, end, units.Millimeters(1),
				decad.WithShellSense(decad.Outward))
			return err
		},
		func() error {
			_, err := decad.Shell(t.Context(), swept, end, units.Millimeters(3))
			return err
		},
	} {
		require.ErrorIs(t, call(), decad.ErrUnsupported)
		require.Equal(t, []*decad.Body{swept}, doc.Bodies())
	}
}

func twistedShellPlacement(t *testing.T) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(23))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(9, -4, 7))
	require.NoError(t, err)
	motion, err := turn.Then(shift)
	require.NoError(t, err)
	return motion
}

func TestTwistedSweepRoundedPlacedShellVerifyAll(t *testing.T) {
	baselineDoc, baselineSweep := twistedShellReceiver(t, units.Degrees(30))
	baseline, err := decad.Shell(t.Context(), baselineSweep,
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(baselineSweep))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{baseline}, baselineDoc.Bodies())
	baselineVolume, err := baseline.Volume()
	require.NoError(t, err)
	baselineMesh, err := baseline.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)

	for _, tc := range []struct {
		name  string
		place func(*decad.Body, r3.Transform) (*decad.Body, error)
		live  int
	}{
		{name: "copy placement", place: func(source *decad.Body, motion r3.Transform) (*decad.Body, error) {
			return decad.Copy(t.Context(), source, decad.WithCopyPlacement(motion))
		}, live: 2},
		{name: "transform placement", place: func(source *decad.Body, motion r3.Transform) (*decad.Body, error) {
			return decad.Transform(t.Context(), source, motion)
		}, live: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := twistedShellReceiver(t, units.Degrees(30))
			placed, err := tc.place(source, twistedShellPlacement(t))
			require.NoError(t, err)
			end := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(placed))).Exactly(1)
			selected, err := end.SelectFaces(placed)
			require.NoError(t, err)
			require.Len(t, selected, 1)
			cup, err := decad.Shell(t.Context(), placed, end, units.Millimeters(1))
			require.NoError(t, err)
			require.Len(t, doc.Bodies(), tc.live)
			require.Contains(t, doc.Bodies(), cup)
			if tc.live == 2 {
				require.Contains(t, doc.Bodies(), source)
			}
			require.Len(t, cup.Faces(), 5)
			require.Len(t, cup.Lumps(), 1)
			volume, err := cup.Volume()
			require.NoError(t, err)
			require.InDelta(t, baselineVolume.Value.Base(), volume.Value.Base(), 1e-9)
			require.GreaterOrEqual(t, volume.Bound.Base(), baselineVolume.Bound.Base())
			require.InDelta(t, baselineVolume.Bound.Base(), volume.Bound.Base(), 1e-8)
			mesh, err := cup.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Len(t, mesh.Triangles(), len(baselineMesh.Triangles()))
			require.InDelta(t, baselineMesh.Bound().Base(), mesh.Bound().Base(), 1e-8)
		})
	}
}

func TestTwistedSweepRoundedPlacedShellRefusalsAreAtomic(t *testing.T) {
	doc, source := twistedShellReceiver(t, units.Degrees(30))
	placed, err := decad.Copy(t.Context(), source,
		decad.WithCopyPlacement(twistedShellPlacement(t)))
	require.NoError(t, err)
	before := doc.Bodies()
	start := decad.Faces(decad.FaceCreatedBy(decad.CapStart(placed))).Exactly(1)
	end := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(placed))).Exactly(1)
	for _, call := range []func() error{
		func() error {
			_, err := decad.Shell(t.Context(), placed, start, units.Millimeters(1))
			return err
		},
		func() error {
			_, err := decad.Shell(t.Context(), placed, end, units.Millimeters(1),
				decad.WithShellSense(decad.Outward))
			return err
		},
		func() error {
			_, err := decad.Shell(t.Context(), placed, end, units.Millimeters(3))
			return err
		},
	} {
		require.ErrorIs(t, call(), decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	}
}
