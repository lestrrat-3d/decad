package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The real Sketch rectangles pass through two Extrudes, a blind Cut, and
// Shell. The square floor's four concave vertices become sphere octants.
func TestShellBlindRectangularPocket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	outer := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), outer, tool)
	require.NoError(t, err)
	requireVolumeNear(t, pocket, 936)

	opening := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pocket))).Exactly(1)
	shell, err := pocket.Shell(t.Context(), opening, units.Millimeters(1))
	require.NoError(t, err)
	requireManifold(t, shell)
	require.Len(t, shell.Faces(), 34)
	want := 504 + 26*math.Pi/3
	requireVolumeNear(t, shell, want)
	var spheres, cylinders int
	for _, face := range shell.Faces() {
		switch face.Surface().Kind() {
		case decad.KindSphere:
			spheres++
		case decad.KindCylinder:
			cylinders++
		}
	}
	require.Equal(t, 4, spheres)
	require.Equal(t, 8, cylinders)

	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, want, meshVolume(mesh), 2)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)

	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := shell.Placed(t.Context(), rotation)
	require.NoError(t, err)
	requireVolumeNear(t, placed, want)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, placedMesh)
	require.True(t, placedMesh.VolumeVerified())
}

func TestShellBlindRectangularPocketClearance(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	outer := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, 2, 3, 6, 7, 6, 4)
	pocket, err := decad.Cut(t.Context(), outer, tool)
	require.NoError(t, err)
	_, err = pocket.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pocket))).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "clearance")
}

func TestShellBlindRectangularPocketOffsetMoments(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	outer := boxBodyAtZ(t, doc, -12, -10, 12, 10, 0, 12)
	tool := boxBodyAtZ(t, doc, -4, -1, 2, 3, 7, 5)
	pocket, err := decad.Cut(t.Context(), outer, tool)
	require.NoError(t, err)
	shell, err := pocket.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pocket))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	wantVolume := 1528 + 32*math.Pi/3
	requireVolumeNear(t, shell, wantVolume)
	centroid, err := shell.Centroid()
	require.NoError(t, err)
	wantX := -(124 + 32*math.Pi/3) / wantVolume
	wantZ := (22036.0/3 + 1043*math.Pi/12) / wantVolume
	require.LessOrEqual(t, math.Abs(centroid.Value.X-wantX), centroid.Bound.Base()+1e-12)
	require.LessOrEqual(t, math.Abs(centroid.Value.Y+wantX), centroid.Bound.Base()+1e-12)
	require.LessOrEqual(t, math.Abs(centroid.Value.Z-wantZ), centroid.Bound.Base()+1e-12)
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
}
