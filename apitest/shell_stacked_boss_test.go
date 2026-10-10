package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestShellCenteredStackedBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 5)
	boss := boxBodyAtZ(t, doc, -5, -5, 5, 5, 5, 5)
	stack, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	require.Len(t, stack.Faces(), 11)

	shelled, err := stack.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	volume, err := shelled.Volume()
	require.NoError(t, err)
	want := 3424.0/3 - 8*math.Pi
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())

	var cylinders, ellipses int
	for _, face := range shelled.Faces() {
		if face.Surface().Kind() == decad.KindCylinder {
			cylinders++
		}
	}
	for _, edge := range shelled.Edges() {
		if _, ok := edge.Curve().(decad.Ellipse3); ok {
			ellipses++
		}
	}
	require.Equal(t, 4, cylinders)
	require.Equal(t, 4, ellipses)
	mesh, err := shelled.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(mesh)-volume.Value.Base()),
		1.0+volume.Bound.Base())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	motion, err := r3.Translation(r3.NewVec(12, -7, 3))
	require.NoError(t, err)
	moved, err := shelled.Placed(t.Context(), motion)
	require.NoError(t, err)
	placedVolume, err := moved.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(placedVolume.Value.Base()-want), placedVolume.Bound.Base())
	placedMesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.VolumeVerified())
	report, err = doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestShellStackedBossClearanceRefusal(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name           string
		bossX0, bossX1 float64
		thickness      float64
	}{
		{name: "floor", bossX0: -5, bossX1: 5, thickness: 2.5},
		{name: "side clearance", bossX0: -9, bossX1: 1, thickness: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 5)
			boss := boxBodyAtZ(t, doc, tc.bossX0, -5, tc.bossX1, 5, 5, 5)
			stack, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			_, err = stack.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
				units.Millimeters(tc.thickness))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = stack.Volume()
			require.NoError(t, err, "a refused shell leaves its source live")
		})
	}
}

func TestShellOffsetRectangularBoss(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -12, -10, 12, 10, 6)
	boss := boxBodyAtZ(t, doc, -2, -4, 6, 2, 6, 4)
	stack, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	shell, err := stack.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	v, err := shell.Volume()
	require.NoError(t, err)
	want := 4096.0/3 - 5*math.Pi
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base())
	c, err := shell.Centroid()
	require.NoError(t, err)
	remainingBoss := 96 - (80.0/3 + 5*math.Pi)
	require.LessOrEqual(t, math.Abs(c.Value.X-2*remainingBoss/want), c.Bound.Base())
	require.LessOrEqual(t, math.Abs(c.Value.Y+remainingBoss/want), c.Bound.Base())
	require.InDelta(t, (4503-25*math.Pi)/want, c.Value.Z, 1e-12)
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(mesh)-v.Value.Base()),
		1.0+v.Bound.Base())
	tool := boxBodyAtZ(t, doc, 11.5, -1, 14, 1, -1, 12)
	cut, err := decad.Cut(t.Context(), shell, tool)
	require.NoError(t, err)
	cutVolume, err := cut.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(cutVolume.Value.Base()-(want-6)), cutVolume.Bound.Base())
}

// shellMeshSignedVolume is the divergence sum of the public mesh's facets.
// The 0.05 mm fixture must approximate the analytic volume closely enough
// to catch a reversed band-mass formula even if VerifyAll reports success.
func shellMeshSignedVolume(mesh *decad.Mesh) float64 {
	verts := mesh.Vertices()
	volume := 0.0
	for _, tri := range mesh.Triangles() {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		volume += (a.X*(b.Y*c.Z-b.Z*c.Y) + a.Y*(b.Z*c.X-b.X*c.Z) + a.Z*(b.X*c.Y-b.Y*c.X)) / 6
	}
	return volume
}
