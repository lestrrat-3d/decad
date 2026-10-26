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

	shelled, err := decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
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
	moved, err := decad.Transform(t.Context(), shelled, motion)
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
			_, err = decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
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
	shell, err := decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stack))).Exactly(1),
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

// The rectangular boss shell also opens at the plate bottom. Its volume is
// the closed arm's volume less the lower inset's area times t, and its
// bottom is one planar annulus between the plate outline and that inset.
func TestShellStackedBossOpenAtBottom(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stack, err := decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 5),
		boxBodyAtZ(t, doc, -5, -5, 5, 5, 5, 5))
	require.NoError(t, err)
	shell, err := decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapStart(stack))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	v, err := shell.Volume()
	require.NoError(t, err)
	// Plate 20x20x5 plus boss 10x10x5, less the 18x18x4 lower cavity (floor
	// open), the 8x8x4 upper cavity under the kept roof, and the root band
	// 20/3 + 8*pi + 60.
	want := 2500.0 - 18*18*4 - 8*8*4 - (60 + 20.0/3 + 8*math.Pi)
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base())
	// Moment about z=0: plate 2000 at 2.5, boss 500 at 7.5, lower cavity
	// 1296 at 2, upper cavity 256 at 7, and the band at its ledge level 4
	// plus its own offset 43 + 2/3.
	bandV := 60 + 20.0/3 + 8*math.Pi
	mz := 5000.0 + 3750 - 1296*2 - 256*7 - (4*bandV + 131.0/3)
	c, err := shell.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(c.Value.Z-mz/want), 1e-12+c.Bound.Base())
	require.InDelta(t, 0, c.Value.X, 1e-12)
	require.InDelta(t, 0, c.Value.Y, 1e-12)

	var bottoms int
	for _, face := range shell.Faces() {
		pl, ok := face.Surface().(decad.Plane)
		if !ok || math.Abs(pl.Frame.Origin().Z) > 1e-9 {
			continue
		}
		n, err := face.NormalAt(pl.Frame.Origin())
		require.NoError(t, err)
		if n.Value.Z < 0 {
			bottoms++
			a, err := face.Area()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(a.Value.Base()-(400-324)), a.Bound.Base())
		}
	}
	require.Equal(t, 1, bottoms)
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(mesh)-v.Value.Base()), 1.0+v.Bound.Base())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

// The off-centre boss makes the lower cavity's U and V moments differ from
// the boss's, so a wrong lower-cavity weight shows in the centroid.
func TestShellStackedBossOpenAtBottomOffCentre(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stack, err := decad.Union(t.Context(), boxBody(t, doc, -12, -10, 8, 10, 6),
		boxBodyAtZ(t, doc, -2, -4, 6, 2, 6, 4))
	require.NoError(t, err)
	shell, err := decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapStart(stack))).Exactly(1),
		units.Millimeters(1))
	require.NoError(t, err)
	v, err := shell.Volume()
	require.NoError(t, err)
	// Plate 20x20x6 centred at U=-2, boss 8x6x4, lower cavity 18x18x5 (floor open), upper
	// cavity 6x4x3 under the kept roof, root band 80/3 + 5*pi.
	bandV := 80.0/3 + 5*math.Pi
	want := 2400.0 + 192 - 1620 - 72 - bandV
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base())
	c, err := shell.Centroid()
	require.NoError(t, err)
	tol := 1e-12 + c.Bound.Base()
	require.LessOrEqual(t, math.Abs(c.Value.X-(-1320-2*bandV)/want), tol)
	require.LessOrEqual(t, math.Abs(c.Value.Y-(-120+bandV)/want), tol)
	mz := 7200.0 + 1536 - 1620*2.5 - 72*7.5 - (5*bandV + 59.0/3)
	require.LessOrEqual(t, math.Abs(c.Value.Z-mz/want), tol)
	mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, math.Abs(shellMeshSignedVolume(mesh)-v.Value.Base()), 1.0+v.Bound.Base())
}

func TestShellStackedBossOpenAtBottomRefusesCollapsedCavity(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                             string
		plateHeight, bossHeight, bossOff float64
	}{
		{name: "floor", plateHeight: 2, bossHeight: 5, bossOff: 5},
		{name: "roof", plateHeight: 5, bossHeight: 1, bossOff: 5},
		{name: "root clearance", plateHeight: 5, bossHeight: 5, bossOff: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stack, err := decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, tc.plateHeight),
				boxBodyAtZ(t, doc, -10+tc.bossOff, -10+tc.bossOff, 10-tc.bossOff, 10-tc.bossOff,
					tc.plateHeight, tc.bossHeight))
			require.NoError(t, err)
			shell, err := decad.Shell(t.Context(), stack,
				decad.Faces(decad.FaceCreatedBy(decad.CapStart(stack))).Exactly(1), units.Millimeters(1))
			require.Nil(t, shell)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = stack.Volume()
			require.NoError(t, err, "a refused shell leaves its source live")
		})
	}
}

// Each of the other stacked shells opens only at the cap its shape names.
// Removing the other cap must not build that shell: the stack falls through to
// route S, which reads no through-cut and refuses.
func TestShellStackedWrongCapRefuses(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		build func(t *testing.T, doc *decad.Document) *decad.Body
	}{
		{name: "round boss", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			return roundBossFromSketch(t, doc, 10, 10)
		}},
		{name: "blind pocket", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			pocket, err := decad.Cut(t.Context(), boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10),
				boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4))
			require.NoError(t, err)
			return pocket
		}},
		{name: "counterbore", build: func(t *testing.T, doc *decad.Document) *decad.Body {
			part, err := decad.Cut(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 0, 3, 0, 10))
			require.NoError(t, err)
			part, err = decad.Cut(t.Context(), part, circleBodyAtZ(t, doc, 0, 5, 7, 3))
			require.NoError(t, err)
			return part
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			stack := tc.build(t, decad.New())
			_, err := decad.Shell(t.Context(), stack, decad.Faces(decad.FaceCreatedBy(decad.CapStart(stack))).Exactly(1),
				units.Millimeters(1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = stack.Volume()
			require.NoError(t, err, "a refused shell leaves its source live")
		})
	}
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
