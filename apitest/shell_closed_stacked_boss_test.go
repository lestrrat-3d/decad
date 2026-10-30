package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func sketchBossSlab(t *testing.T, doc *decad.Document, world *sketch.World,
	z, u0, v0, u1, v1, height float64) *decad.Body {
	return sketchBossSlabOnPlane(t, doc, world, world.XY(), z, u0, v0, u1, v1, height)
}

func sketchBossSlabOnPlane(t *testing.T, doc *decad.Document, world *sketch.World, base *sketch.Plane,
	z, u0, v0, u1, v1, height float64) *decad.Body {
	t.Helper()
	plane, err := world.CreateOffsetPlane(base, z)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestClosedShellOfNestedRectangularSlabs(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	world := sketch.NewWorld()
	plate := sketchBossSlab(t, doc, world, 0, 0, 0, 20, 20, 5)
	boss := sketchBossSlab(t, doc, world, 5, 5, 5, 15, 15, 5)
	joined, err := decad.Union(t.Context(), plate, boss)
	require.NoError(t, err)
	sourceVolume, err := joined.Volume()
	require.NoError(t, err)
	require.Equal(t, 2500.0, sourceVolume.Value.Base())
	require.Zero(t, sourceVolume.Bound.Base())

	closed, err := decad.Shell(t.Context(), joined, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	require.Len(t, closed.Shells(), 2)
	require.False(t, closed.Shells()[0].IsVoid())
	require.True(t, closed.Shells()[1].IsVoid())
	volume, err := closed.Volume()
	require.NoError(t, err)
	// Source less an 18×18×3 lower cavity, an 8×8×4 upper cavity,
	// and the quarter-cylinder transition's exact integrated volume
	// wdt - (w+d)πt²/2 + 8t³/3.
	want := 2500 - 18*18*3 - 8*8*4 - (308.0/3 - 10*math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	for _, edge := range closed.Edges() {
		require.Len(t, edge.Faces(), 2)
	}
	for _, face := range closed.Faces() {
		require.NotEmpty(t, face.Origins())
		selected, err := decad.Faces(decad.FaceCreatedBy(face.Origins()[0])).Exactly(1).SelectFaces(closed)
		require.NoError(t, err)
		require.Same(t, face, selected[0])
	}

	shift, err := r3.Translation(r3.NewVec(3, -2, 4))
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), closed, shift)
	require.NoError(t, err)
	require.Len(t, moved.Shells(), 2)
	require.True(t, moved.Shells()[1].IsVoid())
	movedVolume, err := moved.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(movedVolume.Value.Base()-want), movedVolume.Bound.Base())
	movedMesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, movedMesh.BoundaryVerified())
	require.True(t, movedMesh.VolumeVerified())
	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirror)
	require.NoError(t, err)
	reflected, err := decad.Transform(t.Context(), moved, reflection)
	require.NoError(t, err)
	require.Len(t, reflected.Shells(), 2)
	require.True(t, reflected.Shells()[1].IsVoid())
	reflectedMesh, err := reflected.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, reflectedMesh.BoundaryVerified())
	require.True(t, reflectedMesh.VolumeVerified())
}

func TestClosedShellOfNestedRectangularSlabsRefusesCollapsedCavity(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                    string
		plateHeight, bossHeight float64
		bossU0                  float64
	}{
		{name: "floor", plateHeight: 2, bossHeight: 5, bossU0: 5},
		{name: "roof", plateHeight: 5, bossHeight: 1, bossU0: 5},
		{name: "root clearance", plateHeight: 5, bossHeight: 5, bossU0: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			world := sketch.NewWorld()
			plate := sketchBossSlab(t, doc, world, 0, 0, 0, 20, 20, tc.plateHeight)
			boss := sketchBossSlab(t, doc, world, tc.plateHeight,
				tc.bossU0, 5, tc.bossU0+10, 15, tc.bossHeight)
			joined, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			closed, err := decad.Shell(t.Context(), joined, nil, units.Millimeters(1), decad.WithNoOpenings())
			require.Nil(t, closed)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = joined.Volume()
			require.NoError(t, err)
		})
	}
}

func TestClosedShellOfNestedRectangularSlabsSwappedAxes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                         string
		plateU, plateV, bossU, bossV float64
	}{
		{name: "wide u", plateU: 24, plateV: 18, bossU: 12, bossV: 6},
		{name: "wide v", plateU: 18, plateV: 24, bossU: 6, bossV: 12},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			world := sketch.NewWorld()
			plate := sketchBossSlab(t, doc, world, 0,
				-tc.plateU/2, -tc.plateV/2, tc.plateU/2, tc.plateV/2, 5)
			boss := sketchBossSlab(t, doc, world, 5,
				-tc.bossU/2, -tc.bossV/2, tc.bossU/2, tc.bossV/2, 5)
			joined, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			closed, err := decad.Shell(t.Context(), joined, nil, units.Millimeters(1), decad.WithNoOpenings())
			require.NoError(t, err)
			require.Len(t, closed.Shells(), 2)
			require.True(t, closed.Shells()[1].IsVoid())
			volume, err := closed.Volume()
			require.NoError(t, err)
			// The two orientations have the same dimensions and the same
			// quarter-cylinder transition volume.
			want := 2520 - 22*16*3 - 10*4*4 - (224.0/3 - 9*math.Pi)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
			mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

func TestClosedShellOfNestedRectangularSlabsReversedLevels(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	world := sketch.NewWorld()
	lower := sketchBossSlab(t, doc, world, 0, 5, 5, 15, 15, 5)
	upper := sketchBossSlab(t, doc, world, 5, 0, 0, 20, 20, 5)
	joined, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	closed, err := decad.Shell(t.Context(), joined, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	require.Len(t, closed.Shells(), 2)
	require.True(t, closed.Shells()[1].IsVoid())
	volume, err := closed.Volume()
	require.NoError(t, err)
	want := 2500 - 18*18*3 - 8*8*4 - (308.0/3 - 10*math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	mesh, err := closed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestClosedShellOfNestedRectangularSlabsRefusesInexactReverseFrame(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 7), r3.NewVec(-1, 1, 0))
	require.NoError(t, err)
	flipped, err := r3.NewFrame(frame.Origin(), frame.U(), frame.V().Scale(-1))
	require.NoError(t, err)
	require.NotEqual(t, frame.U(), flipped.U())

	doc := decad.New()
	world := sketch.NewWorld()
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	lower := sketchBossSlabOnPlane(t, doc, world, base, 0, 5, 5, 15, 15, 5)
	upper := sketchBossSlabOnPlane(t, doc, world, base, 5, 0, 0, 20, 20, 5)
	joined, err := decad.Union(t.Context(), lower, upper)
	require.NoError(t, err)
	_, err = joined.Volume()
	require.NoError(t, err)
	closed, err := decad.Shell(t.Context(), joined, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.Nil(t, closed)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
