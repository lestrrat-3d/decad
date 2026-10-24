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

func TestDraftCircleEndRimChamferVerified(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	beveled, err := decad.Chamfer(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(1))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, beveled,
		map[decad.SurfaceKind]int{decad.KindCone: 2, decad.KindPlane: 2}, "chamfer(")
	volume, err := beveled.Volume()
	require.NoError(t, err)
	k := math.Tan(5 * math.Pi / 180)
	n := math.Hypot(1, k)
	far := 8 - 10*k
	frustum := math.Pi * 10 * (64 + 8*far + far*far) / 3
	removed := math.Pi / n * (far + (k/n-1)/3)
	require.InDelta(t, frustum-removed, volume.Value.Base(), volume.Bound.Base()+1e-8)
	require.Less(t, volume.Bound.Base(), 1e-6)
	capEdges, err := decad.Edges(decad.OuterLoopOf(decad.CapEnd(beveled)), decad.Circular()).Exactly(1).
		SelectEdges(beveled)
	require.NoError(t, err)
	capCircle, ok := capEdges[0].Curve().(decad.Circle3)
	require.True(t, ok)
	require.InDelta(t, far-1, capCircle.Radius.Base(), 1e-8)
	var coneJunctions int
	for _, edge := range beveled.Edges() {
		faces := edge.Faces()
		if len(faces) != 2 || faces[0].Surface().Kind() != decad.KindCone ||
			faces[1].Surface().Kind() != decad.KindCone {
			continue
		}
		circle, ok := edge.Curve().(decad.Circle3)
		require.True(t, ok)
		require.InDelta(t, far+k/n, circle.Radius.Base(), 1e-8)
		coneJunctions++
	}
	require.Equal(t, 1, coneJunctions)
	mesh, err := beveled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestDraftCircleEndRimChamferPlacement(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(3, -2, 4), 5)
	shift, err := r3.Translation(r3.NewVec(6, 7, -3))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), base, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	beveled, err := decad.Chamfer(t.Context(), placed, draftedDiskEndRim(placed), units.Millimeters(1))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, beveled,
		map[decad.SurfaceKind]int{decad.KindCone: 2, decad.KindPlane: 2}, "chamfer(")
	moved, err := decad.Copy(t.Context(), beveled, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, moved,
		map[decad.SurfaceKind]int{decad.KindCone: 2, decad.KindPlane: 2}, "chamfer(")
	mesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	a, err := beveled.Volume()
	require.NoError(t, err)
	b, err := moved.Volume()
	require.NoError(t, err)
	require.InDelta(t, a.Value.Base(), b.Value.Base(), a.Bound.Base()+b.Bound.Base())
}

func TestDraftCircleEndRimChamferRefusalsLeaveSourceLive(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	_, err := decad.Chamfer(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(8))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Chamfer(t.Context(), base,
		decad.Edges(decad.CreatedBy(decad.CapStart(base)), decad.Circular()).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Chamfer(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(1),
		decad.WithAsymmetricChamfer(decad.Faces(decad.FaceCreatedBy(decad.CapEnd(base))),
			units.Millimeters(2)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Chamfer(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(1))
	require.NoError(t, err)

	flare := draftedDisk(t, r3.NewVec(0, 0, 0), -5)
	_, err = decad.Chamfer(t.Context(), flare, draftedDiskEndRim(flare), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.CreateCircle(c, 8)
		s.CreateCircle(c, 2)
		s.Fix(c)
	})
	annulus := taperExtrude(t, decad.New(), s, p, 10, 5, decad.Along)
	_, err = decad.Chamfer(t.Context(), annulus,
		decad.Edges(decad.OuterLoopOf(decad.CapEnd(annulus)), decad.Circular()).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
