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

func draftedDisk(t *testing.T, center r3.Vec, taper float64) *decad.Body {
	t.Helper()
	s, p := taperSketch(t, center, 0, drawDisk(8))
	return taperExtrude(t, decad.New(), s, p, 10, taper, decad.Along)
}

func draftedDiskEndRim(b *decad.Body) decad.EdgeSelector {
	return decad.Edges(decad.CreatedBy(decad.CapEnd(b)), decad.Circular()).Exactly(1)
}

// The independent volume uses the cone and circle equations, rather than the
// evaluator's meridian integral. The fillet removes the region between the
// original conical wall and a circle of radius f in an axial section.
func draftedDiskRoundedVolume(radius, height, taper, f float64) float64 {
	k := math.Tan(taper * math.Pi / 180)
	far := radius - height*k
	norm := math.Hypot(1, k)
	rc := far + k*f - f*norm
	zc := height - f
	u0 := f * k / norm
	u1 := f
	z0 := zc + u0
	coneIntegral := func(z float64) float64 {
		return radius*radius*z - radius*k*z*z + k*k*z*z*z/3
	}
	sqrtIntegral := func(u float64) float64 {
		return (u*math.Sqrt(math.Max(0, f*f-u*u)) + f*f*math.Asin(u/f)) / 2
	}
	arcIntegral := rc*rc*(u1-u0) + 2*rc*(sqrtIntegral(u1)-sqrtIntegral(u0)) +
		f*f*(u1-u0) - (u1*u1*u1-u0*u0*u0)/3
	frustum := math.Pi * height * (radius*radius + radius*far + far*far) / 3
	return frustum - math.Pi*(coneIntegral(height)-coneIntegral(z0)-arcIntegral)
}

func TestDraftCircleEndRimFilletVerified(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	rounded, err := decad.Fillet(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(1))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, rounded,
		map[decad.SurfaceKind]int{decad.KindCone: 1, decad.KindTorus: 1}, "fillet(")
	volume, err := rounded.Volume()
	require.NoError(t, err)
	require.InDelta(t, draftedDiskRoundedVolume(8, 10, 5, 1), volume.Value.Base(), 1e-6)
	require.Less(t, volume.Bound.Base(), 1e-6)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestDraftCircleEndRimFilletPlacement(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(3, -2, 4), 5)
	shift, err := r3.Translation(r3.NewVec(6, 7, -3))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), base, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	rounded, err := decad.Fillet(t.Context(), placed, draftedDiskEndRim(placed), units.Millimeters(1))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, rounded,
		map[decad.SurfaceKind]int{decad.KindCone: 1, decad.KindTorus: 1}, "fillet(")
	placedCopy, err := decad.Copy(t.Context(), rounded, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	checkDraftCircleModifyRoles(t, placedCopy,
		map[decad.SurfaceKind]int{decad.KindCone: 1, decad.KindTorus: 1}, "fillet(")
	mesh, err := placedCopy.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	a, err := rounded.Volume()
	require.NoError(t, err)
	b, err := placedCopy.Volume()
	require.NoError(t, err)
	require.InDelta(t, a.Value.Base(), b.Value.Base(), 1e-6)
}

func TestDraftCircleEndRimFilletRefusalsLeaveSourceLive(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	_, err := decad.Fillet(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(10))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Fillet(t.Context(), base,
		decad.Edges(decad.CreatedBy(decad.CapStart(base)), decad.Circular()).Exactly(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Fillet(t.Context(), base, draftedDiskEndRim(base), units.Millimeters(1))
	require.NoError(t, err)

	flare := draftedDisk(t, r3.NewVec(0, 0, 0), -5)
	_, err = decad.Fillet(t.Context(), flare, draftedDiskEndRim(flare), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.CreateCircle(c, 8)
		s.CreateCircle(c, 2)
		s.Fix(c)
	})
	annulus := taperExtrude(t, decad.New(), s, p, 10, 5, decad.Along)
	_, err = decad.Fillet(t.Context(), annulus,
		decad.Edges(decad.OuterLoopOf(decad.CapEnd(annulus)), decad.Circular()).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
