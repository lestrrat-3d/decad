package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func draftedDiskCupVolume(radius, height, taper, thickness float64) float64 {
	slope := math.Tan(taper * math.Pi / 180)
	far := radius - height*slope
	shift := thickness * math.Hypot(1, slope)
	innerFloor := radius - thickness*slope - shift
	innerTop := far - shift
	frustum := func(h, a, b float64) float64 {
		return math.Pi * h * (a*a + a*b + b*b) / 3
	}
	return frustum(height, radius, far) - frustum(height-thickness, innerFloor, innerTop)
}

func checkDraftCircleShellRoles(t *testing.T, b *decad.Body) {
	t.Helper()
	require.Len(t, b.Faces(), 5)
	required := map[string]decad.SurfaceKind{
		roleCapStart:     decad.KindPlane,
		roleRim0:         decad.KindPlane,
		"shellCap":       decad.KindPlane,
		"shellSide(0,0)": decad.KindCone,
	}
	seen := map[string]int{}
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			if want, ok := required[origin.Role]; ok {
				require.Equal(t, want, f.Surface().Kind(), origin.Role)
				seen[origin.Role]++
			}
		}
	}
	for role := range required {
		require.Equal(t, 1, seen[role], role)
	}
	require.Len(t, b.Shells(), 1)
	requireEveryEdgeOnTwoFaces(t, b)
}

func TestDraftCircleShellVerified(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	cup, err := base.Shell(t.Context(),
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(base))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	checkDraftCircleShellRoles(t, cup)
	volume, err := cup.Volume()
	require.NoError(t, err)
	require.InDelta(t, draftedDiskCupVolume(8, 10, 5, 1), volume.Value.Base(), volume.Bound.Base()+1e-8)
	require.Less(t, volume.Bound.Base(), 1e-6)
	slope := taperTan(5)
	far := bfSub(bf(8), bfMul(bf(10), slope))
	shift := bfSqrt(bfAdd(bf(1), bfMul(slope, slope)))
	innerFloor := bfSub(bfSub(bf(8), slope), shift)
	innerTop := bfSub(far, shift)
	frustum := func(h, a, z *big.Float) *big.Float {
		return bfQuo(bfMul(bigPi, h, bfAdd(bfAdd(bfMul(a, a), bfMul(a, z)), bfMul(z, z))), bf(3))
	}
	want := bfSub(frustum(bf(10), bf(8), far), frustum(bf(9), innerFloor, innerTop))
	requireMeasurementCovers(t, "drafted cup volume", volume, want, 1e-9)
	mesh, err := cup.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
}

func TestDraftCircleShellPlacement(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(3, -2, 4), 5)
	shift, err := r3.Translation(r3.NewVec(6, 7, -3))
	require.NoError(t, err)
	placed, err := base.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	cup, err := placed.Shell(t.Context(),
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(placed))).Exactly(1), units.Millimeters(1))
	require.NoError(t, err)
	moved, err := cup.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	checkDraftCircleShellRoles(t, moved)
	mesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	a, err := cup.Volume()
	require.NoError(t, err)
	b, err := moved.Volume()
	require.NoError(t, err)
	require.InDelta(t, a.Value.Base(), b.Value.Base(), a.Bound.Base()+b.Bound.Base())
}

func TestDraftCircleShellRefusals(t *testing.T) {
	t.Parallel()
	base := draftedDisk(t, r3.NewVec(0, 0, 0), 5)
	top := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(base))).Exactly(1)
	_, err := base.Shell(t.Context(), top, units.Millimeters(8))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = base.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapStart(base))).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = base.Shell(t.Context(), top, units.Millimeters(1), decad.WithShellSense(decad.Outward))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	flare := draftedDisk(t, r3.NewVec(0, 0, 0), -5)
	_, err = flare.Shell(t.Context(),
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(flare))).Exactly(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.CreateCircle(c, 8)
		s.CreateCircle(c, 2)
		s.Fix(c)
	})
	annulus := taperExtrude(t, decad.New(), s, p, 10, 5, decad.Along)
	_, err = annulus.Shell(t.Context(),
		decad.Faces(decad.FaceCreatedBy(decad.CapEnd(annulus))).Exactly(1), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
