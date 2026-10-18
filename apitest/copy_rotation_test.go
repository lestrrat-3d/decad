package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCopyRotationOptionErrors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 2)
	z := r3.Vec{Z: 1}
	nan := math.NaN()
	tests := []struct {
		name   string
		center r3.Vec
		axis   r3.Vec
		angle  units.Value
		want   error
	}{
		{"zero axis", r3.Vec{}, r3.Vec{}, units.Degrees(30), decad.ErrDegenerate},
		{"non-finite center", r3.Vec{X: nan}, z, units.Degrees(30), decad.ErrNotFinite},
		{"non-finite axis", r3.Vec{}, r3.Vec{Z: nan}, units.Degrees(30), decad.ErrDegenerate},
		{"non-finite angle", r3.Vec{}, z, units.Radians(nan), decad.ErrNotFinite},
		{"not an angle", r3.Vec{}, z, units.Millimeters(3), decad.ErrUnitKind},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			_, err := decad.Copy(t.Context(), box, decad.WithCopyRotation(tc.center, tc.axis, tc.angle))
			require.ErrorIs(t, err, tc.want)
			require.Len(t, doc.Bodies(), 1)
		})
	}
	_, err := decad.Copy(t.Context(), box, decad.WithCopyRotation(r3.Vec{}, z, units.Radians(0)))
	require.NoError(t, err, "a zero angle is the identity rotation")
}

func TestCopyRotationLastPlacementOptionWins(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 4, 4, 2)
	shift, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	turn := decad.WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90))

	first, err := decad.Copy(t.Context(), box, turn, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	c, err := first.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 12, c.Value.X, 1e-9)
	require.InDelta(t, 2, c.Value.Y, 1e-9)

	second, err := decad.Copy(t.Context(), box, decad.WithCopyPlacement(shift), turn)
	require.NoError(t, err)
	c, err = second.Centroid()
	require.NoError(t, err)
	require.InDelta(t, -2, c.Value.X, 1e-9)
	require.InDelta(t, 2, c.Value.Y, 1e-9)
}

func TestCopyRotationPrismMatchesPlacement(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 1, 2, 5, 4, 3)
	center, axis := r3.NewVec(1, -1, 0), r3.NewVec(1, 2, 3)
	angle := units.Degrees(37)
	placement, err := r3.RotationAround(center, axis, angle)
	require.NoError(t, err)
	want, err := decad.Copy(t.Context(), box, decad.WithCopyPlacement(placement))
	require.NoError(t, err)
	got, err := decad.Copy(t.Context(), box, decad.WithCopyRotation(center, axis, angle))
	require.NoError(t, err)

	wv, err := want.Volume()
	require.NoError(t, err)
	gv, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, wv.Value.Base(), gv.Value.Base(), 1e-9)
	wc, err := want.Centroid()
	require.NoError(t, err)
	gc, err := got.Centroid()
	require.NoError(t, err)
	require.InDelta(t, wc.Value.X, gc.Value.X, 1e-9)
	require.InDelta(t, wc.Value.Y, gc.Value.Y, 1e-9)
	require.InDelta(t, wc.Value.Z, gc.Value.Z, 1e-9)
}

func TestCopyRotationFacetedCopyIsFacetedAndRotates(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 2, 1)
	doc := decad.New()
	lower, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, loftMirrorPlane(t))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), upper, lower)
	require.NoError(t, err)
	angle := units.Degrees(30)
	copied, err := decad.Copy(t.Context(), joined, decad.WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, angle))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 2)

	for _, face := range copied.Faces() {
		require.IsType(t, decad.Faceted{}, face.Surface())
		_, err := face.NormalAt(r3.Vec{})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}

	jc, err := joined.Centroid()
	require.NoError(t, err)
	cc, err := copied.Centroid()
	require.NoError(t, err)
	sin, cos := math.Sincos(angle.Base())
	slack := jc.Bound.Base() + cc.Bound.Base() + 1e-9
	require.InDelta(t, cos*jc.Value.X-sin*jc.Value.Y, cc.Value.X, slack)
	require.InDelta(t, sin*jc.Value.X+cos*jc.Value.Y, cc.Value.Y, slack)
	require.InDelta(t, jc.Value.Z, cc.Value.Z, slack)

	jv, err := joined.Volume()
	require.NoError(t, err)
	cv, err := copied.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(jv.Value.Base()-cv.Value.Base()), jv.Bound.Base()+cv.Bound.Base()+1e-9)

	mesh, err := copied.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestCopyRotationMintsCapRolesUnderEveryCopy(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 2, 1)
	doc := decad.New()
	lower, err := decad.Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, loftMirrorPlane(t))
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), upper, lower)
	require.NoError(t, err)
	turn := decad.WithCopyRotation(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(45))
	first, err := decad.Copy(t.Context(), joined, turn)
	require.NoError(t, err)
	second, err := decad.Copy(t.Context(), first, turn)
	require.NoError(t, err)
	for _, body := range []*decad.Body{first, second} {
		for _, role := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
			faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(body)
			require.NoError(t, err)
			require.IsType(t, decad.Faceted{}, faces[0].Surface())
		}
	}
}
