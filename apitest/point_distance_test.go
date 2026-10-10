package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPointDistanceFromBox(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want float64
	}{
		{name: "inside", at: r3.NewVec(5, 5, 5)},
		{name: "boundary", at: r3.NewVec(0, 5, 5)},
		{name: "outside face", at: r3.NewVec(12, 5, 5), want: 2},
		{name: "outside corner", at: r3.NewVec(11, 11, 11), want: math.Sqrt(3)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			distance, err := box.DistanceToPoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(distance.Value.Base()-tc.want), distance.Bound.Base()+1e-15)
			if tc.name == "outside corner" {
				require.Equal(t, decad.Approximate, distance.Exactness)
				return
			}
			require.Equal(t, decad.Exact, distance.Exactness)
			require.Zero(t, distance.Bound.Base())
		})
	}
	require.NoError(t, doc.Remove(box))
	retired, err := box.DistanceToPoint(t.Context(), r3.NewVec(12, 5, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 2.0, retired.Value.Base())
	_, err = box.DistanceToPoint(context.Context(nil), r3.NewVec(12, 5, 5), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = box.DistanceToPoint(t.Context(), r3.NewVec(math.NaN(), 0, 0), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = box.DistanceToPoint(t.Context(), r3.NewVec(12, 5, 5), units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
}

func TestPointDistanceFromBore(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	bore := circleBodyAtZ(t, doc, 0, 2, 0, 10)
	part, err := decad.Cut(t.Context(), plate, bore)
	require.NoError(t, err)
	center, err := part.DistanceToPoint(t.Context(), r3.NewVec(0, 0, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(center.Value.Base()-2), center.Bound.Base()+1e-15)
	require.Greater(t, center.Value.Base()-center.Bound.Base(), 0.0)
	wall, err := part.DistanceToPoint(t.Context(), r3.NewVec(2, 0, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.LessOrEqual(t, wall.Value.Base()-wall.Bound.Base(), 0.0)
	require.Greater(t, wall.Value.Base()+wall.Bound.Base(), 0.0)
}

func TestPointDistanceWithoutVolumeProof(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, nearTangentSlot)
	body := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	distance, err := body.DistanceToPoint(t.Context(), r3.NewVec(40, 0, 4), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Greater(t, distance.Value.Base()+distance.Bound.Base(), 0.0)
	require.LessOrEqual(t, distance.Value.Base()-distance.Bound.Base(), 0.0)
}
