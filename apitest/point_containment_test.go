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

func TestPointLocationOnAPlate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want decad.PointLocation
	}{
		{name: "material", at: r3.NewVec(5, 5, 5), want: decad.PointInside},
		{name: "outside", at: r3.NewVec(12, 5, 5), want: decad.PointOutside},
		{name: "face", at: r3.NewVec(0, 5, 5), want: decad.PointOnBoundary},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := plate.LocatePoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestPointLocationAcrossPocketsAndBores(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	bore := circleBodyAtZ(t, doc, 0, 2, 0, 10)
	part, err := decad.Cut(t.Context(), plate, bore)
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want decad.PointLocation
	}{
		{name: "bore void", at: r3.NewVec(0, 0, 5), want: decad.PointOutside},
		{name: "material", at: r3.NewVec(5, 0, 5), want: decad.PointInside},
		{name: "curved wall", at: r3.NewVec(2, 0, 5), want: decad.PointUndecided},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := part.LocatePoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}

	pocket := boxBodyAtZ(t, doc, 5, -2, 8, 2, 6, 4)
	stack, err := decad.Cut(t.Context(), part, pocket)
	require.NoError(t, err)
	for _, tc := range []struct {
		at   r3.Vec
		want decad.PointLocation
	}{
		{at: r3.NewVec(6.5, 0, 8), want: decad.PointOutside},
		{at: r3.NewVec(6, 0, 4), want: decad.PointInside},
	} {
		got, err := stack.LocatePoint(t.Context(), tc.at, units.Millimeters(0.1))
		require.NoError(t, err)
		require.Equal(t, tc.want, got)
	}
}

func TestPointLocationOnFacetedAndPlacedBodies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
	part, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	require.True(t, anyFaceIsFaceted(part))
	for _, tc := range []struct {
		at   r3.Vec
		want decad.PointLocation
	}{
		{at: r3.NewVec(10, 10, 4), want: decad.PointOutside},
		{at: r3.NewVec(3, 3, 4), want: decad.PointInside},
		{at: r3.NewVec(30, 30, 4), want: decad.PointOutside},
	} {
		got, err := part.LocatePoint(t.Context(), tc.at, units.Millimeters(0.1))
		require.NoError(t, err)
		require.Equal(t, tc.want, got)
	}

	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := part.Placed(t.Context(), rotation)
	require.NoError(t, err)
	for _, tc := range []struct {
		at   r3.Vec
		want decad.PointLocation
	}{
		{at: r3.NewVec(10, 10, 4), want: decad.PointOutside},
		{at: r3.NewVec(3, 3, 4), want: decad.PointInside},
	} {
		got, err := placed.LocatePoint(t.Context(), rotation.Apply(tc.at), units.Millimeters(0.1))
		require.NoError(t, err)
		require.Equal(t, tc.want, got)
	}
}

func TestPointLocationRejectsInvalidRequests(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 10, 10, 10)
	point := r3.NewVec(5, 5, 5)
	var nilContext context.Context
	_, err := plate.LocatePoint(nilContext, point, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	var nilBody *decad.Body
	_, err = nilBody.LocatePoint(t.Context(), point, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = plate.LocatePoint(t.Context(), r3.NewVec(math.NaN(), 0, 0), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = plate.LocatePoint(t.Context(), point, units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = plate.LocatePoint(t.Context(), point, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)

	s, p := plateSketch(t)
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = sheet.LocatePoint(t.Context(), point, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrNotSolid)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = plate.LocatePoint(ctx, point, units.Millimeters(0.1))
	require.ErrorIs(t, err, context.Canceled)
	require.NoError(t, doc.Remove(plate))
	got, err := plate.LocatePoint(t.Context(), point, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, got)
}
