package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFaceDistanceFromBoxCap(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	faces, err := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(1).SelectFaces(box)
	require.NoError(t, err)
	top := faces[0]
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want float64
	}{
		{name: "on patch", at: r3.NewVec(5, 5, 10)},
		{name: "inside solid", at: r3.NewVec(5, 5, 5), want: 5},
		{name: "beyond corner", at: r3.NewVec(12, 5, 12), want: math.Sqrt(8)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := top.DistanceToPoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(got.Value.Base()-tc.want), got.Bound.Base()+1e-15)
		})
	}
	require.NoError(t, doc.Remove(box))
	retired, err := top.DistanceToPoint(t.Context(), r3.NewVec(5, 5, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 5.0, retired.Value.Base())
	_, err = top.DistanceToPoint(context.Context(nil), r3.NewVec(5, 5, 5), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = top.DistanceToPoint(t.Context(), r3.NewVec(math.NaN(), 0, 0), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = top.DistanceToPoint(t.Context(), r3.NewVec(5, 5, 5), units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
}

func TestFaceDistanceFromCurvedWall(t *testing.T) {
	t.Parallel()
	cylinder := circleBodyAtZ(t, decad.New(), 0, 2, 0, 10)
	faces, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(cylinder)
	require.NoError(t, err)
	got, err := faces[0].DistanceToPoint(t.Context(), r3.NewVec(0, 0, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got.Value.Base()-2), got.Bound.Base())
	require.Greater(t, got.Value.Base()-got.Bound.Base(), 0.0)
}

func TestFaceDistanceFromSheet(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := decad.New().Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	faces, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).Exactly(1).SelectFaces(sheet)
	require.NoError(t, err)
	got, err := faces[0].DistanceToPoint(t.Context(), r3.NewVec(13, 5, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 3.0, got.Value.Base())
	require.Zero(t, got.Bound.Base())
}
