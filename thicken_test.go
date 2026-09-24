package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThickenPatchRectangle(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	solid, err := patch.Thicken(t.Context(), units.Millimeters(2))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	require.Len(t, solid.Faces(), 6)
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(12000), decadtest.Exactly())
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(12640), decadtest.Exactly())
	decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 2), decadtest.Exactly())
	decadtest.MeasuresCentroid(t, solid, r3.NewVec(50, 30, 1), decadtest.Exactly())
	require.Len(t, solid.Lumps(), 1)
	require.False(t, solid.Shells()[0].IsOpen())
	_, err = patch.Thicken(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

func TestThickenPatchSides(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name              string
		side              decad.ThickenSide
		low, high, middle float64
	}{
		{"negative", decad.ThickenNegative, -2, 0, -1},
		{"centered", decad.ThickenCentered, -1, 1, 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := plateSketch(t)
			doc := decad.New()
			patch, err := doc.Patch(t.Context(), s, p)
			require.NoError(t, err)
			solid, err := patch.Thicken(t.Context(), units.Millimeters(2), decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(12000), decadtest.Exactly())
			decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, tc.low),
				r3.NewVec(100, 60, tc.high), decadtest.Exactly())
			decadtest.MeasuresCentroid(t, solid, r3.NewVec(50, 30, tc.middle), decadtest.Exactly())
			face, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(patch)
			require.NoError(t, err)
			n, err := face[0].NormalAt(r3.NewVec(50, 30, 0))
			require.NoError(t, err)
			require.Equal(t, 1.0, n.Value.Z)
		})
	}
}

func TestThickenPatchHole(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	solid, err := patch.Thicken(t.Context(), units.Millimeters(2))
	require.NoError(t, err)
	require.Len(t, solid.Faces(), 7)
	vol, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.LessOrEqual(t, math.Abs(vol.Value.Base()-(12000-200*math.Pi)), vol.Bound.Base())
	area, err := solid.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(12640-160*math.Pi)), area.Bound.Base())
	decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 2), decadtest.Exactly())
}

func TestThickenPatchConversionBound(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, s.Profiles()[0])
	require.NoError(t, err)
	solid, err := patch.Thicken(t.Context(), units.Inches(0.1))
	require.NoError(t, err)
	vol, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
}

func TestThickenPatchRefusals(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	patch, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	for _, tc := range []struct {
		name  string
		value units.Value
		opts  []decad.ThickenOption
		want  error
	}{
		{"zero", units.Millimeters(0), nil, decad.ErrDegenerate},
		{"negative", units.Millimeters(-1), nil, decad.ErrNegativeMagnitude},
		{"wrong kind", units.SquareMillimeters(2), nil, decad.ErrUnitKind},
		{"nil option", units.Millimeters(2), []decad.ThickenOption{nil}, decad.ErrDegenerate},
		{"unknown side", units.Millimeters(2), []decad.ThickenOption{decad.WithThickenSide(100)}, decad.ErrDegenerate},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := patch.Thicken(t.Context(), tc.value, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Len(t, doc.Bodies(), 1)
		})
	}
}
