package decad_test

import (
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestThickenPrismCircle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name                      string
		side                      decad.ThickenSide
		thickness, volume, radius float64
	}{
		{"positive", decad.ThickenPositive, 2, 440, 12},
		{"negative", decad.ThickenNegative, 2, 360, 10},
		{"centered", decad.ThickenCentered, 4, 800, 12},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateCircle(s.CreatePoint(0, 0), 10)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			doc := decad.New()
			sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
				D: units.Millimeters(10), Dir: decad.Along,
			}, decad.WithSurfaceResult())
			require.NoError(t, err)
			solid, err := sheet.Thicken(t.Context(), units.Millimeters(tc.thickness),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.Len(t, solid.Faces(), 4)
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			requirePiLinearEnclosed(t, volume, 0, tc.volume)
			decadtest.MeasuresBounds(t, solid, r3.NewVec(-tc.radius, -tc.radius, 0),
				r3.NewVec(tc.radius, tc.radius, 10), decadtest.Exactly())
		})
	}
}

func TestThickenPrismRectangle(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
		decad.WithSurfaceResult())
	require.NoError(t, err)
	solid, err := sheet.Thicken(t.Context(), units.Millimeters(5),
		decad.WithThickenSide(decad.ThickenNegative))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	require.Len(t, solid.Faces(), 10)
	decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(15000), decadtest.Exactly())
	decadtest.MeasuresArea(t, solid, units.SquareMillimeters(9000), decadtest.Exactly())
	decadtest.MeasuresCentroid(t, solid, r3.NewVec(50, 30, 5), decadtest.Exactly())
	decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 10), decadtest.Exactly())
	_, err = sheet.Thicken(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

func TestThickenPrismRectangleRoundedSides(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name       string
		side       decad.ThickenSide
		thickness  float64
		volumeBase int64
	}{
		{"positive", decad.ThickenPositive, 2, 6400},
		{"centered", decad.ThickenCentered, 4, 12640},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := plateSketch(t)
			doc := decad.New()
			sheet, err := doc.Extrude(s, p, decad.Distance{
				D: units.Millimeters(10), Dir: decad.Along,
			}, decad.WithSurfaceResult())
			require.NoError(t, err)
			solid, err := sheet.Thicken(t.Context(), units.Millimeters(tc.thickness),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Len(t, solid.Faces(), 14)
			volume, err := solid.Volume()
			require.NoError(t, err)
			requirePiLinearEnclosed(t, volume, tc.volumeBase, 40)
			decadtest.MeasuresBounds(t, solid, r3.NewVec(-2, -2, 0), r3.NewVec(102, 62, 10),
				decadtest.Exactly())
		})
	}
}

func TestThickenPrismNeckRefusal(t *testing.T) {
	t.Parallel()
	doc, sheet := thickenNeckSheet(t)
	_, err := sheet.Thicken(t.Context(), units.Millimeters(3), decad.WithThickenSide(decad.ThickenNegative))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "crosses itself"), err.Error())
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, decad.BodySheet, sheet.Kind())
}

func TestThickenPrismConcaveAdmitted(t *testing.T) {
	t.Parallel()
	_, sheet := thickenNeckSheet(t)
	solid, err := sheet.Thicken(t.Context(), units.Millimeters(0.5),
		decad.WithThickenSide(decad.ThickenNegative))
	require.NoError(t, err)
	require.Len(t, solid.Faces(), 30)
	volume, err := solid.Volume()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, volume, 640, 2.5)
	decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(30, 20, 10), decadtest.Exactly())
}

func thickenNeckSheet(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	pts := [][2]float64{
		{0, 0}, {10, 0}, {10, 8}, {20, 8}, {20, 0}, {30, 0},
		{30, 20}, {20, 20}, {20, 12}, {10, 12}, {10, 20}, {0, 20},
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

func TestThickenPrismUnrepresentableOffset(t *testing.T) {
	t.Parallel()
	base := math.Ldexp(1, 40)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(base, 0, base+100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	}, decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = sheet.Thicken(t.Context(), units.Millimeters(math.Ldexp(1, -13)),
		decad.WithThickenSide(decad.ThickenNegative))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.True(t, strings.Contains(err.Error(), "rounded"), err.Error())
	require.Len(t, doc.Bodies(), 1)
}

func TestThickenChainFamiliesStayStaged(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(10, 0)
	b := s.CreatePoint(10, 40)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chain := s.Chains()[0]
	doc := decad.New()
	ribbon, err := doc.ExtrudeChain(s, chain, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	_, err = ribbon.Thicken(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	decadtest.MeasuresArea(t, ribbon, units.SquareMillimeters(400), decadtest.Exactly())
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}
	shell, err := doc.RevolveChain(s, chain, axis, decad.FullRevolution{})
	require.NoError(t, err)
	_, err = shell.Thicken(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(shell)
	require.NoError(t, err)
	require.Len(t, free, 2)
	require.Len(t, doc.Bodies(), 2)
}

func requirePiLinearEnclosed(t *testing.T, got decad.Measurement, base int64, coeff float64) {
	t.Helper()
	piLow, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510")
	require.True(t, ok)
	piHigh, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937511")
	require.True(t, ok)
	if coeff < 0 {
		piLow, piHigh = piHigh, piLow
	}
	factor := new(big.Rat).SetFloat64(coeff)
	wantLow := new(big.Rat).Add(big.NewRat(base, 1), new(big.Rat).Mul(factor, piLow))
	wantHigh := new(big.Rat).Add(big.NewRat(base, 1), new(big.Rat).Mul(factor, piHigh))
	value := new(big.Rat).SetFloat64(got.Value.Base())
	bound := new(big.Rat).SetFloat64(got.Bound.Base())
	measuredLow := new(big.Rat).Sub(value, bound)
	measuredHigh := new(big.Rat).Add(value, bound)
	require.LessOrEqual(t, measuredLow.Cmp(wantLow), 0)
	require.GreaterOrEqual(t, measuredHigh.Cmp(wantHigh), 0)
}
