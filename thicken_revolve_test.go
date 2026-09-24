package decad_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// thickenMeridianAxis is the sketch-plane V axis every fixture below spins
// about: exactly along a recorded plane axis, which is the class §16.5 admits.
var thickenMeridianAxis = decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

// thickenWasherSheet is T158's receiver: the 10x10 mm meridian rectangle at
// r in [10, 20], z in [0, 10], spun as a surface.
func thickenWasherSheet(t *testing.T, a decad.AngularExtent) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(10, 0, 20, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Revolve(s, s.Profiles()[0], thickenMeridianAxis, a, decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// thickenTorusSheet spins one whole circle of radius r centred at (cu, 0).
func thickenTorusSheet(t *testing.T, cu, r float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(cu, 0), r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := doc.Revolve(s, s.Profiles()[0], thickenMeridianAxis, decad.FullRevolution{},
		decad.WithSurfaceResult())
	require.NoError(t, err)
	return doc, sheet
}

// TestThickenRevolveWasherInward is T158. The eroded meridian sweeps a
// toroidal cavity, so the one lump carries a second, VOID shell, and both
// Pappus readings are 1920pi: the outer rectangle's own first moment is 1500
// and the eroded one's is 540.
func TestThickenRevolveWasherInward(t *testing.T) {
	t.Parallel()
	doc, sheet := thickenWasherSheet(t, decad.FullRevolution{})
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 4)

	solid, err := sheet.Thicken(t.Context(), units.Millimeters(2),
		decad.WithThickenSide(decad.ThickenNegative))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	require.Len(t, solid.Faces(), 8)
	require.Len(t, solid.Lumps(), 1)

	shells := solid.Lumps()[0].Shells()
	require.Len(t, shells, 2)
	voids := 0
	for _, sh := range shells {
		if sh.IsVoid() {
			voids++
		}
	}
	require.Equal(t, 1, voids)

	volume, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.Positive(t, volume.Bound.Base())
	requirePiLinearEnclosed(t, volume, 0, 1920)

	area, err := solid.Area()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, area, 0, 1920)

	decadtest.MeasuresBounds(t, solid, r3.NewVec(-20, 0, -20), r3.NewVec(20, 10, 20), decadtest.Exactly())

	// The sheet is retired and still readable, and the document holds the
	// result alone.
	require.Equal(t, decad.BodySheet, sheet.Kind())
	_, err = sheet.Thicken(t.Context(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	require.Len(t, doc.Bodies(), 1)
}

// TestThickenRevolveTorus is T159. Pappus over a circular meridian puts a
// second pi in the volume: 2pi * 20 * pi(7^2 - 5^2) for the outward call.
func TestThickenRevolveTorus(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		side      decad.ThickenSide
		thickness float64
		piSquared float64
		radial    float64
		axial     float64
	}{
		{"outward", decad.ThickenPositive, 2, 960, 27, 7},
		{"centered", decad.ThickenCentered, 4, 1600, 27, 7},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, sheet := thickenTorusSheet(t, 20, 5)
			require.Len(t, sheet.Faces(), 1)
			solid, err := sheet.Thicken(t.Context(), units.Millimeters(tc.thickness),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.Len(t, solid.Faces(), 2)

			volume, err := solid.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			require.Positive(t, volume.Bound.Base())
			requirePiSquaredEnclosed(t, volume, tc.piSquared)

			decadtest.MeasuresBounds(t, solid,
				r3.NewVec(-tc.radial, -tc.axial, -tc.radial),
				r3.NewVec(tc.radial, tc.axial, tc.radial), decadtest.Exactly())
		})
	}
}

// TestThickenRevolvePartialSweep is T160: the quarter turn keeps both annular
// caps, so the result has two more faces than the full turn and its area
// carries their 2 * 64 mm^2 beside the swept walls' 480pi.
func TestThickenRevolvePartialSweep(t *testing.T) {
	t.Parallel()
	_, sheet := thickenWasherSheet(t, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	_, err := sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	solid, err := sheet.Thicken(t.Context(), units.Millimeters(2),
		decad.WithThickenSide(decad.ThickenNegative))
	require.NoError(t, err)
	require.Len(t, solid.Faces(), 10)

	volume, err := solid.Volume()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, volume, 0, 480)

	area, err := solid.Area()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, area, 128, 480)
}

// TestThickenRevolveAxisRefusals is T161: the radial gate and the exact-axis
// gate each refuse for their own reason and neither commits a body.
func TestThickenRevolveAxisRefusals(t *testing.T) {
	t.Parallel()

	t.Run("reaches the axis", func(t *testing.T) {
		t.Parallel()
		doc, sheet := thickenTorusSheet(t, 6, 5)
		_, err := sheet.Thicken(t.Context(), units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "reaches the revolve axis")
		require.Contains(t, err.Error(), "-1.000000000")
		require.Len(t, doc.Bodies(), 1)
		require.Equal(t, decad.BodySheet, sheet.Kind())
	})

	t.Run("axis off a recorded plane axis", func(t *testing.T) {
		t.Parallel()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(10, 0, 20, 10)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		doc := decad.New()
		// A 30-degree tilt inside the sketch plane: the axis still clears the
		// meridian, but its plane-local direction is no longer 0 or +/-1.
		tilted := decad.SketchLine{
			Start: decad.Point2{U: -40, V: 0},
			End:   decad.Point2{U: -40 + 0.5, V: 0.8660254037844386},
		}
		sheet, err := doc.Revolve(s, s.Profiles()[0], tilted, decad.FullRevolution{},
			decad.WithSurfaceResult())
		require.NoError(t, err)
		_, err = sheet.Thicken(t.Context(), units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "not parallel to a recorded plane axis")
		require.Len(t, doc.Bodies(), 1)
	})
}

// requirePiSquaredEnclosed asserts that got's own proven interval encloses
// coeff * pi^2, bracketed over big.Rat rather than against a second float.
func requirePiSquaredEnclosed(t *testing.T, got decad.Measurement, coeff float64) {
	t.Helper()
	piLow, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937510")
	require.True(t, ok)
	piHigh, ok := new(big.Rat).SetString("3.14159265358979323846264338327950288419716939937511")
	require.True(t, ok)
	require.Positive(t, coeff)
	factor := new(big.Rat).SetFloat64(coeff)
	wantLow := new(big.Rat).Mul(factor, new(big.Rat).Mul(piLow, piLow))
	wantHigh := new(big.Rat).Mul(factor, new(big.Rat).Mul(piHigh, piHigh))
	value := new(big.Rat).SetFloat64(got.Value.Base())
	bound := new(big.Rat).SetFloat64(got.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(value, bound).Cmp(wantLow), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(value, bound).Cmp(wantHigh), 0)
}
