package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/thread-design.md §6's public-surface tests for
// ThreadForm, on the rod and bore of thread_test.go.

// removedTrapezoid is the volume of a trapezoidal groove that runs out of both
// ends of a cylinder: at radius rho the groove covers width(rho)/pitch of the
// length, where width is the root flat plus both flanks' widening above the root.
func removedTrapezoid(lo, hi, length, pitch, flat, slope float64, fromRoot func(float64) float64) float64 {
	const steps = 100000
	step := (hi - lo) / steps
	total := 0.0
	for i := range steps {
		rho := lo + (float64(i)+0.5)*step
		total += 2 * math.Pi * rho * length * (flat + 2*fromRoot(rho)*slope) / pitch * step
	}
	return total
}

func acmeForm() decad.ThreadForm {
	return decad.ThreadForm{Angle: units.Degrees(29), Depth: units.Millimeters(1.1), Flat: units.Millimeters(0.74)}
}

func TestThreadFormCutsATrapezoidalGrooveInARod(t *testing.T) {
	t.Parallel()
	const pitch = 2.0
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(pitch), Form: acmeForm()})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	slope := math.Tan(units.Degrees(29).Base() / 2)
	removed := removedTrapezoid(5-1.1, 5, 20, pitch, 0.74, slope, func(rho float64) float64 { return rho - (5 - 1.1) })
	requireThreadVolume(t, volume, math.Pi*25*20-removed)
}

func TestThreadFormCutsATrapezoidalGrooveOutOfABore(t *testing.T) {
	t.Parallel()
	const pitch = 2.0
	doc := decad.New()
	block := boxBody(t, doc, -10, -10, 10, 10, 10)
	bore := circleBodyAtZ(t, doc, 0, 4.2, -1, 12)
	part, err := decad.Cut(t.Context(), block, bore)
	require.NoError(t, err)
	form := decad.ThreadForm{Angle: units.Degrees(30), Depth: units.Millimeters(1), Flat: units.Millimeters(0.6)}
	out, err := decad.Thread(t.Context(), part, decad.ThreadSpec{
		Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(pitch), Form: form})
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	slope := math.Tan(units.Degrees(30).Base() / 2)
	removed := removedTrapezoid(4.2, 5.2, 10, pitch, 0.6, slope, func(rho float64) float64 { return 4.2 + 1 - rho })
	requireThreadVolume(t, volume, 20*20*10-math.Pi*4.2*4.2*10-removed)
}

func TestThreadFormOfTheISOValuesIsTheISOThread(t *testing.T) {
	t.Parallel()
	iso := decad.ThreadForm{
		Angle: units.Degrees(60), Depth: units.Millimeters(threadHeight * 17 / 24 * threadPitch),
		Flat: units.Millimeters(threadPitch / 8),
	}
	volumes := make([]decad.Measurement, 2)
	for i, form := range []decad.ThreadForm{{}, iso} {
		rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
		out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
			Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(threadPitch), Form: form})
		require.NoError(t, err)
		volumes[i], err = out.Volume()
		require.NoError(t, err)
	}
	require.InDelta(t, volumes[0].Value.Base(), volumes[1].Value.Base(), 1e-6)
}

func TestThreadFormIsWiderThanTheISOGroove(t *testing.T) {
	t.Parallel()
	// A 60 degree groove 1.2 mm deep cuts more of a rod than the 29 degree one 1.2 mm deep
	// with the same flat, because its flanks open faster.
	volume := func(angle float64) float64 {
		rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
		out, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
			Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(3),
			Form: decad.ThreadForm{Angle: units.Degrees(angle), Depth: units.Millimeters(1.2), Flat: units.Millimeters(0.5)}})
		require.NoError(t, err)
		v, err := out.Volume()
		require.NoError(t, err)
		return v.Value.Base()
	}
	require.Less(t, volume(60), volume(29))
}

func TestThreadFormRefusals(t *testing.T) {
	t.Parallel()
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	thread := func(form decad.ThreadForm) error {
		_, err := decad.Thread(t.Context(), rod, decad.ThreadSpec{
			Face: decad.Faces(decad.Cylindrical()), Pitch: units.Millimeters(2), Form: form})
		return err
	}
	full := acmeForm()
	partly := full
	partly.Flat = units.Value{}
	require.ErrorIs(t, thread(partly), decad.ErrDegenerate, "a form with no root flat")
	wrongKind := full
	wrongKind.Angle = units.Millimeters(29)
	require.ErrorIs(t, thread(wrongKind), decad.ErrUnitKind)
	negative := full
	negative.Angle = units.Degrees(-29)
	require.ErrorIs(t, thread(negative), decad.ErrNegativeMagnitude)
	wide := full
	wide.Angle = units.Degrees(121)
	require.ErrorIs(t, thread(wide), decad.ErrDegenerate)
	deep := full
	deep.Depth = units.Millimeters(5)
	require.ErrorIs(t, thread(deep), decad.ErrDegenerate, "a groove that reaches the axis")
	fat := full
	fat.Flat = units.Millimeters(1.9)
	require.ErrorIs(t, thread(fat), decad.ErrDegenerate, "a groove as wide as the pitch")
	badDepth := full
	badDepth.Depth = units.Degrees(1)
	require.ErrorIs(t, thread(badDepth), decad.ErrUnitKind)
}
