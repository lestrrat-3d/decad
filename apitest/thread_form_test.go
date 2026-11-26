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
	return decad.TrapezoidThreadForm(units.Degrees(29), units.Millimeters(1.1), units.Millimeters(0.74))
}

func TestThreadFormCutsATrapezoidalGrooveInARod(t *testing.T) {
	t.Parallel()
	const pitch = 2.0
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	out, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(pitch)).
		Form(acmeForm()).Do(t.Context())
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
	part, err := decad.Cut(block, bore).Do(t.Context())
	require.NoError(t, err)
	form := decad.TrapezoidThreadForm(units.Degrees(30), units.Millimeters(1), units.Millimeters(0.6))
	out, err := decad.Thread(part, decad.Faces(decad.Cylindrical()), units.Millimeters(pitch)).
		Form(form).Do(t.Context())
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	slope := math.Tan(units.Degrees(30).Base() / 2)
	removed := removedTrapezoid(4.2, 5.2, 10, pitch, 0.6, slope, func(rho float64) float64 { return 4.2 + 1 - rho })
	requireThreadVolume(t, volume, 20*20*10-math.Pi*4.2*4.2*10-removed)
}

func TestThreadFormOfTheISOValuesIsTheISOThread(t *testing.T) {
	t.Parallel()
	iso := decad.TrapezoidThreadForm(
		units.Degrees(60), units.Millimeters(threadHeight*17/24*threadPitch), units.Millimeters(threadPitch/8))
	volumes := make([]decad.Measurement, 2)
	for i, form := range []decad.ThreadForm{{}, iso} {
		rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
		out, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(threadPitch)).
			Form(form).Do(t.Context())
		require.NoError(t, err)
		volumes[i], err = out.Volume()
		require.NoError(t, err)
	}
	require.InDelta(t, volumes[0].Value.Base(), volumes[1].Value.Base(), 1e-6)
}

func TestThreadFormIsWiderThanTheISOGroove(t *testing.T) {
	t.Parallel()
	// A 60 degree groove 1.2 mm deep cuts more of a rod than the 29 degree one 1.2 mm deep
	// with the same flat, because its flanks open faster. One 60 degree build is checked against
	// its closed form, and the 29 degree volume comes from the closed form that
	// TestThreadFormCutsATrapezoidalGrooveInARod checks against a build.
	const pitch, depth, flat = 3.0, 1.2, 0.5
	removed := func(angle float64) float64 {
		slope := math.Tan(units.Degrees(angle).Base() / 2)
		return removedTrapezoid(5-depth, 5, 20, pitch, flat, slope, func(rho float64) float64 { return rho - (5 - depth) })
	}
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	form := decad.TrapezoidThreadForm(units.Degrees(60), units.Millimeters(depth), units.Millimeters(flat))
	out, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(pitch)).
		Form(form).Do(t.Context())
	require.NoError(t, err)
	v, err := out.Volume()
	require.NoError(t, err)
	requireThreadVolume(t, v, math.Pi*25*20-removed(60))
	require.Less(t, v.Value.Base()+v.Bound.Base(), math.Pi*25*20-removed(29),
		"the 60 degree groove leaves less of the rod than the 29 degree one")
}

func TestThreadFormRefusals(t *testing.T) {
	t.Parallel()
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	thread := func(form decad.ThreadForm) error {
		_, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(2)).
			Form(form).Do(t.Context())
		return err
	}
	// acme is the Acme form with any of its three values replaced.
	acme := func(angle, depth, flat units.Value) decad.ThreadForm {
		return decad.TrapezoidThreadForm(angle, depth, flat)
	}
	angle, depth, flat := units.Degrees(29), units.Millimeters(1.1), units.Millimeters(0.74)
	require.ErrorIs(t, thread(acme(angle, depth, units.Value{})), decad.ErrDegenerate, "a form with no root flat")
	require.ErrorIs(t, thread(acme(units.Millimeters(29), depth, flat)), decad.ErrUnitKind)
	require.ErrorIs(t, thread(acme(units.Degrees(-29), depth, flat)), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, thread(acme(units.Degrees(121), depth, flat)), decad.ErrDegenerate)
	require.ErrorIs(t, thread(acme(angle, units.Millimeters(5), flat)), decad.ErrDegenerate,
		"a groove that reaches the axis")
	require.ErrorIs(t, thread(acme(angle, depth, units.Millimeters(1.9))), decad.ErrDegenerate,
		"a groove as wide as the pitch")
	require.ErrorIs(t, thread(acme(angle, units.Degrees(1), flat)), decad.ErrUnitKind)
}

func buttressForm() decad.ThreadForm {
	return buttress(units.Degrees(45), units.Degrees(7))
}

// buttress is the buttress form of buttressForm with the two flank leans replaced.
func buttress(lead, trail units.Value) decad.ThreadForm {
	return decad.ButtressThreadForm(lead, trail, units.Millimeters(1.5), units.Millimeters(0.4))
}

func TestThreadFormCutsAnAsymmetricButtressGroove(t *testing.T) {
	t.Parallel()
	const pitch = 3.0
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	out, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(pitch)).
		Form(buttressForm()).Do(t.Context())
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// Width at a radius is the flat plus each flank's lean over the height above the root: the two
	// slopes add, so the volume is a symmetric groove's of their mean.
	slope := (math.Tan(45*math.Pi/180) + math.Tan(7*math.Pi/180)) / 2
	removed := removedTrapezoid(5-1.5, 5, 20, pitch, 0.4, slope, func(rho float64) float64 { return rho - (5 - 1.5) })
	requireThreadVolume(t, volume, math.Pi*25*20-removed)
}

func TestThreadFormAsymmetricRefusals(t *testing.T) {
	t.Parallel()
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	thread := func(form decad.ThreadForm) error {
		_, err := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(3)).
			Form(form).Do(t.Context())
		return err
	}
	lead, trail := units.Degrees(45), units.Degrees(7)
	require.ErrorIs(t, thread(buttress(lead, units.Value{})), decad.ErrDegenerate, "both flanks or none")
	require.ErrorIs(t, thread(buttress(lead, units.Degrees(90))), decad.ErrDegenerate,
		"a flank must lean from the radial")
	require.ErrorIs(t, thread(buttress(units.Millimeters(1), trail)), decad.ErrUnitKind)
	require.ErrorIs(t, thread(buttress(units.Degrees(-5), trail)), decad.ErrNegativeMagnitude)
}
