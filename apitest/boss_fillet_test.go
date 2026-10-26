package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §6's public-surface tests for
// BossSpec.RootFillet, on the plate of boss_test.go.

// filletRing is the volume a fillet of radius r adds round a cylinder of radius
// R where it meets a plane: the corner square r by r less the quarter disc,
// revolved about the cylinder's axis.
func filletRing(R, r float64) float64 {
	const steps = 200000
	step := r / steps
	total := 0.0
	for i := range steps {
		x := (float64(i) + 0.5) * step
		total += 2 * math.Pi * (R + x) * (r - math.Sqrt(r*r-(r-x)*(r-x))) * step
	}
	return total
}

func TestBossRootFilletRoundsTheCornerWhereTheBossMeetsTheFace(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	spec := bossSpec(plate, true, nil)
	spec.RootFillet = units.Millimeters(1)
	out, err := decad.Boss(t.Context(), plate, spec)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the plate, the boss and its probe all leave the document")
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+math.Pi*16*8+filletRing(4, 1)),
		decadtest.WithinRel(units.Scalar(1e-3)))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
	var tori int
	for _, f := range out.Faces() {
		if torus, ok := f.Surface().(decad.Torus); ok {
			tori++
			major, err := torus.Major.In(units.Millimeter)
			require.NoError(t, err)
			minor, err := torus.Minor.In(units.Millimeter)
			require.NoError(t, err)
			require.InDelta(t, 5, major, 1e-9, "the root circle's radius 4 and the fillet's 1")
			require.InDelta(t, 1, minor, 1e-9)
		}
	}
	require.Equal(t, 1, tori)
}

func TestBossRootFilletOnTheUndersideAndWithAPilot(t *testing.T) {
	t.Parallel()
	volume := func(fillet units.Value) float64 {
		_, plate := bossPlate(t)
		spec := bossSpec(plate, false, &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(6)})
		spec.RootFillet = fillet
		out, err := decad.Boss(t.Context(), plate, spec)
		require.NoError(t, err)
		v, err := out.Volume()
		require.NoError(t, err)
		return v.Value.Base()
	}
	// The same boss with and without the fillet differ by the ring the fillet adds.
	require.InDelta(t, filletRing(4, 0.5), volume(units.Millimeters(0.5))-volume(units.Value{}), 0.05)
}

func TestBossRootFilletRefusalsLeaveTheBodyLive(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	spec := bossSpec(plate, true, nil)
	spec.RootFillet = units.Millimeters(-1)
	_, err := decad.Boss(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	spec.RootFillet = units.Degrees(1)
	_, err = decad.Boss(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrUnitKind)
	// A fillet wider than the plate's top can hold is the fillet's own refusal, and the
	// plate stays live with the document as it was.
	spec.RootFillet = units.Millimeters(30)
	_, err = decad.Boss(t.Context(), plate, spec)
	require.Error(t, err)
	require.Equal(t, []*decad.Body{plate}, doc.Bodies())
}
