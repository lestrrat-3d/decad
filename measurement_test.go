package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExactness(t *testing.T) {
	t.Parallel()
	// Exact is the zero value on purpose: an analytic result is the default
	// state, and Approximate is the marked one.
	var e decad.Exactness
	require.Equal(t, decad.Exact, e, `the zero Exactness should be Exact`)

	require.Equal(t, "Exact", decad.Exact.String())
	require.Equal(t, "Approximate", decad.Approximate.String())
	require.Equal(t, "Exactness(42)", decad.Exactness(42).String(), `an out-of-range value should render its raw number`)
}

func TestMeasurement(t *testing.T) {
	t.Parallel()
	// A v1 boolean-derived volume: the value and its error bound carry the
	// same Kind — the error bound on a volume is a volume.
	vol := decad.Measurement{
		Value:     units.CubicMillimeters(12.9997),
		Exactness: decad.Approximate,
		Bound:     units.CubicMillimeters(1e-3),
	}
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Equal(t, vol.Value.Kind(), vol.Bound.Kind(), `Bound should carry the same Kind as Value`)
	require.Equal(t, units.Volume, vol.Value.Kind())
	require.InDelta(t, 12.9997, vol.Value.Base(), 1e-12, `Base is the magnitude in mm³`)

	// The exact analog: zero Bound.
	exact := decad.Measurement{Value: units.CubicMillimeters(13)}
	require.Equal(t, decad.Exact, exact.Exactness)
	require.InDelta(t, 13.0, exact.Value.Base(), 1e-12)
	require.Zero(t, exact.Bound.Base(), `an Exact measurement carries a zero Bound`)
}

func TestVecMeasurement(t *testing.T) {
	t.Parallel()
	// A position: Value is millimetres by convention, Bound is a Length.
	pos := decad.VecMeasurement{
		Value:     r3.NewVec(10, 20, 30),
		Exactness: decad.Approximate,
		Bound:     units.Millimeters(1e-3),
	}
	require.Equal(t, decad.Approximate, pos.Exactness)
	require.Equal(t, units.Length, pos.Bound.Kind(), `a position's Bound is a Length`)
	require.Equal(t, r3.NewVec(10, 20, 30), pos.Value)

	// A direction: dimensionless, so its Bound is dimensionless too — typing
	// it as a length would be the wrong-Kind coercion §5.1 forbids.
	dir := decad.VecMeasurement{
		Value:     r3.NewVec(0, 0, 1),
		Exactness: decad.Approximate,
		Bound:     units.Scalar(1e-6),
	}
	require.Equal(t, decad.Approximate, dir.Exactness)
	require.Equal(t, r3.NewVec(0, 0, 1), dir.Value)
	require.Equal(t, units.Dimensionless, dir.Bound.Kind(), `a direction's Bound is Dimensionless`)
}

func TestBox(t *testing.T) {
	t.Parallel()
	box := decad.Box{
		Min:       r3.NewVec(0, 0, 0),
		Max:       r3.NewVec(100, 60, 10),
		Exactness: decad.Approximate,
		Bound:     units.Millimeters(0.01),
	}
	require.Equal(t, decad.Approximate, box.Exactness)
	require.Equal(t, units.Length, box.Bound.Kind(), `a Box's Bound is a Length`)
	require.Equal(t, 100.0, box.Max.X-box.Min.X)

	// The exact analog: zero Bound, Exact by default.
	tight := decad.Box{Min: r3.NewVec(-1, -1, -1), Max: r3.NewVec(1, 1, 1)}
	require.Equal(t, decad.Exact, tight.Exactness)
	require.Equal(t, r3.NewVec(-1, -1, -1), tight.Min)
	require.Equal(t, r3.NewVec(1, 1, 1), tight.Max)
	require.Zero(t, tight.Bound.Base())
}
