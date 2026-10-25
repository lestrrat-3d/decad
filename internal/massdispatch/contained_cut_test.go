package massdispatch

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutMassSelectionUsesOneWholeTuple(t *testing.T) {
	ordinary, source := massSelectionFixture(0.2, 0.2)
	for i := range source.Tensor {
		source.Tensor[i].Bound = units.KilogramSquareMillimeters(0.1)
		ordinary.Tensor[i].Bound = units.KilogramSquareMillimeters(0.2)
	}
	source.Mass.Bound = units.Kilograms(0.1)
	ordinary.Mass.Bound = units.Kilograms(0.2)
	selected := sourceMassPropertiesStrictlyBetter(ordinary, source)
	require.True(t, selected)

	t.Run("tie keeps ordinary", func(t *testing.T) {
		tiedOrdinary, tiedSource := massSelectionFixture(0.2, 0.2)
		tiedSource.Mass.Bound = tiedOrdinary.Mass.Bound
		for i := range tiedSource.Tensor {
			tiedSource.Tensor[i].Bound = tiedOrdinary.Tensor[i].Bound
		}
		require.False(t, sourceMassPropertiesStrictlyBetter(tiedOrdinary, tiedSource))
	})
	t.Run("one worse tensor term rejects entire source tuple", func(t *testing.T) {
		worseOrdinary, worseSource := massSelectionFixture(0.2, 0.1)
		for i := range worseSource.Tensor {
			worseSource.Tensor[i].Bound = units.KilogramSquareMillimeters(0.1)
			worseOrdinary.Tensor[i].Bound = units.KilogramSquareMillimeters(0.2)
		}
		worseSource.Tensor[4].Bound = units.KilogramSquareMillimeters(0.3)
		require.False(t, sourceMassPropertiesStrictlyBetter(worseOrdinary, worseSource))
	})
}

func massSelectionFixture(ordinaryCenter, sourceCenter float64) (massmoment.MassReadings, massmoment.MassReadings) {
	reading := func(bound float64, unit units.Unit) measurement.Measurement {
		return measurement.Measurement{Value: units.New(10, unit), Bound: units.New(bound, unit),
			Exactness: measurement.ExactnessOf(bound)}
	}
	ordinary := massmoment.MassReadings{Mass: reading(0.2, units.Kilogram),
		Center: measurement.VecMeasurement{Bound: units.Millimeters(ordinaryCenter)}}
	source := massmoment.MassReadings{Mass: reading(0.1, units.Kilogram),
		Center: measurement.VecMeasurement{Bound: units.Millimeters(sourceCenter)}}
	for i := range ordinary.Tensor {
		ordinary.Tensor[i] = reading(0.2, units.KilogramSquareMillimeter)
		source.Tensor[i] = reading(0.1, units.KilogramSquareMillimeter)
	}
	return ordinary, source
}
