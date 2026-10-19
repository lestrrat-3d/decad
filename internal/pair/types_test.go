package pair_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestScalarReadingMeasurement(t *testing.T) {
	t.Parallel()
	exact := pair.ScalarReading{ValueMM: 2.5}.Measurement()
	require.Equal(t, measurement.Measurement{Value: units.Millimeters(2.5), Bound: units.Millimeters(0),
		Exactness: measurement.Exact}, exact)
	bounded := pair.ScalarReading{ValueMM: -1, BoundMM: 0.25}.Measurement()
	require.Equal(t, measurement.Measurement{Value: units.Millimeters(-1), Bound: units.Millimeters(0.25),
		Exactness: measurement.Approximate}, bounded)
}
