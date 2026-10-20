package box

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// GapMeasurement publishes the axis gap of an exact box pair as a length.
func GapMeasurement(gaps [3]proofarith.Dyadic) (measurement.Measurement, bool) {
	reading, ok := AxisGap(gaps)
	if !ok {
		return measurement.Measurement{}, false
	}
	return reading.Measurement(), true
}

// DyadicPointMeasurement publishes an exact dyadic point as a vector measurement.
func DyadicPointMeasurement(point *proofarith.DyV3) (measurement.VecMeasurement, bool) {
	reading, ok := ReadPointAt(point)
	if !ok {
		return measurement.VecMeasurement{}, false
	}
	return reading.Measurement(), true
}

// SignedMeasurement publishes an exact signed dyadic as a length.
func SignedMeasurement(value proofarith.Dyadic) (measurement.Measurement, bool) {
	reading, ok := SignedReading(value)
	if !ok {
		return measurement.Measurement{}, false
	}
	return reading.Measurement(), true
}

// DyadicNormalMeasurement publishes an exact dyadic axis as a unit normal and its
// angle.
func DyadicNormalMeasurement(axis proofarith.DyV3) (measurement.VecMeasurement, units.Value, bool) {
	reading, ok := DyadicNormalReading(axis)
	if !ok {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	normal, angle := reading.Measurement()
	return normal, angle, true
}
