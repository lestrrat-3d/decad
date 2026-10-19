package box

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Measurement publishes the reading as a position, Exact only for a zero
// bound.
func (r PointReading) Measurement() measurement.VecMeasurement {
	return measurement.VecMeasurement{Value: r.Value, Bound: units.Millimeters(r.BoundMM),
		Exactness: measurement.ExactnessOf(r.BoundMM)}
}

// Measurement publishes the reading as a unit direction, Exact only for a
// zero bound, and its angular error in radians.
func (r NormalReading) Measurement() (measurement.VecMeasurement, units.Value) {
	return measurement.VecMeasurement{Value: r.Value, Bound: units.Scalar(r.Bound),
		Exactness: measurement.ExactnessOf(r.Bound)}, units.Radians(r.Angle)
}

// PointMeasurement converts an exact world point once to a float witness
// with an outward ball (RationalPointReading). It reports false when the
// point or its ball is not finite.
func PointMeasurement(point [3]*big.Rat) (measurement.VecMeasurement, bool) {
	reading, ok := RationalPointReading(point)
	if !ok {
		return measurement.VecMeasurement{}, false
	}
	return reading.Measurement(), true
}

// PlanarNormalMeasurement normalizes an exact direction with an enclosing
// ball and its angle (PlanarNormalReading). It reports false for a zero
// direction or a reading that is not finite.
func PlanarNormalMeasurement(dir proofarith.DyV3) (measurement.VecMeasurement, units.Value, bool) {
	reading, ok := PlanarNormalReading(dir)
	if !ok {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	normal, angle := reading.Measurement()
	return normal, angle, true
}

// RationalPointReading rounds an exact world point and bounds its 3D error.
func RationalPointReading(point [3]*big.Rat) (PointReading, bool) {
	var coordinate [3]float64
	maxError := new(big.Rat)
	for axis := range 3 {
		coordinate[axis], _ = point[axis].Float64()
		if !finite(coordinate[axis]) {
			return PointReading{}, false
		}
		deviation := new(big.Rat).Sub(point[axis], proofarith.FloatRat(coordinate[axis]))
		deviation.Abs(deviation)
		if deviation.Cmp(maxError) > 0 {
			maxError = deviation
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	if !finite(bound) {
		return PointReading{}, false
	}
	return PointReading{Value: r3.Vec{X: coordinate[0], Y: coordinate[1], Z: coordinate[2]},
		BoundMM: bound}, true
}

// NormalReading encloses a unit normal and its angular error.
type NormalReading struct {
	Value        r3.Vec
	Bound, Angle float64
}

// PlanarNormalReading normalizes an exact direction. It first scales the
// direction by its largest absolute component, an exact rational that is the
// same for every positive multiple of the direction and exactly negated for
// its opposite, so a pair queried in either order publishes exactly opposite
// normals. The ball encloses the true unit vector through exact square-root
// brackets of the scaled direction's squared length.
func PlanarNormalReading(dir proofarith.DyV3) (NormalReading, bool) {
	largest := proofarith.DyAbs(dir[0])
	for _, c := range dir[1:] {
		if proofarith.DyCmp(proofarith.DyAbs(c), largest) > 0 {
			largest = proofarith.DyAbs(c)
		}
	}
	if largest.Sign() == 0 {
		return NormalReading{}, false
	}
	var scaled [3]*big.Rat
	squared := new(big.Rat)
	for k := range 3 {
		scaled[k] = new(big.Rat).Quo(dir[k].Rat(), largest.Rat())
		squared.Add(squared, new(big.Rat).Mul(scaled[k], scaled[k]))
	}
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finite(low) || !finite(high) {
		return NormalReading{}, false
	}
	raw := r3.Vec{X: sweeppath.RatFloatNearest(scaled[0]), Y: sweeppath.RatFloatNearest(scaled[1]), Z: sweeppath.RatFloatNearest(scaled[2])}
	value, ok := raw.Normalize()
	if !ok || !proofbound.FiniteVec(value) {
		return NormalReading{}, false
	}
	components := [3]float64{value.X, value.Y, value.Z}
	maxError := new(big.Rat)
	for k := range 3 {
		for _, length := range []float64{low, high} {
			deviation := new(big.Rat).Quo(scaled[k], proofarith.FloatRat(length))
			deviation.Sub(proofarith.FloatRat(components[k]), deviation)
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	angle := proofbound.UpRound(4 * bound)
	if !finite(bound) || !finite(angle) || angle >= math.Pi {
		return NormalReading{}, false
	}
	return NormalReading{Value: value, Bound: bound, Angle: angle}, true
}

// DyadicNormalReading rounds a dyadic axis into a bounded unit normal.
func DyadicNormalReading(axis proofarith.DyV3) (NormalReading, bool) {
	squared := proofarith.DvDot(axis, axis).Rat()
	if squared.Sign() <= 0 {
		return NormalReading{}, false
	}
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finite(low) || !finite(high) {
		return NormalReading{}, false
	}
	raw := r3.Vec{}
	raw.X, _ = axis[0].Float64()
	raw.Y, _ = axis[1].Float64()
	raw.Z, _ = axis[2].Float64()
	value, ok := raw.Normalize()
	if !ok || !proofbound.FiniteVec(value) {
		return NormalReading{}, false
	}
	components := [3]float64{value.X, value.Y, value.Z}
	maxError := new(big.Rat)
	for k := range 3 {
		first := new(big.Rat).Quo(axis[k].Rat(), proofarith.FloatRat(low))
		second := new(big.Rat).Quo(axis[k].Rat(), proofarith.FloatRat(high))
		for _, endpoint := range []*big.Rat{first, second} {
			deviation := new(big.Rat).Sub(proofarith.FloatRat(components[k]), endpoint)
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	angle := proofbound.UpRound(4 * bound)
	if !finite(bound) || !finite(angle) || angle >= math.Pi {
		return NormalReading{}, false
	}
	return NormalReading{Value: value, Bound: bound, Angle: angle}, true
}
