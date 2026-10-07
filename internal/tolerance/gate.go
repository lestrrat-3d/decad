// Package tolerance compares bounded readings with their relative references.
package tolerance

import (
	"math"

	"github.com/lestrrat-3d/units"
)

// Epsilon is the scale floor used by body and pair references.
const Epsilon = 1e-9

// Scalar applies the inclusive scalar gate. A zero bound passes without
// loading a reference; haveRef distinguishes that path from a comparison.
func Scalar(value, bound units.Value, rel float64, reference func(float64) (float64, bool)) (bool, float64, bool) {
	boundValue := bound.Base()
	if !UsableMagnitude(boundValue) {
		return false, 0, false
	}
	if boundValue == 0 {
		return true, 0, false
	}
	v := math.Abs(value.Base())
	if !UsableMagnitude(v) {
		return false, 0, false
	}
	ref, ok := reference(v)
	if !ok {
		return false, 0, false
	}
	return Within(boundValue, ref, rel), ref, true
}

// Bounded applies the same gate to a bound whose reference takes no value.
func Bounded(bound, rel float64, reference func() (float64, bool)) (bool, float64, bool) {
	if !UsableMagnitude(bound) {
		return false, 0, false
	}
	if bound == 0 {
		return true, 0, false
	}
	ref, ok := reference()
	if !ok {
		return false, 0, false
	}
	return Within(bound, ref, rel), ref, true
}

// Within compares the represented ratio directly at the inclusive boundary.
// Multiplying that ratio back by ref can round one ulp below bound.
func Within(bound, ref, rel float64) bool {
	if !UsableMagnitude(ref) {
		return false
	}
	if ref == 0 || rel == 0 {
		return bound <= rel*ref
	}
	return bound/ref <= rel
}

// UsableMagnitude accepts finite nonnegative values.
func UsableMagnitude(v float64) bool {
	return v >= 0 && !math.IsNaN(v) && !math.IsInf(v, 0)
}

// AreaReference loads the diameter and edge length in that order.
func AreaReference(value float64, diameter, edgeLength func() (float64, bool)) (float64, bool) {
	d, ok := diameter()
	if !ok {
		return 0, false
	}
	e, ok := edgeLength()
	if !ok {
		return 0, false
	}
	return math.Max(value, Epsilon*d*e), true
}

// VolumeReference loads the diameter before reading the area.
func VolumeReference(value float64, area units.Value, diameter func() (float64, bool)) (float64, bool) {
	d, ok := diameter()
	if !ok {
		return 0, false
	}
	a := math.Abs(area.Base())
	if !UsableMagnitude(a) {
		return 0, false
	}
	return math.Max(value, Epsilon*d*a), true
}

// LengthReference loads a body's diameter for length readings.
func LengthReference(value float64, diameter func() (float64, bool)) (float64, bool) {
	d, ok := diameter()
	if !ok {
		return 0, false
	}
	return math.Max(value, Epsilon*d), true
}

// PairLengthReference uses the pair's already loaded diameter.
func PairLengthReference(value, diameter float64) (float64, bool) {
	if !UsableMagnitude(diameter) {
		return 0, false
	}
	return math.Max(value, Epsilon*diameter), true
}
