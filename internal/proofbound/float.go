package proofbound

import "math"

// UlpOf is the spacing of float64 at magnitude x.
func UlpOf(x float64) float64 {
	ax := math.Abs(x)
	return math.Nextafter(ax, math.Inf(1)) - ax
}

// IsNonFinite reports NaN or +-Inf. It compares instead of calling math.IsInf, whose
// math.Abs bit cast costs a checkptr call under -race.
func IsNonFinite(f float64) bool { return f != f || f > math.MaxFloat64 || f < -math.MaxFloat64 }
