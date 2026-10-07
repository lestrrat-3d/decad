package proofbound

import "math"

// UlpOf is the spacing of float64 at magnitude x.
func UlpOf(x float64) float64 {
	ax := math.Abs(x)
	return math.Nextafter(ax, math.Inf(1)) - ax
}

func IsNonFinite(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }
