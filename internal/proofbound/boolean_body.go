package proofbound

import (
	"math"
)

// Moved from the root package's boolean_body.go.

// UlpOf is the spacing of float64 at magnitude x.
func UlpOf(x float64) float64 {
	ax := math.Abs(x)
	return math.Nextafter(ax, math.Inf(1)) - ax
}
