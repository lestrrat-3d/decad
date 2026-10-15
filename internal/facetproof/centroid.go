package facetproof

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FacetedCentroidAllowance keeps a positive occupied-volume displacement
// positive even when its product with the pair diameter underflows.
func FacetedCentroidAllowance(sym, diameter, volumeFloor float64) float64 {
	if volumeFloor <= 0 {
		return diameter
	}
	return math.Min(diameter, proofbound.DivUpper(proofbound.ProductUpper(sym, diameter), volumeFloor))
}
