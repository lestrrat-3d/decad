package facetedrecord

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PointCapRingInterval bounds a Split cap's projection on the held axis.
func PointCapRingInterval(ctx context.Context, ring []r3.Vec, axis r3.Vec,
	axisLen, displacement float64) (float64, float64, bool, error) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for i, vertex := range ring {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, false, err
			}
		}
		if !proofbound.FiniteVec(vertex) {
			return 0, 0, false, nil
		}
		dot := proofbound.AxisDot(vertex, axis)
		lo = min(lo, proofbound.RatFloatDown(dot))
		hi = max(hi, proofbound.RatFloatUp(dot))
	}
	delta := proofbound.ProductUpper(displacement, axisLen)
	lo = math.Nextafter(lo-delta, math.Inf(-1))
	hi = math.Nextafter(hi+delta, math.Inf(1))
	return lo, hi, !proofbound.IsNonFinite(lo) && !proofbound.IsNonFinite(hi), nil
}

// PointCapAffineDown rounds the weighted cap projection downward.
func PointCapAffineDown(lower, upper float64, lowerWeight, upperWeight, divisor int64) float64 {
	q := pointPlaneAdd(pointPlaneMul(big.NewRat(lowerWeight, 1), ratOfPointPlane(lower)),
		pointPlaneMul(big.NewRat(upperWeight, 1), ratOfPointPlane(upper)))
	return proofbound.RatFloatDown(new(big.Rat).Quo(q, big.NewRat(divisor, 1)))
}

// PointCapAffineUp rounds the weighted cap projection upward.
func PointCapAffineUp(lower, upper float64, lowerWeight, upperWeight, divisor int64) float64 {
	q := pointPlaneAdd(pointPlaneMul(big.NewRat(lowerWeight, 1), ratOfPointPlane(lower)),
		pointPlaneMul(big.NewRat(upperWeight, 1), ratOfPointPlane(upper)))
	return proofbound.RatFloatUp(new(big.Rat).Quo(q, big.NewRat(divisor, 1)))
}
