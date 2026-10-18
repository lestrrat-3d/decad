package facetedrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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

// PointCapExtremeOrder proves which cap centroid is first along the axis.
// It returns 1 for start then end, -1 for end then start, or a refusal.
func PointCapExtremeOrder(ctx context.Context, fp Payload, axis r3.Vec, axisLen float64,
	startLo, startHi, endLo, endHi float64) (int, error) {
	n := len(fp.PointPlane.Base.Verts) - 1
	lowerLo, lowerHi, ok, err := PointCapRingInterval(ctx, fp.Verts[:n], axis, axisLen,
		fp.PointPlaneCapDisplacement)
	if err != nil {
		return 0, err
	}
	if !ok {
		return 0, fmt.Errorf("%w: the lower cap boundary has no finite projection", decaderr.ErrUnsupported)
	}
	upperLo, upperHi, ok, err := PointCapRingInterval(ctx, fp.Verts[n:], axis, axisLen,
		fp.PointPlaneCapDisplacement)
	if err != nil {
		return 0, err
	}
	if !ok {
		return 0, fmt.Errorf("%w: the upper cap boundary has no finite projection", decaderr.ErrUnsupported)
	}
	// On each true source ray, the area density of a conical strip is
	// t*|(p-a)×p'|. Between ray fractions l and u, the centroid fraction
	// from the lower cap is (2u+l)/(3(u+l)), always in [1/2, 2/3]. A side
	// face's centroid is a positive-weight average over any subset of rays.
	// Evaluate both endpoint fractions so a reversed projection also works.
	sideLo := min(PointCapAffineDown(lowerLo, upperLo, 1, 1, 2),
		PointCapAffineDown(lowerLo, upperLo, 1, 2, 3))
	sideHi := max(PointCapAffineUp(lowerHi, upperHi, 1, 1, 2),
		PointCapAffineUp(lowerHi, upperHi, 1, 2, 3))
	if startHi < sideLo && sideHi < endLo {
		return 1, nil
	}
	if endHi < sideLo && sideHi < startLo {
		return -1, nil
	}
	return 0, fmt.Errorf("%w: cap centroid intervals do not separate every side centroid", decaderr.ErrUnsupported)
}
