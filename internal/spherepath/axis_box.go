package spherepath

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// AxisFaceCorridor selects the first exterior box face and proves the ball's
// projected disk stays strictly within that face for the full affine path.
// The normal gap is Start + f*Slope.
func AxisFaceCorridor(sphere box.AxisSphere, startBox box.AxisBox,
	sphereDelta, boxDelta [3]proof.Dyadic) (int, int, proof.Dyadic, proof.Dyadic, bool) {
	for axis := range 3 {
		if proof.DyCmp(sphere.Center[axis], startBox.Hi[axis]) > 0 {
			gap := proof.DySubScalar(proof.DySubScalar(sphere.Center[axis], startBox.Hi[axis]), sphere.Radius)
			if gap.Sign() < 0 {
				return 0, 0, proof.Dyadic{}, proof.Dyadic{}, false
			}
			slope := proof.DySubScalar(sphereDelta[axis], boxDelta[axis])
			return axis, 1, gap, slope, axisFaceProjectionInside(sphere, startBox, sphereDelta, boxDelta, axis)
		}
		if proof.DyCmp(sphere.Center[axis], startBox.Lo[axis]) < 0 {
			gap := proof.DySubScalar(proof.DySubScalar(startBox.Lo[axis], sphere.Center[axis]), sphere.Radius)
			if gap.Sign() < 0 {
				return 0, 0, proof.Dyadic{}, proof.Dyadic{}, false
			}
			slope := proof.DySubScalar(boxDelta[axis], sphereDelta[axis])
			return axis, -1, gap, slope, axisFaceProjectionInside(sphere, startBox, sphereDelta, boxDelta, axis)
		}
	}
	return 0, 0, proof.Dyadic{}, proof.Dyadic{}, false
}

func axisFaceProjectionInside(sphere box.AxisSphere, startBox box.AxisBox,
	sphereDelta, boxDelta [3]proof.Dyadic, normalAxis int) bool {
	for axis := range 3 {
		if axis == normalAxis {
			continue
		}
		for endpoint := range 2 {
			center, lo, hi := sphere.Center[axis], startBox.Lo[axis], startBox.Hi[axis]
			if endpoint == 1 {
				center = proof.DyAdd(center, sphereDelta[axis])
				lo = proof.DyAdd(lo, boxDelta[axis])
				hi = proof.DyAdd(hi, boxDelta[axis])
			}
			if proof.DyCmp(proof.DySubScalar(center, sphere.Radius), lo) <= 0 ||
				proof.DyCmp(proof.DyAdd(center, sphere.Radius), hi) >= 0 {
				return false
			}
		}
	}
	return true
}

// AxisTrackPointsWithin bounds conversion of a sphere-box contact coordinate
// by one outward ULP at the largest start or end support coordinate.
func AxisTrackPointsWithin(sphere box.AxisSphere, startBox box.AxisBox,
	sphereDelta, boxDelta [3]proof.Dyadic, resolution float64) bool {
	maximum := new(big.Rat)
	for axis := range 3 {
		endCenter := proof.DyAdd(sphere.Center[axis], sphereDelta[axis])
		for _, v := range []proof.Dyadic{
			sphere.Center[axis], endCenter,
			proof.DyAdd(sphere.Center[axis], sphere.Radius),
			proof.DySubScalar(sphere.Center[axis], sphere.Radius),
			proof.DyAdd(endCenter, sphere.Radius),
			proof.DySubScalar(endCenter, sphere.Radius),
			startBox.Lo[axis], startBox.Hi[axis],
			proof.DyAdd(startBox.Lo[axis], boxDelta[axis]),
			proof.DyAdd(startBox.Hi[axis], boxDelta[axis]),
		} {
			abs := new(big.Rat).Abs(v.Rat())
			if abs.Cmp(maximum) > 0 {
				maximum = abs
			}
		}
	}
	maxFloat := proofbound.RatFloatUp(maximum)
	if math.IsNaN(maxFloat) || math.IsInf(maxFloat, 0) {
		return false
	}
	return proofbound.Radius3D(math.Nextafter(maxFloat, math.Inf(1))-maxFloat) <= resolution
}

// AxisSpherePoseDeviation bounds the L1 difference between a rounded sphere
// center and box and their exact affine positions at fraction f.
func AxisSpherePoseDeviation(sphere box.AxisSphere, observedCenter proof.DyV3,
	startBox, observedBox box.AxisBox, sphereDelta, boxDelta [3]proof.Dyadic, f *big.Rat) *big.Rat {
	deviation := box.PoseDeviation(startBox, observedBox, boxDelta, f)
	for axis := range 3 {
		move := new(big.Rat).Mul(sphereDelta[axis].Rat(), f)
		center := new(big.Rat).Add(sphere.Center[axis].Rat(), move)
		diff := new(big.Rat).Sub(observedCenter[axis].Rat(), center)
		deviation.Add(deviation, diff.Abs(diff))
	}
	return deviation
}
