package twistsweep

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// placementCharge is what a twisted sweep's plane-local readings owe its
// placement. Every admitted twist sketches on the world XY frame through the
// origin (Eval, TwistCardinalPathOf), so the map its record denotes through is
// the placement's held basis B alone, read as exact rationals. The volume,
// area and boundary closed forms are plane-local, while r3 keeps B
// orthonormal only to rounding and a placement composed through
// r3.Transform.Then drifts further (agent-docs/sweep-design.md §17). An
// exactly orthonormal B, such as a translation's identity, charges nothing.
func placementCharge(xform r3.Transform) (massmoment.MapCharge, error) {
	linear, err := massmoment.PlacementRotation(xform)
	if err != nil {
		return massmoment.MapCharge{}, err
	}
	return massmoment.MapChargeOf(linear)
}

// chargeBoundary widens a twist's boundary displacement delta and occupied
// symmetric-difference volume to cover their images under the placement.
// Both mix plane-local terms with the held Loft's own placed terms; charging
// the whole of each over-covers the placed terms by the same tiny factor.
func chargeBoundary(charge massmoment.MapCharge, delta, volSymDiff float64) (float64, float64) {
	return charge.LengthBound(0, delta),
		charge.VolumeOf(proofbound.MeasuredScalar(0, volSymDiff)).Bound
}

// twistShellPlacedGap compares the affine image of a local shell with the
// denoted world-normal shell on an almost rigidly placed source. Every amount
// is expressed in local coordinates, before facetedPayload.Placed applies L.
type twistShellPlacedGap struct {
	Boundary float64
	Volume   float64
	Area     float64
	Defect   float64
}

// TwistShellPlacedProof uses the exact stored placement matrix to enclose its
// metric defect. Let G=LᵀL and e=||G-I||<1/2. The pullback of a world-unit
// normal is M(N)=G⁻¹N/sqrt(N·G⁻¹N). For a local unit normal N,
// |M(N)-N|<=4e/(1-e). On a unit tangent to the normal sphere,
// |DM(N)-I|<=100e/(1-e)^3; the loose constant follows by bounding the two
// terms of DM with the eigenvalue interval [1/(1+e),1/(1-e)].
//
// The source wall's height after offset is strictly increasing in its source
// height when eta<1. Its section at every height is a simple star-shaped
// curve when the old radial cross-product margin exceeds the C1 change.
// Clearance from the outer wall and a positive floor then give one cup with
// the same cap/rim incidence. The returned charges compare each corresponding
// boundary patch; neither area nor occupied volume uses signed cancellation.
func TwistShellPlacedProof(profile profileRecord, height, thickness float64,
	angle units.Value, source twistOffsetSourceProof, motion r3.Transform) (twistShellPlacedGap, error) {
	linear, err := massmoment.PlacementRotation(motion)
	if err != nil {
		return twistShellPlacedGap{}, err
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return twistShellPlacedGap{}, err
	}
	e := charge.Stretch
	if e < 0 || e >= 0.5 || proofbound.IsNonFinite(e) ||
		proofbound.IsNonFinite(height) || proofbound.IsNonFinite(thickness) ||
		height <= thickness || thickness <= 0 || source.ClearanceLo == nil ||
		source.SpeedLo == nil || source.RadialLo == nil {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no metric or source margin`, decaderr.ErrUnsupported)
	}
	denom := math.Nextafter(1-e, 0)
	if denom <= 0 {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no metric margin`, decaderr.ErrUnsupported)
	}
	poly, err := TwistPolygonOf(profile)
	if err != nil {
		return twistShellPlacedGap{}, err
	}
	theta, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no turn enclosure`, decaderr.ErrUnsupported)
	}
	k := proofbound.DivUpper(proofbound.RatFloatUp(proofbound.IntervalAbsUpper(theta)), height)
	radius := proofbound.AbsSumUpper(poly.MaxRadiusL1, thickness)
	kRadius := proofbound.ProductUpper(k, radius)
	mu := proofbound.DivUpper(proofbound.ProductUpper(100, e),
		proofbound.ProductUpper(denom, proofbound.ProductUpper(denom, denom)))
	delta := proofbound.DivUpper(proofbound.ProductUpper(4, proofbound.ProductUpper(thickness, e)), denom)
	eta := proofbound.ProductUpper(thickness, proofbound.ProductUpper(mu, k))
	if eta >= 1 {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell height can fold`, decaderr.ErrUnsupported)
	}
	heightDenom := math.Nextafter(1-eta, 0)
	floorGap := proofbound.DivUpper(proofbound.ProductUpper(thickness, e), denom)
	if !(heightDenom > 0) || proofbound.AbsSumUpper(thickness, floorGap) >= height {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no positive floor clearance`, decaderr.ErrUnsupported)
	}

	// Unit-speed source arclength is the common contour parameter. These
	// bounds cover every point of each authenticated Line or Arc span.
	var perimeter, oldContourSpeed, fixedContourSpeed, normalSpeed, oldHeightSpeed float64
	const parts = int64(64)
	for _, segment := range profile.Outer.Segments {
		for index := range parts {
			var span twistOffsetSpan
			switch edge := segment.(type) {
			case lineSeg:
				span, err = twistOffsetLineSpan(edge, index, index+1, parts)
			case arcSeg:
				span, err = twistOffsetArcSpan(edge, index, index+1, parts)
			default:
				return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell source has an unsupported edge`, decaderr.ErrUnsupported)
			}
			if err != nil {
				return twistShellPlacedGap{}, err
			}
			perimeter = proofbound.AbsSumUpper(perimeter, proofbound.RatFloatUp(span.length.Hi))
			f := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(span.f))
			h := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(span.h))
			curvature := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(span.kappa))
			fPrime := proofbound.AbsSumUpper(1, proofbound.ProductUpper(curvature, h))
			k2 := proofbound.ProductUpper(k, k)
			betaPrime := proofbound.ProductUpper(k2, proofbound.ProductUpper(thickness, fPrime))
			oldContourSpeed = max(oldContourSpeed, proofbound.AbsSumUpper(1,
				proofbound.ProductUpper(thickness, curvature),
				proofbound.ProductUpper(betaPrime, proofbound.AbsSumUpper(h, thickness))))
			fixedContourSpeed = max(fixedContourSpeed, proofbound.AbsSumUpper(1,
				proofbound.ProductUpper(thickness, curvature),
				proofbound.ProductUpper(proofbound.ProductUpper(thickness, k2),
					proofbound.ProductUpper(f, fPrime))))
			normalSpeed = max(normalSpeed, proofbound.AbsSumUpper(curvature,
				proofbound.ProductUpper(k, fPrime),
				proofbound.ProductUpper(k2, proofbound.ProductUpper(f, fPrime))))
			oldHeightSpeed = max(oldHeightSpeed,
				proofbound.ProductUpper(proofbound.ProductUpper(thickness, k), fPrime))
		}
	}
	if perimeter <= 0 {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no source perimeter`, decaderr.ErrUnsupported)
	}
	// At a fixed height, the local and pulled-back world offsets are at
	// most delta apart. The implicit height inverse moves source z by at
	// most delta. These are C0, contour-C1, and height-C1 gaps.
	positionGap := proofbound.ProductUpper(delta, proofbound.AbsSumUpper(1, kRadius))
	es := proofbound.ProductUpper(proofbound.ProductUpper(thickness, mu), normalSpeed)
	ez := eta
	zSpeedGap := proofbound.DivUpper(proofbound.AbsSumUpper(es,
		proofbound.ProductUpper(oldHeightSpeed, ez)), heightDenom)
	newHeightSpeed := proofbound.AbsSumUpper(oldHeightSpeed, zSpeedGap)
	contourGap := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(k, fixedContourSpeed), delta), es,
		proofbound.ProductUpper(proofbound.ProductUpper(k, kRadius),
			proofbound.ProductUpper(delta, newHeightSpeed)),
		proofbound.ProductUpper(ez, newHeightSpeed),
		proofbound.ProductUpper(kRadius, zSpeedGap))
	heightGap := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(k, kRadius), delta), ez,
		proofbound.DivUpper(proofbound.ProductUpper(
			proofbound.AbsSumUpper(kRadius, ez), eta), heightDenom))
	crossGap := proofbound.AbsSumUpper(
		proofbound.ProductUpper(radius, contourGap),
		proofbound.ProductUpper(oldContourSpeed, positionGap),
		proofbound.ProductUpper(positionGap, contourGap))
	crossLo := proofbound.RatFloatDown(new(big.Rat).Mul(source.SpeedLo, source.RadialLo))
	if !(crossGap < crossLo) ||
		!(positionGap < proofbound.RatFloatDown(source.ClearanceLo)) {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell loses radial or outer-wall clearance`, decaderr.ErrUnsupported)
	}

	// The straight contour homotopy stays star-shaped by the same cross
	// margin. Its swept area bounds symmetric difference in each section,
	// not merely the signed difference of their enclosed areas.
	sectionSweep := proofbound.ProductUpper(perimeter,
		proofbound.ProductUpper(positionGap,
			proofbound.AbsSumUpper(oldContourSpeed, contourGap)))
	boundary := proofbound.AbsSumUpper(positionGap,
		proofbound.ProductUpper(proofbound.AbsSumUpper(kRadius, heightGap, 1), floorGap))
	volume := proofbound.AbsSumUpper(
		proofbound.ProductUpper(height, sectionSweep),
		proofbound.ProductUpper(floorGap,
			proofbound.AbsSumUpper(proofbound.RatFloatUp(source.Area.Hi), sectionSweep)))
	wallDensity := proofbound.AbsSumUpper(
		proofbound.ProductUpper(contourGap, proofbound.AbsSumUpper(1, kRadius)),
		proofbound.ProductUpper(heightGap, oldContourSpeed),
		proofbound.ProductUpper(contourGap, heightGap))
	area := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(perimeter, height), wallDensity),
		proofbound.ProductUpper(proofbound.ProductUpper(perimeter, floorGap),
			proofbound.ProductUpper(proofbound.AbsSumUpper(oldContourSpeed, contourGap),
				proofbound.AbsSumUpper(1, kRadius, heightGap))),
		proofbound.ProductUpper(2, sectionSweep))
	if proofbound.IsNonFinite(boundary) || proofbound.IsNonFinite(volume) ||
		proofbound.IsNonFinite(area) {
		return twistShellPlacedGap{}, fmt.Errorf(`%w: placed twisted shell has no finite source charges`, decaderr.ErrUnsupported)
	}
	return twistShellPlacedGap{Boundary: boundary, Volume: volume,
		Area: area, Defect: e}, nil
}
