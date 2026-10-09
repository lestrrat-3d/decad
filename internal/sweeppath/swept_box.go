package sweeppath

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SweptBounds holds exact coordinate extremes over one whole path.
type SweptBounds struct {
	Lo, Hi [3]proofarith.Dyadic
}

// SweepBounds encloses the From-mapped inflated corners over the whole path.
// mapPoint applies the exact held transform, and radius bounds distance from
// a rotating path's axis. A translating path takes its exact endpoint hull.
func SweepBounds(corners [8]proofarith.DyV3, path AffinePath,
	mapPoint func(r3.Transform, proofarith.DyV3) proofarith.DyV3,
	radius func(r3.Transform, r3.Vec, motionbound.RatVec) (*big.Rat, bool),
) (SweptBounds, bool) {
	var out SweptBounds
	for i, corner := range corners {
		mapped := mapPoint(path.From, corner)
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(mapped[axis], out.Lo[axis]) < 0 {
				out.Lo[axis] = mapped[axis]
			}
			if i == 0 || proofarith.DyCmp(mapped[axis], out.Hi[axis]) > 0 {
				out.Hi[axis] = mapped[axis]
			}
		}
	}
	var travel *big.Rat
	var ok bool
	switch {
	case path.Drift != nil:
		travel, ok = driftTravel(path, radius)
	case path.Read != nil:
		travel, ok = screwTravel(path, radius)
	default:
		for axis := range 3 {
			shiftedLo := proofarith.DyAdd(out.Lo[axis], path.Delta[axis])
			shiftedHi := proofarith.DyAdd(out.Hi[axis], path.Delta[axis])
			if proofarith.DyCmp(shiftedLo, out.Lo[axis]) < 0 {
				out.Lo[axis] = shiftedLo
			}
			if proofarith.DyCmp(shiftedHi, out.Hi[axis]) > 0 {
				out.Hi[axis] = shiftedHi
			}
		}
		return out, true
	}
	if !ok {
		return SweptBounds{}, false
	}
	up := proofbound.RatFloatUp(travel)
	if !allFinite(up) {
		return SweptBounds{}, false
	}
	grow := proofarith.MustDyOf(up)
	for axis := range 3 {
		out.Lo[axis] = proofarith.DySubScalar(out.Lo[axis], grow)
		out.Hi[axis] = proofarith.DyAdd(out.Hi[axis], grow)
	}
	return out, true
}

// driftTravel is (V + ρΩ)·Duration over a rigid drift's whole path.
func driftTravel(path AffinePath,
	radius func(r3.Transform, r3.Vec, motionbound.RatVec) (*big.Rat, bool),
) (*big.Rat, bool) {
	drift := path.Drift
	linear := [3]units.Value{drift.LinearVelocity.X, drift.LinearVelocity.Y, drift.LinearVelocity.Z}
	angular := [3]units.Value{drift.AngularVelocity.X, drift.AngularVelocity.Y, drift.AngularVelocity.Z}
	var omega motionbound.RatVec
	vSquared, omegaSquared := new(big.Rat), new(big.Rat)
	for axis := range 3 {
		v, okV := ExactBaseValue(linear[axis])
		w, okW := ExactBaseValue(angular[axis])
		if !okV || !okW {
			return nil, false
		}
		omega[axis] = w
		vSquared.Add(vSquared, new(big.Rat).Mul(v, v))
		omegaSquared.Add(omegaSquared, new(big.Rat).Mul(w, w))
	}
	speed, spin := proofbound.RatSqrtUp(vSquared), proofbound.RatSqrtUp(omegaSquared)
	if !allFinite(speed, spin) {
		return nil, false
	}
	rho, ok := radius(path.From, drift.Center, omega)
	if !ok {
		return nil, false
	}
	rate := new(big.Rat).Add(proofarith.FloatRat(speed), new(big.Rat).Mul(rho, proofarith.FloatRat(spin)))
	return rate.Mul(rate, path.Duration), true
}

// screwTravel is ρ|θ| + |d| over a read screw's whole path.
func screwTravel(path AffinePath,
	radius func(r3.Transform, r3.Vec, motionbound.RatVec) (*big.Rat, bool),
) (*big.Rat, bool) {
	screw := path.Read
	angle, okAngle := ExactBaseValue(screw.Angle)
	slide := proofarith.FloatRat(screw.Slide)
	if !okAngle || slide == nil {
		return nil, false
	}
	travel := new(big.Rat).Abs(slide)
	if angle.Sign() == 0 {
		return travel, true
	}
	axis, ok := motionbound.RatVecOf(screw.Axis)
	if !ok {
		return nil, false
	}
	rho, ok := radius(path.From, screw.Point, axis)
	if !ok {
		return nil, false
	}
	return travel.Add(travel, new(big.Rat).Mul(rho, angle.Abs(angle))), true
}

// InflatedBoundsCorners returns the exact corners of a held box expanded by
// its nonnegative bound. The caller validates the bound and coordinates.
func InflatedBoundsCorners(minimum, maximum r3.Vec, bound float64) ([8]proofarith.DyV3, bool) {
	var corners [8]proofarith.DyV3
	lo := [3]float64{minimum.X, minimum.Y, minimum.Z}
	hi := [3]float64{maximum.X, maximum.Y, maximum.Z}
	var extremes [3][2]proofarith.Dyadic
	widen := proofarith.MustDyOf(bound)
	for axis := range 3 {
		if lo[axis] > hi[axis] {
			return corners, false
		}
		extremes[axis] = [2]proofarith.Dyadic{
			proofarith.DySubScalar(proofarith.MustDyOf(lo[axis]), widen),
			proofarith.DyAdd(proofarith.MustDyOf(hi[axis]), widen),
		}
	}
	for index := range corners {
		for axis := range 3 {
			corners[index][axis] = extremes[axis][(index>>axis)&1]
		}
	}
	return corners, true
}
