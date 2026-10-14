package extent

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
)

// RevolvedPlaneStop returns the exact sweep level of a straight revolved wall
// only when its recorded meridian has one fixed axial coordinate. The record
// encloses the real wall, including axis and section displacement; requiring
// every relevant interval to be a point leaves no uncharged plane tilt.
func RevolvedPlaneStop(r surfacenormal.Revolved, sweep r3.Frame) (*big.Rat, bool) {
	if !r.Valid || r.Cap || r.Circular || len(r.Ends) == 0 {
		return nil, false
	}
	var origin [3]*big.Rat
	var basis [3][3]*big.Rat
	for axis := range 3 {
		v, ok := intervalPoint(r.Origin[axis])
		if !ok {
			return nil, false
		}
		origin[axis] = v
		for i := range 3 {
			v, ok = intervalPoint(r.Basis[i][axis])
			if !ok {
				return nil, false
			}
			basis[i][axis] = v
		}
	}
	z, ok := intervalPoint(r.Ends[0][0][0])
	if !ok {
		return nil, false
	}
	for _, ends := range r.Ends {
		for _, end := range ends {
			at, exact := intervalPoint(end[0])
			if !exact || at.Cmp(z) != 0 {
				return nil, false
			}
		}
	}
	m := ratCross(basis[0], basis[1])
	if m[0].Sign() == 0 && m[1].Sign() == 0 && m[2].Sign() == 0 {
		return nil, false
	}
	su, sv, sn, so := ratVec(sweep.U()), ratVec(sweep.V()), ratVec(sweep.N()), ratVec(sweep.Origin())
	for _, v := range [][3]*big.Rat{su, sv, sn, so} {
		if v[0] == nil || v[1] == nil || v[2] == nil {
			return nil, false
		}
	}
	if ratDot(m, su).Sign() != 0 || ratDot(m, sv).Sign() != 0 {
		return nil, false
	}
	mn := ratDot(m, sn)
	if mn.Sign() == 0 {
		return nil, false
	}
	var rel [3]*big.Rat
	for i := range rel {
		rel[i] = new(big.Rat).Sub(proofbound.RatAdd(origin[i], proofbound.RatMul(basis[2][i], z)), so[i])
	}
	return new(big.Rat).Quo(ratDot(m, rel), mn), true
}

func intervalPoint(iv proofbound.RatInterval) (*big.Rat, bool) {
	if iv.Lo == nil || iv.Hi == nil || iv.Lo.Cmp(iv.Hi) != 0 {
		return nil, false
	}
	return iv.Lo, true
}
