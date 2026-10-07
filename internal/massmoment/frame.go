package massmoment

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RevolveAnchor is the axis anchor before placement, read exactly from held floats.
func RevolveAnchor(frame r3.Frame, aUFloat, aVFloat float64) ([3]*big.Rat, error) {
	aU, aV := proofarith.FloatRat(aUFloat), proofarith.FloatRat(aVFloat)
	if aU == nil || aV == nil {
		return [3]*big.Rat{}, fmt.Errorf("%w: revolve axis anchor is not finite", decaderr.ErrNotFinite)
	}
	origin, u, v := frame.Origin(), frame.U(), frame.V()
	var out [3]*big.Rat
	for i := range out {
		o := proofarith.FloatRat(vecComponent(origin, i))
		ui, vi := proofarith.FloatRat(vecComponent(u, i)), proofarith.FloatRat(vecComponent(v, i))
		if o == nil || ui == nil || vi == nil {
			return [3]*big.Rat{}, fmt.Errorf("%w: revolve frame is not finite", decaderr.ErrNotFinite)
		}
		out[i] = proofbound.RatAdd(o, proofbound.RatMul(aU, ui), proofbound.RatMul(aV, vi))
	}
	return out, nil
}

func vecComponent(v r3.Vec, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// RevolveRotation maps the local axis basis to world directions as exact rationals.
func RevolveRotation(frame r3.Frame, dUFloat, dVFloat float64, xform r3.Transform) ([3][3]*big.Rat, error) {
	vec := func(v r3.Vec) ([3]*big.Rat, bool) {
		out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
		return out, out[0] != nil && out[1] != nil && out[2] != nil
	}
	u, okU := vec(frame.U())
	v, okV := vec(frame.V())
	dU, dV := proofarith.FloatRat(dUFloat), proofarith.FloatRat(dVFloat)
	basis := xform.Basis()
	ex, okX := vec(basis.EX)
	ey, okY := vec(basis.EY)
	ez, okZ := vec(basis.EZ)
	if !okU || !okV || dU == nil || dV == nil || !okX || !okY || !okZ {
		return [3][3]*big.Rat{}, fmt.Errorf("%w: revolve orientation is not finite", decaderr.ErrNotFinite)
	}
	var w, e0, e1 [3]*big.Rat
	for i := range 3 {
		w[i] = proofbound.RatAdd(proofbound.RatMul(u[i], dU), proofbound.RatMul(v[i], dV))
		e0[i] = new(big.Rat).Sub(proofbound.RatMul(v[i], dU), proofbound.RatMul(u[i], dV))
	}
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		e1[i] = new(big.Rat).Sub(proofbound.RatMul(w[j], e0[k]), proofbound.RatMul(w[k], e0[j]))
	}
	placement := [3][3]*big.Rat{ex, ey, ez}
	var out [3][3]*big.Rat
	for i := range 3 {
		for k, local := range [3][3]*big.Rat{w, e0, e1} {
			sum := new(big.Rat)
			for l := range 3 {
				sum.Add(sum, proofbound.RatMul(placement[l][i], local[l]))
			}
			out[i][k] = sum
		}
	}
	return out, nil
}

// PlacementRotation reads the exact rational linear part of a placement.
func PlacementRotation(placement r3.Transform) ([3][3]*big.Rat, error) {
	basis := placement.Basis()
	var out [3][3]*big.Rat
	for k, column := range []r3.Vec{basis.EX, basis.EY, basis.EZ} {
		exact, ok := ExactVec(column)
		if !ok {
			return out, fmt.Errorf("%w: placement basis is not finite", decaderr.ErrNotFinite)
		}
		for i := range exact {
			out[i][k] = exact[i]
		}
	}
	return out, nil
}

// ExactVec reads the held components of v as rational numbers.
func ExactVec(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}
