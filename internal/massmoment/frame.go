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

// PlaneMap is the exact linear map L = B·[U V U×V] that carries plane
// coordinates (u, v, n) to world directions: the frame's held U and V, their
// exact cross product, and the placement's held basis B. It is
// RevolveRotation's construction about the plane's own U axis, since the local
// basis (W, E0, E1) is then (U, V, U×V).
func PlaneMap(frame r3.Frame, xform r3.Transform) ([3][3]*big.Rat, error) {
	return RevolveRotation(frame, 1, 0, xform)
}

// Determinant is the exact determinant of m.
func Determinant(m [3][3]*big.Rat) *big.Rat {
	minor := func(i, j, k, l int) *big.Rat {
		return new(big.Rat).Sub(proofbound.RatMul(m[i][k], m[j][l]), proofbound.RatMul(m[i][l], m[j][k]))
	}
	det := proofbound.RatMul(m[0][0], minor(1, 2, 1, 2))
	det.Sub(det, proofbound.RatMul(m[0][1], minor(1, 2, 0, 2)))
	return det.Add(det, proofbound.RatMul(m[0][2], minor(1, 2, 0, 1)))
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

// MapCharge is what a reading taken in plane coordinates owes the linear map
// L the record denotes through (PlaneMap, PrismRotation): the held frame and
// placement read as exact rationals, which r3 keeps orthonormal only to
// rounding. Volume is ||det L| − 1|, rounded up: L scales every volume by
// exactly |det L|. Stretch is the orthonormality defect e of L, the
// entrywise absolute sum of LᵀL − I, rounded up: every eigenvalue of LᵀL lies
// in [1 − e, 1 + e], so L scales an area, and for e ≤ 1 a length, by a factor
// in [1 − e, 1 + e]. Both are zero for an exactly orthonormal L, and every
// widening below is then a no-op.
type MapCharge struct {
	Volume, Stretch float64
}

// MapChargeOf reads l's MapCharge, refusing a map whose defect reaches 1/2,
// where the length factor's range no longer holds with margin. r3's own 1e-9
// orthonormality admission keeps every real frame far below it.
func MapChargeOf(l [3][3]*big.Rat) (MapCharge, error) {
	defect := OrthonormalityDefect(l)
	if defect.Cmp(big.NewRat(1, 2)) >= 0 {
		return MapCharge{}, fmt.Errorf(`%w: the placed frame departs from orthonormal by %s`, decaderr.ErrUnsupported, defect.FloatString(3))
	}
	det := Determinant(l)
	det.Abs(det).Sub(det, big.NewRat(1, 1))
	return MapCharge{
		Volume:  proofbound.RatFloatUp(det.Abs(det)),
		Stretch: proofbound.RatFloatUp(defect),
	}, nil
}

// VolumeOf widens a plane-coordinate volume reading to cover its image.
func (c MapCharge) VolumeOf(x proofbound.BoundedScalar) proofbound.BoundedScalar {
	return proofbound.BoundedStretch(x, c.Volume)
}

// AreaOf widens a plane-coordinate area reading to cover its image.
func (c MapCharge) AreaOf(x proofbound.BoundedScalar) proofbound.BoundedScalar {
	return proofbound.BoundedStretch(x, c.Stretch)
}

// LengthBound is the bound a plane-coordinate length and its bound carry
// once widened to cover the curve's image.
func (c MapCharge) LengthBound(value, bound float64) float64 {
	return proofbound.BoundedStretch(proofbound.MeasuredScalar(value, bound), c.Stretch).Bound
}
