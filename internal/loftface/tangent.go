package loftface

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// TangentEnd is the immutable construction of a cone-tangent LoftFaces end:
// the plane that touches the cone x + Slope*r = Apex along one ruling. The
// map is rational because its direction e is a half-angle parameterised
// unit vector, so e.y^2 + e.z^2 = 1 holds exactly. Callers store a
// *TangentEnd and never mutate it.
type TangentEnd struct {
	Apex, Slope *big.Rat // copies of the cone's exact apex X and slope
	T, Rho      *big.Rat // half-angle parameter and axial scale
	Reflect     bool     // e_y = -(1-T^2)/(1+T^2) when set
}

// NewTangentEnd builds the tangent end of the cone with exact apex and slope
// whose touching ruling lies along the float direction (radialY, radialZ).
// The float arithmetic here only chooses T and Rho; every identity of the
// resulting map is exact for the chosen rationals. Both inputs are copied.
func NewTangentEnd(apex, slope *big.Rat, radialY, radialZ float64) (TangentEnd, error) {
	if apex == nil || slope == nil || slope.Sign() <= 0 {
		return TangentEnd{}, fmt.Errorf("%w: cone-tangent end needs a cone with a positive slope",
			decaderr.ErrDegenerate)
	}
	// Scale first so that neither Hypot nor the half-angle denominator can
	// overflow or underflow. Only the direction matters, and the scaling
	// only chooses T.
	scale := math.Max(math.Abs(radialY), math.Abs(radialZ))
	if proofbound.IsNonFinite(radialY) || proofbound.IsNonFinite(radialZ) || scale == 0 {
		return TangentEnd{}, fmt.Errorf("%w: cone-tangent end needs a finite nonzero radial direction",
			decaderr.ErrDegenerate)
	}
	y, z := radialY/scale, radialZ/scale
	n := math.Hypot(y, z)
	reflect := y < 0
	den := y + n
	if reflect {
		den = n - y
	}
	t := z / den
	if proofbound.IsNonFinite(n) || n == 0 || den == 0 || proofbound.IsNonFinite(den) || proofbound.IsNonFinite(t) {
		return TangentEnd{}, fmt.Errorf("%w: cone-tangent end radial direction has no finite half-angle",
			decaderr.ErrDegenerate)
	}
	kf, _ := slope.Float64()
	rho := 1 / math.Sqrt(1+kf*kf)
	if proofbound.IsNonFinite(kf) || proofbound.IsNonFinite(rho) || rho <= 0 {
		return TangentEnd{}, fmt.Errorf("%w: cone-tangent end parameters are not finite", decaderr.ErrDegenerate)
	}
	return TangentEnd{Apex: new(big.Rat).Set(apex), Slope: new(big.Rat).Set(slope),
		T: ratF(t), Rho: ratF(rho), Reflect: reflect}, nil
}

// direction returns the exact unit vector (e_y, e_z).
func (e TangentEnd) direction() (*big.Rat, *big.Rat) {
	t2 := ratMul(e.T, e.T)
	den := ratAdd(ratOne(), t2)
	c := ratDiv(ratSub(ratOne(), t2), den)
	s := ratDiv(ratMul(big.NewRat(2, 1), e.T), den)
	if e.Reflect {
		c = new(big.Rat).Neg(c)
	}
	return c, s
}

// Map returns the exact cap map P(u, v) = (Apex, 0, 0) + u*Rho*(k, -e_y, -e_z)
// + v*(0, -e_z, e_y) with constant denominator 1.
func (e TangentEnd) Map() Map {
	ey, ez := e.direction()
	neg := func(a *big.Rat) *big.Rat { return new(big.Rat).Neg(a) }
	qu := RatVec{ratMul(e.Rho, e.Slope), ratMul(e.Rho, neg(ey)), ratMul(e.Rho, neg(ez))}
	qv := RatVec{ratZero(), neg(ez), ey}
	norm2 := ratAdd(VDot(qu, qu), VDot(qv, qv))
	return Map{
		Constant: RatVec{new(big.Rat).Set(e.Apex), ratZero(), ratZero()},
		Q0:       RatVec{ratZero(), ratZero(), ratZero()},
		QU:       qu, QV: qv,
		G0: ratOne(), GU: ratZero(), GV: ratZero(),
		// The Frobenius norm bounds the operator norm of the linear part.
		Stretch: proofbound.RatSqrtUp(norm2),
	}
}

// Equal reports whether both ends hold exactly the same construction.
func (e TangentEnd) Equal(other TangentEnd) bool {
	return e.Reflect == other.Reflect && ratEq(e.Apex, other.Apex) && ratEq(e.Slope, other.Slope) &&
		ratEq(e.T, other.T) && ratEq(e.Rho, other.Rho)
}

// MatchesCone reports whether the end was built for exactly this cone.
func (e TangentEnd) MatchesCone(apex, slope *big.Rat) bool {
	return ratEq(e.Apex, apex) && ratEq(e.Slope, slope)
}

func ratEq(a, b *big.Rat) bool { return a != nil && b != nil && a.Cmp(b) == 0 }

// ConePullback returns the exact pullback of the cone x + slope*r = apex to
// the map's cap plane: the quadratic slope^2*(y^2+z^2) - (apex-x)^2 after
// multiplying through the squared positive map denominator. The coefficients
// are those of the monomials 1, u, v, u^2, u*v, v^2 in that order.
func ConePullback(m Map, apex, slope *big.Rat) [6]*big.Rat {
	g := [3]*big.Rat{m.G0, m.GU, m.GV}
	coordinate := func(axis int) [3]*big.Rat {
		return [3]*big.Rat{
			ratAdd(ratMul(m.Constant[axis], g[0]), m.Q0[axis]),
			ratAdd(ratMul(m.Constant[axis], g[1]), m.QU[axis]),
			ratAdd(ratMul(m.Constant[axis], g[2]), m.QV[axis]),
		}
	}
	x, y, z := coordinate(0), coordinate(1), coordinate(2)
	d := [3]*big.Rat{}
	for i := range d {
		d[i] = ratSub(ratMul(apex, g[i]), x[i])
	}
	two := big.NewRat(2, 1)
	quad := func(v [3]*big.Rat) [6]*big.Rat {
		return [6]*big.Rat{ratMul(v[0], v[0]), ratMul(two, ratMul(v[0], v[1])),
			ratMul(two, ratMul(v[0], v[2])), ratMul(v[1], v[1]),
			ratMul(two, ratMul(v[1], v[2])), ratMul(v[2], v[2])}
	}
	y2, z2, d2 := quad(y), quad(z), quad(d)
	k2 := ratMul(slope, slope)
	var q [6]*big.Rat
	for i := range q {
		q[i] = ratSub(ratMul(k2, ratAdd(y2[i], z2[i])), d2[i])
	}
	return q
}

// ConeStrictOutsideForm reports whether the pullback q has the form
// A*u^2 + C*v^2 with A, C > 0 and no other term.
func ConeStrictOutsideForm(q [6]*big.Rat) bool {
	return q[0].Sign() == 0 && q[1].Sign() == 0 && q[2].Sign() == 0 &&
		q[4].Sign() == 0 && q[3].Sign() > 0 && q[5].Sign() > 0
}

// MapEqual reports whether two maps hold exactly the same coefficients.
func MapEqual(a, b Map) bool {
	for i := range a.Constant {
		if !ratEq(a.Constant[i], b.Constant[i]) || !ratEq(a.Q0[i], b.Q0[i]) ||
			!ratEq(a.QU[i], b.QU[i]) || !ratEq(a.QV[i], b.QV[i]) {
			return false
		}
	}
	return ratEq(a.G0, b.G0) && ratEq(a.GU, b.GU) && ratEq(a.GV, b.GV) && a.Stretch == b.Stretch
}

// RoundPoint returns the float64 nearest each coordinate of p.
func RoundPoint(p RatVec) r3.Vec {
	x, _ := p[0].Float64()
	y, _ := p[1].Float64()
	z, _ := p[2].Float64()
	return r3.Vec{X: x, Y: y, Z: z}
}
