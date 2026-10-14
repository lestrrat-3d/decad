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

// tangentPullbackForm reports whether q is exactly C*v^2 with C > 0: the
// pullback of a plane that is nowhere inside the cone and meets it, at
// most, along the ruling v = 0. It takes no slope, so it accepts any C > 0.
func tangentPullbackForm(q [6]*big.Rat) bool {
	for _, i := range []int{0, 1, 2, 3, 4} {
		if q[i].Sign() != 0 {
			return false
		}
	}
	return q[5].Sign() > 0
}

// ConeTangentAlongRuling reports whether the cap of m has no point in the
// open interior of the cone x + slope*r < apex over the source box. It is a
// pullback and non-intrusion check, not a contact check. It requires:
//
//  1. slope > 0, which the inequality needs: for slope <= 0 the squared
//     pullback no longer separates the cone's nappe from the opposite one;
//  2. the denominator-cleared pullback is exactly C*v^2 with C > 0, the
//     strict form with A = 0;
//  3. the box lies strictly on one side of u = 0.
//
// Then k^2 r^2 - (apex-x)^2 >= 0 over the cap, with equality only on v = 0,
// and the one-sided box excludes the apex point of the issued map. The check
// does not prove that the cap touches the nappe, that the map is not
// collapsed, or that the apex is excluded for an arbitrary map. Any apex or
// ruling-contact reading needs the caller's authenticated map, the tangent
// end record. The caller has proved the map denominator positive over the
// box.
func ConeTangentAlongRuling(m Map, box [4]*big.Rat, apex, slope *big.Rat) bool {
	if apex == nil || slope == nil || slope.Sign() <= 0 || box[0].Sign() <= 0 && box[1].Sign() >= 0 {
		return false
	}
	return tangentPullbackForm(ConePullback(m, apex, slope))
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

// TangentPlane is the exact plane Pi of a cone-tangent cap: phi(P) = N.P - D.
// Side is the strict sign of phi on the loft side of the plane, the sign that
// phi takes on the preceding cap. Side*phi is therefore positive on the side
// where the cone lies.
type TangentPlane struct {
	N    RatVec
	D    *big.Rat
	Side int
}

// Phi returns the exact plane functional N.p - D.
func (p TangentPlane) Phi(point RatVec) *big.Rat {
	return ratSub(VDot(p.N, point), p.D)
}

// Psi returns Side*phi(point): positive on the cone's side of the plane.
func (p TangentPlane) Psi(point RatVec) *big.Rat {
	value := p.Phi(point)
	if p.Side < 0 {
		value.Neg(value)
	}
	return value
}

// capPlane returns the exact plane of an affine cap map: constant
// denominator, N = QU x QV nonzero, D = N.(Constant + Q0/G0). The side is
// left at zero. It reports false for a projective or collapsed map.
func capPlane(m Map) (TangentPlane, bool) {
	if m.G0 == nil || m.G0.Sign() <= 0 || m.GU == nil || m.GU.Sign() != 0 || m.GV == nil || m.GV.Sign() != 0 {
		return TangentPlane{}, false
	}
	n := VCross(m.QU, m.QV)
	if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
		return TangentPlane{}, false
	}
	origin := VAdd(m.Constant, VScale(m.Q0, ratDiv(ratOne(), m.G0)))
	return TangentPlane{N: n, D: VDot(n, origin)}, true
}

// NewTangentPlane returns the plane of the affine cap map final with the
// given loft side, which must be -1 or +1.
func NewTangentPlane(final Map, side int) (TangentPlane, bool) {
	if side != -1 && side != 1 {
		return TangentPlane{}, false
	}
	plane, ok := capPlane(final)
	plane.Side = side
	return plane, ok
}

// ConeOnLoftSide is the exact side gate of a cone-tangent cap. final is the
// tangent cap map and previous the cap map before it, whose box corners
// give the loft side. It requires:
//
//  1. slope > 0, which the open-interior argument below needs;
//  2. the pullback of the cone x + slope*r = apex to final's plane is
//     exactly C*v^2 with C > 0, so the plane meets no interior point of the
//     cone;
//  3. apex > far, and the axis point m = ((apex+far)/2, 0, 0), which is in
//     the open interior of the finite cone, is not on the plane;
//  4. previous has a positive denominator over the box and phi of one
//     strict sign s at all four box corners;
//  5. s*phi(m) > 0.
//
// The interior of the finite cone is convex and misses the plane, so it
// lies wholly on m's side, which is then the loft side s. It returns s.
func ConeOnLoftSide(final, previous Map, box [4]*big.Rat, apex, slope, far *big.Rat) (int, bool) {
	if apex == nil || slope == nil || far == nil || slope.Sign() <= 0 || apex.Cmp(far) <= 0 {
		return 0, false
	}
	if !tangentPullbackForm(ConePullback(final, apex, slope)) {
		return 0, false
	}
	plane, ok := capPlane(final)
	if !ok || PositiveDenominator(previous, box) != nil {
		return 0, false
	}
	side := 0
	for _, corner := range BoxCorners(box) {
		sign := plane.Phi(previous.At(corner[0], corner[1])).Sign()
		if sign == 0 || (side != 0 && sign != side) {
			return 0, false
		}
		side = sign
	}
	mid := ratDiv(ratAdd(apex, far), big.NewRat(2, 1))
	if plane.Phi(RatVec{mid, ratZero(), ratZero()}).Sign() != side {
		return 0, false
	}
	return side, true
}

// CapsOnSide is the exact earlier-cap gate of a cone-tangent cap (G2). final is
// the tangent cap map and side the strict loft side that ConeOnLoftSide
// returned. Every map in earlier must have a positive denominator over the
// box and phi of sign side at all four box corners.
//
// The numerator of phi after clearing the positive denominator is affine in
// (u, v), so strict positivity of side*phi at the four corners holds over the
// whole box: every earlier cap lies strictly in {side*phi > 0}. On band i the
// loft point is (1-t)*C_i(p) + t*C_(i+1)(p) and phi is affine, so with every
// earlier cap strictly positive the loft lies in {side*phi >= 0} and meets the
// plane only in the end cap itself.
func CapsOnSide(final Map, earlier []Map, box [4]*big.Rat, side int) bool {
	if side != -1 && side != 1 || len(earlier) == 0 {
		return false
	}
	plane, ok := capPlane(final)
	if !ok {
		return false
	}
	for _, m := range earlier {
		if PositiveDenominator(m, box) != nil {
			return false
		}
		for _, corner := range BoxCorners(box) {
			if plane.Phi(m.At(corner[0], corner[1])).Sign() != side {
				return false
			}
		}
	}
	return true
}
