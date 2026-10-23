package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// The interval contains the single flank/root crossing on one source cubic
// at one LoftFaces cap. Its endpoints are exact rationals with opposite
// implicit signs; no rounded point is treated as the true crossing.
type RootCrossing struct {
	Span   int
	Lo, Hi *big.Rat
}

// capPlaneAndCircle returns the exact plane functional phi(P) = normal.P - d
// of the cap plane through corners, and the terms of phi on the root circle
// of radius m*circleX+b at x = circleX. On that circle
// phi = a + y*cos(theta) + z*sin(theta), and yz2 = y^2 + z^2. It reports
// false for a collapsed plane (zero normal), four non-coplanar corners, or
// a circle radius that is not positive.
func capPlaneAndCircle(capMap Map, corners [4][2]*big.Rat,
	m, b, circleX *big.Rat) (normal RatVec, d, a, yz2 *big.Rat, ok bool) {
	p0 := capMap.At(corners[0][0], corners[0][1])
	p1 := capMap.At(corners[1][0], corners[1][1])
	p2 := capMap.At(corners[2][0], corners[2][1])
	normal = VCross(VSub(p1, p0), VSub(p2, p0))
	if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
		return normal, nil, nil, nil, false
	}
	d = VDot(normal, p0)
	if VDot(normal, capMap.At(corners[3][0], corners[3][1])).Cmp(d) != 0 {
		return normal, nil, nil, nil, false
	}
	radius := ratAdd(ratMul(m, circleX), b)
	if radius.Sign() <= 0 {
		return normal, nil, nil, nil, false
	}
	a = ratSub(ratMul(normal[0], circleX), d)
	y := ratMul(normal[1], radius)
	z := ratMul(normal[2], radius)
	return normal, d, a, ratAdd(ratMul(y, y), ratMul(z, z)), true
}

// CapPlaneTouchesRootCircle reports whether the cap plane through corners
// meets the root circle of radius m*x+b at x = circleX at exactly one point,
// with the circle on the same side of the plane as the source band. With
// phi = a + y*cos(theta) + z*sin(theta) on the circle, a^2 = y^2+z^2 and
// a != 0 make phi range over [a-|a|, a+|a|], so the circle lies in the
// closed half-space of sign(a) and touches the plane once. The preceding
// cap map source must have phi of one strict sign s at all four corners,
// and sign(a) must equal s. The caller has proved source's denominator
// positive over the corners.
func CapPlaneTouchesRootCircle(capMap Map, corners [4][2]*big.Rat,
	m, b, circleX *big.Rat, source Map) bool {
	normal, d, a, yz2, ok := capPlaneAndCircle(capMap, corners, m, b, circleX)
	if !ok || a.Sign() == 0 || ratMul(a, a).Cmp(yz2) != 0 {
		return false
	}
	side := 0
	for _, corner := range corners {
		sign := ratSub(VDot(normal, source.At(corner[0], corner[1])), d).Sign()
		if sign == 0 || (side != 0 && sign != side) {
			return false
		}
		side = sign
	}
	return side == a.Sign()
}

// CapOutsideNestedCone reports whether every point of the cap plane over the
// source box satisfies x + innerSlope*r > innerApex, which is strictly outside
// the inner cone, by nesting it in the outer cone x + outerSlope*r = outerApex
// truncated at outerFar. It requires:
//
//  1. the cap plane lies in the closed outside of the outer double cone: its
//     pullback has q0 = q1 = q2 = q4 = 0, q3 >= 0 and q5 > 0;
//  2. the map denominator is positive over the box, and X.Hi is below both
//     apexes over the box;
//  3. R = (outerApex-outerFar)/outerSlope is positive and the box satisfies
//     max(Y^2) + max(Z^2) <= R^2;
//  4. G(0) = outerApex-innerApex > 0 and
//     G(R) = outerFar-innerApex+innerSlope*R > 0.
//
// G(r) is affine, so 4 makes it positive on [0, R], where 3 keeps r.
func CapOutsideNestedCone(capMap Map, box [4]*big.Rat,
	outerApex, outerSlope, outerFar, innerApex, innerSlope *big.Rat) bool {
	if outerApex == nil || outerSlope == nil || outerFar == nil || innerApex == nil || innerSlope == nil {
		return false
	}
	q := ConePullback(capMap, outerApex, outerSlope)
	if q[0].Sign() != 0 || q[1].Sign() != 0 || q[2].Sign() != 0 || q[4].Sign() != 0 ||
		q[3].Sign() < 0 || q[5].Sign() <= 0 {
		return false
	}
	if PositiveDenominator(capMap, box) != nil {
		return false
	}
	corners := BoxCorners(box)
	xHi := ScalarRange(capMap, RatVec{ratOne(), ratZero(), ratZero()}, corners).Hi
	if xHi.Cmp(innerApex) >= 0 || xHi.Cmp(outerApex) >= 0 {
		return false
	}
	if outerSlope.Sign() <= 0 {
		return false
	}
	reach := ratDiv(ratSub(outerApex, outerFar), outerSlope)
	if reach.Sign() <= 0 {
		return false
	}
	yRange := ScalarRange(capMap, RatVec{ratZero(), ratOne(), ratZero()}, corners)
	zRange := ScalarRange(capMap, RatVec{ratZero(), ratZero(), ratOne()}, corners)
	if ratAdd(IntervalSquareMax(yRange), IntervalSquareMax(zRange)).Cmp(ratMul(reach, reach)) > 0 {
		return false
	}
	if ratSub(outerApex, innerApex).Sign() <= 0 {
		return false
	}
	atReach := ratAdd(ratSub(outerFar, innerApex), ratMul(innerSlope, reach))
	return atReach.Sign() > 0
}

// These cap-X coefficients are negative on the admitted first spline span.
// Thus endpoint evaluations enclose X for a rational crossing interval.
func CapXDecreases(m Map, span freeform.BezierSpan) bool {
	g, h := CapBernsteinRows(m, span)
	numerator := polynomial.BernsteinSub(polynomial.BernsteinMul(polynomial.BernsteinDeriv(h[0]), g),
		polynomial.BernsteinMul(h[0], polynomial.BernsteinDeriv(g)))

	for _, coefficient := range numerator {
		if coefficient.Sign() >= 0 {
			return false
		}
	}
	return true
}

// BezierPoint evaluates span at s exactly by de Casteljau.
func BezierPoint(span freeform.BezierSpan, s *big.Rat) [2]*big.Rat {
	var u, v [4]*big.Rat
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	for width := len(span); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			u[i] = ratAdd(ratMul(ratSub(ratOne(), s), u[i]), ratMul(s, u[i+1]))
			v[i] = ratAdd(ratMul(ratSub(ratOne(), s), v[i]), ratMul(s, v[i+1]))
		}
	}
	return [2]*big.Rat{u[0], v[0]}
}

// CapXRange returns the X interval of cap map m over the crossing interval.
func CapXRange(m Map, span freeform.BezierSpan,
	crossing RootCrossing) Interval {
	left, right := BezierPoint(span, crossing.Lo), BezierPoint(span, crossing.Hi)
	return Interval{
		Lo: m.At(right[0], right[1])[0],
		Hi: m.At(left[0], left[1])[0],
	}
}

// BandXRange returns the X interval at band time t between two cap maps over
// the crossing interval.
func BandXRange(first, second Map, span freeform.BezierSpan,
	root RootCrossing, t *big.Rat) Interval {
	a, z := CapXRange(first, span, root), CapXRange(second, span, root)
	oneMinus := ratSub(ratOne(), t)
	return Interval{
		Lo: ratAdd(ratMul(oneMinus, a.Lo), ratMul(t, z.Lo)),
		Hi: ratAdd(ratMul(oneMinus, a.Hi), ratMul(t, z.Hi)),
	}
}
