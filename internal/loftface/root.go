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

// CapPlaneAvoidsRootCircle reports whether the cap plane through corners
// provably misses the root circle of radius m*x+b at x = circleX.
func CapPlaneAvoidsRootCircle(capMap Map, corners [4][2]*big.Rat,
	m, b, circleX *big.Rat) bool {
	p0 := capMap.At(corners[0][0], corners[0][1])
	p1 := capMap.At(corners[1][0], corners[1][1])
	p2 := capMap.At(corners[2][0], corners[2][1])
	normal := VCross(VSub(p1, p0), VSub(p2, p0))
	if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
		return false
	}
	d := VDot(normal, p0)
	if VDot(normal, capMap.At(corners[3][0], corners[3][1])).Cmp(d) != 0 {
		return false
	}
	radius := ratAdd(ratMul(m, circleX), b)
	if radius.Sign() <= 0 {
		return false
	}
	a := ratSub(ratMul(normal[0], circleX), d)
	y := ratMul(normal[1], radius)
	z := ratMul(normal[2], radius)
	return ratMul(a, a).Cmp(ratAdd(ratMul(y, y), ratMul(z, z))) > 0
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

// The 64 dyadic bisections give a rational enclosure narrower than 2^-64
// of one source cubic. The cap builder still has to charge its mapped width.
func IsolateRoot(coeff []*big.Rat) RootCrossing {
	return IsolateRootN(coeff, 64)
}

// IsolateRootN bisects the one sign change of a Bernstein row steps
// times and returns the dyadic bracket as a span-0 crossing.
func IsolateRootN(coeff []*big.Rat, steps int) RootCrossing {
	lo, hi := polynomial.BernsteinIsolateRoot(coeff, steps)
	return RootCrossing{Span: 0, Lo: lo, Hi: hi}
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

// A linear-fractional cap coordinate has its extrema at corners of a source
// rectangle when its denominator is positive. The restricted cubic's
// Bernstein controls enclose the true source U,V over the s interval.
func JunctionBox(first, second Map, span freeform.BezierSpan,
	sLo, sHi, tLo, tHi *big.Rat) ([3]Interval, bool) {
	var out [3]Interval
	u, v := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	u, v = polynomial.BernsteinRestrict(u, sLo, sHi), polynomial.BernsteinRestrict(v, sLo, sHi)
	uMin, uMax, vMin, vMax := u[0], u[0], v[0], v[0]
	for i := range u {
		if u[i].Cmp(uMin) < 0 {
			uMin = u[i]
		}
		if u[i].Cmp(uMax) > 0 {
			uMax = u[i]
		}
		if v[i].Cmp(vMin) < 0 {
			vMin = v[i]
		}
		if v[i].Cmp(vMax) > 0 {
			vMax = v[i]
		}
	}
	corners := [4][2]*big.Rat{{uMin, vMin}, {uMin, vMax}, {uMax, vMin}, {uMax, vMax}}
	var caps [2][3]Interval
	for i, mapped := range []Map{first, second} {
		for j, corner := range corners {
			if mapped.Denominator(corner[0], corner[1]).Sign() <= 0 {
				return out, false
			}
			point := mapped.At(corner[0], corner[1])
			for axis := range out {
				if j == 0 || point[axis].Cmp(caps[i][axis].Lo) < 0 {
					caps[i][axis].Lo = point[axis]
				}
				if j == 0 || point[axis].Cmp(caps[i][axis].Hi) > 0 {
					caps[i][axis].Hi = point[axis]
				}
			}
		}
	}
	interpolate := func(a, z, t *big.Rat) *big.Rat {
		return ratAdd(ratMul(ratSub(ratOne(), t), a), ratMul(t, z))
	}
	for axis := range out {
		lo0 := interpolate(caps[0][axis].Lo, caps[1][axis].Lo, tLo)
		lo1 := interpolate(caps[0][axis].Lo, caps[1][axis].Lo, tHi)
		hi0 := interpolate(caps[0][axis].Hi, caps[1][axis].Hi, tLo)
		hi1 := interpolate(caps[0][axis].Hi, caps[1][axis].Hi, tHi)
		if lo0.Cmp(lo1) > 0 {
			lo0 = lo1
		}
		if hi0.Cmp(hi1) < 0 {
			hi0 = hi1
		}
		out[axis] = Interval{Lo: lo0, Hi: hi0}
	}
	return out, true
}
