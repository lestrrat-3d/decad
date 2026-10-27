package proof

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// OrientSignExpansion computes an exact determinant sign when the coordinate
// differences are exactly representable and zero or in [2^-200, 2^200]. Other
// inputs return uncertain and must use integer arithmetic.
//
// Each admitted difference lies on the 2^-252 grid. Products of three lie on
// the 2^-756 grid, so every nonzero product remainder and expansion component
// is normal. Six triple products and their sums stay below 2^603. FMA therefore
// recovers exact product remainders without overflow or underflow. Each triple
// product is the exact sum of four components, so the determinant is the exact
// sum of 24 floats. orientSignSum2 decides its sign from a compensated sum when
// that sum clears its proven error bound. Otherwise TwoSum accumulates the 24
// components as an exact expansion whose highest component gives the sign,
// including exact zero. See https://www.cs.cmu.edu/~quake/robust.html.
func OrientSignExpansion(a, b, c, d r3.Vec) (int, bool) {
	ba, ca, da := b.Sub(a), c.Sub(a), d.Sub(a)
	if !exactDifference(b.X, a.X, ba.X) || !exactDifference(b.Y, a.Y, ba.Y) ||
		!exactDifference(b.Z, a.Z, ba.Z) || !exactDifference(c.X, a.X, ca.X) ||
		!exactDifference(c.Y, a.Y, ca.Y) || !exactDifference(c.Z, a.Z, ca.Z) ||
		!exactDifference(d.X, a.X, da.X) || !exactDifference(d.Y, a.Y, da.Y) ||
		!exactDifference(d.Z, a.Z, da.Z) {
		return 0, false
	}
	if s, ok := orientSignSum2(ba, ca, da); ok {
		return s, true
	}
	return orientSignGrow(ba, ca, da), true
}

// exactDifference reports whether diff, the rounded p - q, is zero or in
// [2^-200, 2^200] and exact. TwoDiff reconstructs the subtraction's error; a
// nonzero tail requires the general integer path, rather than a rounded
// difference.
func exactDifference(p, q, diff float64) bool {
	if !(diff == 0 || diff >= 0x1p-200 && diff <= 0x1p200 || diff <= -0x1p-200 && diff >= -0x1p200) {
		return false
	}
	virtualB := p - diff
	virtualA := diff + virtualB
	return (p-virtualA)+(virtualB-q) == 0
}

// orientSignSum2 is the sign of det[ba, ca, da] when Sum2 decides it, for
// differences OrientSignExpansion has admitted. ok is false when the
// compensated sum lies within its error bound; the caller then builds the
// exact expansion.
//
// Sum2 (Ogita, Rump and Oishi, "Accurate sum and dot product", SIAM J. Sci.
// Comput. 26(6), 2005, Algorithm 4.4) runs TwoSum along the n = 24 exact
// components p_i: pi carries the rounded running sum, sigma the float sum of
// the TwoSum errors, and res = fl(pi + sigma). Their Proposition 4.5 bounds
// |res - s| <= eps|s| + g*S for s the exact sum, S = sum |p_i|, eps = 2^-53
// and g = gamma(23)^2, where gamma(k) = k*eps/(1-k*eps); it holds with
// underflow too. If |res| > g*S, res and s are nonzero with one sign: s = 0
// gives |res| <= g*S, and opposite signs give |res| + |s| = |res - s|, so
// |res| <= g*S - (1-eps)|s| <= g*S.
//
// mag is the float sum of the six |hi|. Each triple product's four components
// sum to at most (1+3eps)|hi| in magnitude: |lo| <= eps|hi|, |xyTail| <=
// eps|xy|, |tailHi| <= eps(1+eps)^2|hi| and |tailLo| <= eps|tailHi|. A float
// sum of six nonnegative values is at least (1-gamma(5)) times their exact
// sum, so S <= (1+3eps)/(1-gamma(5))*mag. g*(1+3eps)/(1-gamma(5)) is below
// 6.6e-30, and 2^-96 exceeds 1.26e-29, so bound = 2^-96*mag >= g*S. Scaling
// by 2^-96 is exact: a nonzero mag is at least 2^-600, and the result stays
// normal. Each product that feeds an addition is wrapped in float64(), so no
// compiler fuses it into that addition and changes TwoSum's operands. The
// steps are written out in registers rather than looped over an array, whose
// every access the race detector instruments.
func orientSignSum2(ba, ca, da r3.Vec) (int, bool) {
	var pi, sigma float64
	h0, l0, th0, tl0 := tripleComponents(ba.X, ca.Y, da.Z)
	pi, sigma = sum2Step(pi, sigma, h0)
	pi, sigma = sum2Step(pi, sigma, l0)
	pi, sigma = sum2Step(pi, sigma, th0)
	pi, sigma = sum2Step(pi, sigma, tl0)
	h1, l1, th1, tl1 := tripleComponents(ba.Y, ca.Z, da.X)
	pi, sigma = sum2Step(pi, sigma, h1)
	pi, sigma = sum2Step(pi, sigma, l1)
	pi, sigma = sum2Step(pi, sigma, th1)
	pi, sigma = sum2Step(pi, sigma, tl1)
	h2, l2, th2, tl2 := tripleComponents(ba.Z, ca.X, da.Y)
	pi, sigma = sum2Step(pi, sigma, h2)
	pi, sigma = sum2Step(pi, sigma, l2)
	pi, sigma = sum2Step(pi, sigma, th2)
	pi, sigma = sum2Step(pi, sigma, tl2)
	h3, l3, th3, tl3 := tripleComponents(-ba.X, ca.Z, da.Y)
	pi, sigma = sum2Step(pi, sigma, h3)
	pi, sigma = sum2Step(pi, sigma, l3)
	pi, sigma = sum2Step(pi, sigma, th3)
	pi, sigma = sum2Step(pi, sigma, tl3)
	h4, l4, th4, tl4 := tripleComponents(-ba.Y, ca.X, da.Z)
	pi, sigma = sum2Step(pi, sigma, h4)
	pi, sigma = sum2Step(pi, sigma, l4)
	pi, sigma = sum2Step(pi, sigma, th4)
	pi, sigma = sum2Step(pi, sigma, tl4)
	h5, l5, th5, tl5 := tripleComponents(-ba.Z, ca.Y, da.X)
	pi, sigma = sum2Step(pi, sigma, h5)
	pi, sigma = sum2Step(pi, sigma, l5)
	pi, sigma = sum2Step(pi, sigma, th5)
	pi, sigma = sum2Step(pi, sigma, tl5)
	res := pi + sigma
	mag := Abs(h0) + Abs(h1) + Abs(h2) + Abs(h3) + Abs(h4) + Abs(h5)
	bound := 0x1p-96 * mag
	if res > bound {
		return 1, true
	}
	if res < -bound {
		return -1, true
	}
	return 0, false
}

// tripleComponents returns four floats whose exact sum is x*y*z: hi is the
// rounded product and lo, tailHi, tailLo the FMA remainders.
func tripleComponents(x, y, z float64) (float64, float64, float64, float64) {
	xy := float64(x * y)
	xyTail := math.FMA(x, y, -xy)
	hi := float64(xy * z)
	tailHi := float64(xyTail * z)
	return hi, math.FMA(xy, z, -hi), tailHi, math.FMA(xyTail, z, -tailHi)
}

// sum2Step is one TwoSum step of Sum2: the new running sum, and sigma plus
// the exact error of that sum.
func sum2Step(pi, sigma, p float64) (float64, float64) {
	sum := pi + p
	virtual := sum - pi
	low := (pi - (sum - virtual)) + (p - virtual)
	return sum, sigma + low
}

// orientSignGrow is the exact sign of det[ba, ca, da] for admitted
// differences, from the full zero-eliminating expansion of its 24 components.
func orientSignGrow(ba, ca, da r3.Vec) int {
	var sum orientationExpansion
	for _, term := range [...][3]float64{
		{ba.X, ca.Y, da.Z}, {ba.Y, ca.Z, da.X}, {ba.Z, ca.X, da.Y},
		{-ba.X, ca.Z, da.Y}, {-ba.Y, ca.X, da.Z}, {-ba.Z, ca.Y, da.X},
	} {
		xy := float64(term[0] * term[1])
		xyTail := math.FMA(term[0], term[1], -xy)
		hi := float64(xy * term[2])
		lo := math.FMA(xy, term[2], -hi)
		if xyTail != 0 {
			tailHi := float64(xyTail * term[2])
			tailLo := math.FMA(xyTail, term[2], -tailHi)
			sum.add(tailLo)
			sum.add(tailHi)
		}
		sum.add(lo)
		sum.add(hi)
	}
	if sum.n == 0 {
		return 0
	}
	if sum.terms[sum.n-1] > 0 {
		return 1
	}
	return -1
}

// orientationExpansion is a zero-eliminating Grow-Expansion using TwoSum.
// Every triple product contributes at most four components, so six products
// need at most 24 slots. Each add writes behind the input it has already read.
type orientationExpansion struct {
	terms [24]float64
	n     int
}

func (e *orientationExpansion) add(q float64) {
	if q == 0 {
		return
	}
	used := 0
	for _, component := range e.terms[:e.n] {
		sum := q + component
		virtual := sum - q
		low := (q - (sum - virtual)) + (component - virtual)
		if low != 0 {
			e.terms[used] = low
			used++
		}
		q = sum
	}
	if q != 0 {
		e.terms[used] = q
		used++
	}
	e.n = used
}
