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
// recovers exact product remainders without overflow or underflow. TwoSum
// accumulates them as an exact expansion whose highest component gives the
// sign, including exact zero. See https://www.cs.cmu.edu/~quake/robust.html.
func OrientSignExpansion(a, b, c, d r3.Vec) (int, bool) {
	ba, ca, da := b.Sub(a), c.Sub(a), d.Sub(a)
	for _, row := range [...][3]float64{
		{b.X, a.X, ba.X}, {b.Y, a.Y, ba.Y}, {b.Z, a.Z, ba.Z},
		{c.X, a.X, ca.X}, {c.Y, a.Y, ca.Y}, {c.Z, a.Z, ca.Z},
		{d.X, a.X, da.X}, {d.Y, a.Y, da.Y}, {d.Z, a.Z, da.Z},
	} {
		x := Abs(row[2])
		if !(x == 0 || x >= 0x1p-200 && x <= 0x1p200) {
			return 0, false
		}
		// TwoDiff reconstructs the subtraction's error. Nonzero tails
		// require the general integer path, rather than a rounded difference.
		virtualB := row[0] - row[2]
		virtualA := row[2] + virtualB
		tail := (row[0] - virtualA) + (virtualB - row[1])
		if tail != 0 {
			return 0, false
		}
	}
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
		return 0, true
	}
	if sum.terms[sum.n-1] > 0 {
		return 1, true
	}
	return -1, true
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
