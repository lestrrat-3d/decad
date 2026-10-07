package proof

import "math/big"

// DySquaredDistance3 returns the exact squared distance between two points
// whose coordinates are finite float64 values. It returns false for a
// non-finite coordinate.
func DySquaredDistance3(a0, a1, a2, b0, b1, b2 float64) (Dyadic, bool) {
	sum := DyZero()
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, okX := DyOf(pair[0])
		y, okY := DyOf(pair[1])
		if !okX || !okY {
			return Dyadic{}, false
		}
		diff := DySubScalar(x, y)
		sum = DyAdd(sum, DyMul(diff, diff))
	}
	return sum, true
}

// RatSquaredDistance3 returns DySquaredDistance3 as a rational value. It
// returns nil for a non-finite coordinate.
func RatSquaredDistance3(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	d, ok := DySquaredDistance3(a0, a1, a2, b0, b1, b2)
	if !ok {
		return nil
	}
	return d.Rat()
}
