package proof

import "math/big"

// XptDyV3 returns the exact coordinates of p as Dyadics. ok is false unless
// p.W is a power of two, which holds for every point lifted from floats. The
// result shares nothing with p.
func XptDyV3(p Xpt) (DyV3, bool) {
	k, ok := powerOfTwoExp(p.W)
	if !ok {
		return DyV3{}, false
	}
	// fromBig shifts its argument in place and may keep it as the mantissa.
	return DyV3{
		fromBig(new(big.Int).Set(p.X), -k),
		fromBig(new(big.Int).Set(p.Y), -k),
		fromBig(new(big.Int).Set(p.Z), -k),
	}, true
}
