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
	return xptDyadicCoordinates(p, k), true
}

func xptDyadicCoordinates(p Xpt, weightExp int) DyV3 {
	return DyV3{
		integerDyadic(p.X, -weightExp),
		integerDyadic(p.Y, -weightExp),
		integerDyadic(p.Z, -weightExp),
	}
}

// integerDyadic reads inline mantissas without allocating a temporary integer.
// A wider mantissa needs a copy because fromBig shifts and may keep its input.
func integerDyadic(n *big.Int, exp int) Dyadic {
	if n.Sign() == 0 {
		return Dyadic{}
	}
	tz := n.TrailingZeroBits()
	if n.BitLen()-int(tz) <= dyBits {
		words := n.Bits()
		return fromMag(absShiftedLow64(words, tz), absShiftedLow64(words, tz+64), n.Sign() < 0, exp+int(tz))
	}
	return fromBig(new(big.Int).Set(n), exp)
}

// XtriangleNormal returns (b-a) cross (c-a), with the same four integers as
// Xcross(Xsub(b, a), Xsub(c, a)). Power-of-two weights use exact dyadic
// arithmetic without constructing the two homogeneous edge vectors.
// All point coordinates and weights must be non-nil. The inputs are read-only.
func XtriangleNormal(a, b, c Xpt) Xpt {
	ka, oka := powerOfTwoExp(a.W)
	kb, okb := powerOfTwoExp(b.W)
	kc, okc := powerOfTwoExp(c.W)
	if !oka || !okb || !okc {
		return Xcross(Xsub(b, a), Xsub(c, a))
	}
	// Stripping common twos gives a positive power-of-two weight exactly one
	// spelling, so reconstructing the dyadic result preserves all four integers.
	da := xptDyadicCoordinates(a, ka)
	db := xptDyadicCoordinates(b, kb)
	dc := xptDyadicCoordinates(c, kc)
	n := DvCross(DvSub(db, da), DvSub(dc, da))
	return Xpt(XhpStripTwosOwned(XhpOfDyadics(n[0], n[1], n[2])))
}
