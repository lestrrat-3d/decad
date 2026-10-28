package proof

import (
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

// powerOfTwoExp reports k for w = 2^k and false for any other positive w.
func powerOfTwoExp(w *big.Int) (int, bool) {
	if w.Sign() <= 0 {
		return 0, false
	}
	k := w.TrailingZeroBits()
	return int(k), w.BitLen() == int(k)+1
}

// XdotDyadic is a·b as a Dyadic: XdotNum over the positive denominator
// a.W·b.W, which is a power of two for every point this package lifts from
// floats and every cross product of two such points. ok is false for any other
// denominator; the caller then takes XdotRat. The value is the rational
// XdotRat returns, bit for bit, once converted (Dyadic.Rat is lowest terms).
func XdotDyadic(a, b Xpt) (Dyadic, bool) {
	ka, okA := powerOfTwoExp(a.W)
	kb, okB := powerOfTwoExp(b.W)
	if !okA || !okB {
		return Dyadic{}, false
	}
	return fromBig(XdotNum(a, b), -(ka + kb)), true
}

// XhpOfDyadics is XhpOf over three exact Dyadic coordinates: one shared
// power-of-two denominator from the lowest binary exponent, numerators shifted
// up to it. For three lifted floats it is XhpOf.
func XhpOfDyadics(dx, dy, dz Dyadic) Xhp {
	base := 0
	lower := func(d Dyadic) {
		if !d.IsZero() && d.Exp() < base {
			base = d.Exp()
		}
	}
	lower(dx)
	lower(dy)
	lower(dz)
	// Bound all four integers' capacity to the largest shifted coordinate or
	// denominator. Small points then keep the same one allocation with less
	// unused word storage; large integers retain the existing three-word margin
	// before big.Int grows independently.
	maxBits := -base + 1
	for _, d := range [3]Dyadic{dx, dy, dz} {
		if !d.IsZero() {
			maxBits = max(maxBits, d.bitLen()+d.Exp()-base)
		}
	}
	wordsPerInt := min(3, (maxBits+bits.UintSize-1)/bits.UintSize)
	fill := func(coordinates *[4]big.Int, words []big.Word) Xhp {
		out := Xhp{X: &coordinates[0], Y: &coordinates[1], Z: &coordinates[2], W: &coordinates[3]}
		coord := func(index int, dst *big.Int, d Dyadic) {
			if d.IsZero() {
				return
			}
			start := index * wordsPerInt
			dst.SetBits(words[start : start : start+wordsPerInt])
			d.MantInto(dst)
			shiftBigIntWithinCapacity(dst, uint(d.Exp()-base))
		}
		coord(0, out.X, dx)
		coord(1, out.Y, dy)
		coord(2, out.Z, dz)
		start := 3 * wordsPerInt
		out.W.SetBits(words[start : start : start+wordsPerInt])
		setShiftedOne(out.W, uint(-base))
		return out
	}
	switch wordsPerInt {
	case 1:
		storage := new(struct {
			coordinates [4]big.Int
			words       [4]big.Word
		})
		return fill(&storage.coordinates, storage.words[:])
	case 2:
		storage := new(struct {
			coordinates [4]big.Int
			words       [8]big.Word
		})
		return fill(&storage.coordinates, storage.words[:])
	default:
		storage := new(struct {
			coordinates [4]big.Int
			words       [12]big.Word
		})
		return fill(&storage.coordinates, storage.words[:])
	}
}

// shiftBigIntWithinCapacity left-shifts z in its existing word storage. The
// caller sized that storage for the shifted value. A larger value falls back
// to big.Int.Lsh, which allocates independent storage as usual.
func shiftBigIntWithinCapacity(z *big.Int, shift uint) {
	if shift == 0 || z.Sign() == 0 {
		return
	}
	words := z.Bits()
	oldLength := len(words)
	wordShift, bitShift := int(shift/bits.UintSize), shift%bits.UintSize
	length := len(words) + wordShift
	var carry big.Word
	if bitShift != 0 {
		carry = words[len(words)-1] >> (bits.UintSize - bitShift)
		if carry != 0 {
			length++
		}
	}
	if length > cap(words) {
		z.Lsh(z, shift)
		return
	}
	negative := z.Sign() < 0
	words = words[:length]
	for i := oldLength - 1; i >= 0; i-- {
		word := words[i]
		at := i + wordShift
		words[at] = word << bitShift
		if bitShift != 0 && i > 0 {
			words[at] |= words[i-1] >> (bits.UintSize - bitShift)
		}
	}
	if bitShift != 0 && carry != 0 {
		words[oldLength+wordShift] = carry
	}
	clear(words[:wordShift])
	z.SetBits(words)
	if negative {
		z.Neg(z)
	}
}

// setShiftedOne writes 2^shift into z's reserved words when it fits there.
func setShiftedOne(z *big.Int, shift uint) {
	words := z.Bits()
	wordShift, bitShift := int(shift/bits.UintSize), shift%bits.UintSize
	if wordShift >= cap(words) {
		z.Lsh(big.NewInt(1), shift)
		return
	}
	words = words[:wordShift+1]
	clear(words)
	words[wordShift] = big.Word(1) << bitShift
	z.SetBits(words)
}

// XptDenotes reports whether the exact point p is exactly the float vertex v,
// coordinate by coordinate, by cross-multiplied integer comparison: p.X·2^k ==
// v.X·p.W with v lifted over 2^k. It decides what CoordDistance(p, v).Sign()
// == 0 decides, without forming any rational.
func XptDenotes(p Xpt, v r3.Vec) bool {
	h := XhpOf(v)
	var lhs, rhs big.Int
	for _, pair := range [3][2]*big.Int{{p.X, h.X}, {p.Y, h.Y}, {p.Z, h.Z}} {
		lhs.Mul(pair[0], h.W)
		rhs.Mul(pair[1], p.W)
		if lhs.Cmp(&rhs) != 0 {
			return false
		}
	}
	return true
}

// ExactAffineImage is the exact image of p under the linear part (ex, ey, ez
// are the basis columns) followed by translation t, every product and sum a
// Dyadic: the same point meshbool.ExactRigidImage builds through big.Rat, in
// the same homogeneous spelling (XhpOfDyadics then XhpStripTwosOwned).
func ExactAffineImage(ex, ey, ez, t, p r3.Vec) Xpt {
	px, py, pz := MustDyOf(p.X), MustDyOf(p.Y), MustDyOf(p.Z)
	coord := func(cx, cy, cz, tc float64) Dyadic {
		s := DyMul(MustDyOf(cx), px)
		s = DyAdd(s, DyMul(MustDyOf(cy), py))
		s = DyAdd(s, DyMul(MustDyOf(cz), pz))
		return DyAdd(s, MustDyOf(tc))
	}
	return Xpt(XhpStripTwosOwned(XhpOfDyadics(
		coord(ex.X, ey.X, ez.X, t.X),
		coord(ex.Y, ey.Y, ez.Y, t.Y),
		coord(ex.Z, ey.Z, ez.Z, t.Z))))
}
