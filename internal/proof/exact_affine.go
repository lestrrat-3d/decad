package proof

import (
	"math/big"

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
	// The usual float-derived coordinates fit in three words each. Bound
	// every segment's capacity so growth allocates independent storage rather
	// than overwriting another coordinate. Larger integers grow as usual.
	storage := new(struct {
		coordinates [4]big.Int
		words       [12]big.Word
	})
	out := Xhp{X: &storage.coordinates[0], Y: &storage.coordinates[1], Z: &storage.coordinates[2], W: &storage.coordinates[3]}
	coord := func(dst *big.Int, words []big.Word, d Dyadic) {
		if d.IsZero() {
			return
		}
		dst.SetBits(words[:0:len(words)])
		d.MantInto(dst)
		dst.Lsh(dst, uint(d.Exp()-base))
	}
	coord(out.X, storage.words[0:3], dx)
	coord(out.Y, storage.words[3:6], dy)
	coord(out.Z, storage.words[6:9], dz)
	out.W.SetBits(storage.words[9:9:12])
	out.W.Lsh(big.NewInt(1), uint(-base))
	return out
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
