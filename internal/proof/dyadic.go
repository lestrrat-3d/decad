package proof

import (
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

// This file is the package's exact BINARY-SCALED arithmetic: Dyadic, the exact
// scalar every proof over held float64 coordinates is carried in, and DyV3, the
// exact vector built on it.
//
// Every float64 is exactly a Dyadic rational — a mantissa times a power of two
// (math.Frexp) — and the closure of that set under +, − and × is itself. So a
// determinant, a cross product, a dot product or any other polynomial in held
// coordinates is Dyadic too, and needs no general fraction to represent it.
//
// big.Rat represents such a value correctly but pays for generality it never
// uses: it reduces to lowest terms after EVERY operation, and that reduction is
// a Lehmer GCD over the full numerator and denominator. On a Dyadic value the
// answer is always a power of two, so the GCD computes something the exponent
// already states. Stripping the mantissa's trailing zero bits reaches the same
// reduced form with a bit count and a shift (norm).
//
// So the rule this file exists to enforce: a quantity whose whole derivation is
// +, − and × over held floats is a Dyadic, never a big.Rat. A quantity that
// genuinely LEAVES that set — a moment integral's division by (i+1)(j+1), a
// factorial denominator, any ratio of two computed values — converts at exactly
// the point it divides (rat), and every such point is the boundary between this
// file's arithmetic and math/big's.
//
// Nothing here is a tolerance, an approximation or a widened float path. A
// Dyadic holds the same number big.Rat held, bit for bit, and every comparison
// it answers is the comparison big.Rat answered.
//
// The mantissa is held INLINE, as a 127-bit magnitude in two machine words
// with the sign in the top bit of the high word, whenever it fits, and in a
// big.Int only when it does not. A held float's mantissa has at most 53 bits
// and a product of two at most 106, so lifts, products of lifts and most of
// the sums built from them run on machine words and allocate nothing. Each
// inline operation detects the one way it can fail — a carry, a product word
// or an aligning shift that leaves the 127 bits — with math/bits, and then
// redoes the operation in big.Int. The inline and big.Int paths compute the
// same exact integer; they differ only in where it is stored.
//
// The struct is four word-sized fields, which is what lets the compiler keep a
// Dyadic in registers rather than in a stack slot. That matters beyond speed:
// Go 1.26.0 to 1.26.5 merge stack slots by a liveness analysis that ends a
// local's life at the instruction computing an address into it, not at the
// copy that reads through that address (fixed in Go 1.26.6, golang/go#80127),
// and a Dyadic too large for registers turned every by-value argument and
// inlined parameter into such a slot. A wider inline mantissa would need a
// fifth field or an array and give that up. Arrays of Dyadic (DyV3) still
// live on the stack, so code over them copies an element into a named local
// before it reaches a call, or indexes by constant.

// dyBits is the inline mantissa's width in bits.
const dyBits = 127

// dySign is the sign bit in an inline mantissa's high word.
const dySign = 1 << 63

// dyBuf is caller-owned storage for a big.Int view of an inline mantissa
// (view), sized for the platform's big.Word.
type dyBuf [128 / bits.UintSize]big.Word

// Dyadic is an exact binary-scaled rational: mant × 2^exp, with mant an
// arbitrary-precision integer and exp a binary exponent.
//
// The representation is kept REDUCED — a non-zero mant is odd — so that two
// dyadics are equal exactly when their fields are (norm). Zero is the one value
// with an even mantissa, held as mant 0 at exp 0. It is also CANONICAL in
// where the mantissa lives: a mantissa of at most 127 bits is always held
// inline in lo and hi with big nil, and a wider one always in big, so the two
// never describe the same value.
//
// A big mantissa is never mutated once the Dyadic holding it is built, so two
// dyadics may share one.
//
// The zero VALUE of the struct is a valid zero, which is what lets a DyV3 be
// declared with var and filled in component by component the way its big.Rat
// predecessor could be.
type Dyadic struct {
	big *big.Int // the mantissa when it is wider than 127 bits; nil otherwise
	lo  uint64   // |mantissa|'s low word when big is nil
	hi  uint64   // the sign (dySign) and |mantissa|'s high 63 bits when big is nil
	exp int
}

// DyZero is the additive identity, and what a Dyadic's zero value denotes.
func DyZero() Dyadic { return Dyadic{} }

// sign reports the value's sign, which is its mantissa's: the scale factor
// 2^exp is positive for every exp.
//
// A non-zero inline mantissa is odd, so its low word alone tells it from zero,
// here and in IsZero.
func (d Dyadic) Sign() int {
	switch {
	case d.big != nil:
		return d.big.Sign()
	case d.hi&dySign != 0:
		return -1
	case d.lo == 0:
		return 0
	default:
		return 1
	}
}

// isZero reports whether the value is exactly zero.
func (d Dyadic) IsZero() bool { return d.big == nil && d.lo == 0 }

// neg reports whether an inline mantissa is negative.
func (d Dyadic) neg() bool { return d.hi&dySign != 0 }

// bitLen is the bit length of the mantissa's magnitude.
func (d Dyadic) bitLen() int {
	if d.big != nil {
		return d.big.BitLen()
	}
	return bitLen128(d.lo, d.hi&^dySign)
}

// bitLen128 is the bit length of the magnitude hi·2^64 + lo.
func bitLen128(lo, hi uint64) int {
	if hi != 0 {
		return 64 + bits.Len64(hi)
	}
	return bits.Len64(lo)
}

// fromMag builds the canonical Dyadic for ±(hi·2^64 + lo) × 2^exp, a
// magnitude of at most 127 bits: a zero magnitude is the zero value, and a
// non-zero one is made odd by shifting its trailing zero bits into the
// exponent.
func fromMag(lo, hi uint64, neg bool, exp int) Dyadic {
	switch {
	case lo != 0:
		if tz := bits.TrailingZeros64(lo); tz > 0 {
			lo = lo>>tz | hi<<(64-tz)
			hi >>= tz
			exp += tz
		}
	case hi != 0:
		tz := bits.TrailingZeros64(hi)
		lo, hi = hi>>tz, 0
		exp += 64 + tz
	default:
		return Dyadic{}
	}
	if neg {
		hi |= dySign
	}
	return Dyadic{lo: lo, hi: hi, exp: exp}
}

// fromBig builds the canonical Dyadic for z × 2^exp, taking ownership of z: it
// reduces z in place, then moves it inline where it fits.
func fromBig(z *big.Int, exp int) Dyadic {
	if z.Sign() == 0 {
		return Dyadic{}
	}
	if shift := z.TrailingZeroBits(); shift > 0 {
		z.Rsh(z, shift)
		exp += int(shift)
	}
	if z.BitLen() > dyBits {
		return Dyadic{big: z, exp: exp}
	}
	var lo, hi uint64
	words := z.Bits()
	if bits.UintSize == 64 {
		lo = uint64(words[0])
		if len(words) > 1 {
			hi = uint64(words[1])
		}
	} else {
		var w [4]uint64
		for i, word := range words {
			w[i] = uint64(word)
		}
		lo, hi = w[0]|w[1]<<32, w[2]|w[3]<<32
	}
	if z.Sign() < 0 {
		hi |= dySign
	}
	return Dyadic{lo: lo, hi: hi, exp: exp}
}

// view returns d's mantissa as a big.Int: a big mantissa as it stands, and an
// inline one written into t over the caller's buf. The result is read-only.
func (d *Dyadic) view(t *big.Int, buf *dyBuf) *big.Int {
	if d.big != nil {
		return d.big
	}
	lo, hi := d.lo, d.hi&^dySign
	words := buf[:]
	if bits.UintSize == 64 {
		words[0], words[1] = big.Word(lo), big.Word(hi)
	} else {
		words[0], words[1], words[2], words[3] = big.Word(lo), big.Word(lo>>32), big.Word(hi), big.Word(hi>>32)
	}
	n := len(words)
	for n > 0 && words[n-1] == 0 {
		n--
	}
	t.SetBits(words[:n])
	if d.neg() {
		t.Neg(t)
	}
	return t
}

// bigOfWords returns the integer ±words, least significant word first, in a
// new big.Int.
func bigOfWords(words []uint64, neg bool) *big.Int {
	n := len(words)
	for n > 0 && words[n-1] == 0 {
		n--
	}
	out := make([]big.Word, 0, n*64/bits.UintSize)
	if bits.UintSize == 64 {
		for _, w := range words[:n] {
			out = append(out, big.Word(w))
		}
	} else {
		for _, w := range words[:n] {
			out = append(out, big.Word(w), big.Word(w>>32))
		}
	}
	z := new(big.Int).SetBits(out)
	if neg {
		z.Neg(z)
	}
	return z
}

// DyOf lifts a float64 exactly, reporting false for a NaN or an infinity, which
// no exact proof may consume. The lift is exact by construction: math.Frexp
// splits the value into a fraction in [0.5, 1) and a binary exponent, and
// scaling that fraction by 2^53 makes it an integer without moving a bit.
func DyOf(f float64) (Dyadic, bool) {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return Dyadic{}, false
	}
	if f == 0 {
		return Dyadic{}, true
	}
	return DyOfFinite(f), true
}

// DyOfFinite lifts a finite, nonzero float. Its callers gate zero and
// non-finite inputs before reaching it; the Frexp scale and norm match DyOf,
// and a float's mantissa always fits inline, so it never allocates.
func DyOfFinite(f float64) Dyadic {
	frac, exp := math.Frexp(f)
	return dyInt64(int64(frac*(1<<53)), exp-53)
}

// dyInt64 is the canonical Dyadic for v × 2^exp.
func dyInt64(v int64, exp int) Dyadic {
	mag := uint64(v)
	if v < 0 {
		mag = -mag
	}
	return fromMag(mag, 0, v < 0, exp)
}

// MustDyOf is DyOf for a value the caller has already proven finite
// (finiteVec), which every caller of it does.
//
// A non-finite value reaching it is a missing gate in the CALLER, and it panics
// rather than answering. The alternative is worse than a crash: returning a
// zero would feed an exact, confident, wrong number into a proof that then
// publishes a bound it never established, and no test would see it. The panic
// names the caller that built the bad value instead — the same contract
// mustRatOf held, and the one ~/.claude/docs/go.md's nil-argument rule states
// for a broken caller claim.
func MustDyOf(f float64) Dyadic {
	d, ok := DyOf(f)
	if !ok {
		panic("decad: exact Dyadic lift requires a finite float")
	}
	return d
}

// DyAdd returns a + b exactly.
func DyAdd(a, b Dyadic) Dyadic { return dyAddSigned(a, b, false) }

// DySubScalar returns a − b exactly.
func DySubScalar(a, b Dyadic) Dyadic { return dyAddSigned(a, b, true) }

// dyAddSigned returns a + b, or a − b when negB is set.
func dyAddSigned(a, b Dyadic, negB bool) Dyadic {
	switch {
	case b.IsZero():
		return a
	case a.IsZero():
		if negB {
			return DyNeg(b)
		}
		return b
	case a.big == nil && b.big == nil:
		if out, ok := addInline(a, b, b.neg() != negB); ok {
			return out
		}
	}
	return addBig(a, b, negB)
}

// shl128 returns (hi·2^64 + lo) << s, reporting false when a set bit would
// pass bit 126. n is the magnitude's own bit length.
func shl128(lo, hi uint64, s, n int) (uint64, uint64, bool) {
	switch {
	case s == 0:
		return lo, hi, true
	case n+s > dyBits:
		return 0, 0, false
	case s >= 64:
		return 0, lo << (s - 64), true
	default:
		return lo << s, hi<<s | lo>>(64-s), true
	}
}

// addInline returns a + b' for b' the magnitude of b with sign bNeg, over two
// non-zero inline mantissas. The operand with the larger exponent is shifted
// down to the smaller one, and false reports that the shift left the 127
// bits. A sum that carries past them builds its big.Int from the words in
// hand.
func addInline(a, b Dyadic, bNeg bool) (Dyadic, bool) {
	alo, ahi := a.lo, a.hi&^dySign
	blo, bhi := b.lo, b.hi&^dySign
	aNeg, exp := a.neg(), a.exp
	var ok bool
	switch {
	case a.exp > b.exp:
		if alo, ahi, ok = shl128(alo, ahi, a.exp-b.exp, bitLen128(alo, ahi)); !ok {
			return Dyadic{}, false
		}
		exp = b.exp
	case b.exp > a.exp:
		if blo, bhi, ok = shl128(blo, bhi, b.exp-a.exp, bitLen128(blo, bhi)); !ok {
			return Dyadic{}, false
		}
	}
	if aNeg == bNeg {
		lo, carry := bits.Add64(alo, blo, 0)
		hi, _ := bits.Add64(ahi, bhi, carry)
		// Two magnitudes below 2^127 sum below 2^128, so the only overflow is
		// into bit 127, the sign bit's position.
		if hi&dySign != 0 {
			return fromBig(bigOfWords([]uint64{lo, hi}, aNeg), exp), true
		}
		return fromMag(lo, hi, aNeg, exp), true
	}
	switch {
	case ahi > bhi || ahi == bhi && alo > blo:
		lo, borrow := bits.Sub64(alo, blo, 0)
		hi, _ := bits.Sub64(ahi, bhi, borrow)
		return fromMag(lo, hi, aNeg, exp), true
	case ahi == bhi && alo == blo:
		return Dyadic{}, true
	default:
		lo, borrow := bits.Sub64(blo, alo, 0)
		hi, _ := bits.Sub64(bhi, ahi, borrow)
		return fromMag(lo, hi, bNeg, exp), true
	}
}

// addBig is dyAddSigned over big.Int, for an operand already held there or an
// inline sum whose aligning shift left the 127 bits.
func addBig(a, b Dyadic, negB bool) Dyadic {
	var ta, tb big.Int
	var ba, bb dyBuf
	am, bm := a.view(&ta, &ba), b.view(&tb, &bb)
	var out *big.Int
	exp := a.exp
	switch {
	case a.exp == b.exp:
		out = new(big.Int).Set(am)
	case a.exp > b.exp:
		out = new(big.Int).Lsh(am, uint(a.exp-b.exp))
		exp = b.exp
	default:
		out = new(big.Int).Set(am)
		bm = new(big.Int).Lsh(bm, uint(b.exp-a.exp))
	}
	if negB {
		out.Sub(out, bm)
	} else {
		out.Add(out, bm)
	}
	return fromBig(out, exp)
}

// DyMul returns a × b exactly. Exponents add, so no alignment is needed and the
// product of two reduced mantissas is already reduced.
func DyMul(a, b Dyadic) Dyadic {
	if a.IsZero() || b.IsZero() {
		return Dyadic{}
	}
	if a.big == nil && b.big == nil {
		// Two inline mantissas have a product of at most 254 bits, computed in
		// full and held inline when it fits in 127.
		alo, ahi := a.lo, a.hi&^dySign
		blo, bhi := b.lo, b.hi&^dySign
		neg := a.neg() != b.neg()
		h0, p0 := bits.Mul64(alo, blo)
		if ahi == 0 && bhi == 0 {
			if h0&dySign == 0 {
				return fromMulWords(p0, h0, neg, a.exp+b.exp)
			}
			return Dyadic{big: bigOfWords([]uint64{p0, h0}, neg), exp: a.exp + b.exp}
		}
		h1, l1 := bits.Mul64(alo, bhi)
		h2, l2 := bits.Mul64(ahi, blo)
		h3, l3 := bits.Mul64(ahi, bhi)
		p1, c := bits.Add64(h0, l1, 0)
		p2, c2 := bits.Add64(h1, l3, c)
		p3 := h3 + c2
		p1, c = bits.Add64(p1, l2, 0)
		p2, c2 = bits.Add64(p2, h2, c)
		p3 += c2
		if p2 == 0 && p3 == 0 && p1&dySign == 0 {
			return fromMulWords(p0, p1, neg, a.exp+b.exp)
		}
		return Dyadic{big: bigOfWords([]uint64{p0, p1, p2, p3}, neg), exp: a.exp + b.exp}
	}
	var ta, tb big.Int
	var ba, bb dyBuf
	return fromBig(new(big.Int).Mul(a.view(&ta, &ba), b.view(&tb, &bb)), a.exp+b.exp)
}

// fromMulWords is the Dyadic for an inline product: odd, since both factors
// are, and so already reduced.
func fromMulWords(lo, hi uint64, neg bool, exp int) Dyadic {
	if neg {
		hi |= dySign
	}
	return Dyadic{lo: lo, hi: hi, exp: exp}
}

// DyCmp compares a against b, returning -1, 0 or +1 the way big.Rat.Cmp does.
//
// Two values of opposite signs compare by sign, and two of one sign whose
// magnitudes' leading bits sit at different binary positions compare by that
// position: |m|·2^e lies in [2^(n+e−1), 2^(n+e)) for an n-bit mantissa m. Only
// values whose leading bits coincide pay the aligning shift, and that shift
// never widens the shifted mantissa past the other's bit length, so two
// inline mantissas compare inline.
func DyCmp(a, b Dyadic) int {
	sign := a.Sign()
	if other := b.Sign(); sign != other {
		if sign > other {
			return 1
		}
		return -1
	}
	if sign == 0 {
		return 0
	}
	if a.big == nil && b.big == nil {
		alo, ahi := a.lo, a.hi&^dySign
		blo, bhi := b.lo, b.hi&^dySign
		if a.exp != b.exp {
			aLen, bLen := bitLen128(alo, ahi), bitLen128(blo, bhi)
			if top, other := aLen+a.exp, bLen+b.exp; top != other {
				if top > other {
					return sign
				}
				return -sign
			}
			if a.exp > b.exp {
				alo, ahi, _ = shl128(alo, ahi, a.exp-b.exp, aLen)
			} else {
				blo, bhi, _ = shl128(blo, bhi, b.exp-a.exp, bLen)
			}
		}
		switch {
		case ahi > bhi || ahi == bhi && alo > blo:
			return sign
		case ahi == bhi && alo == blo:
			return 0
		default:
			return -sign
		}
	}
	aLen, bLen := a.bitLen(), b.bitLen()
	if top, other := aLen+a.exp, bLen+b.exp; top != other {
		if top > other {
			return sign
		}
		return -sign
	}
	var ta, tb big.Int
	var ba, bb dyBuf
	am, bm := a.view(&ta, &ba), b.view(&tb, &bb)
	switch {
	case a.exp == b.exp:
		return am.Cmp(bm)
	case a.exp > b.exp:
		return new(big.Int).Lsh(am, uint(a.exp-b.exp)).Cmp(bm)
	default:
		return am.Cmp(new(big.Int).Lsh(bm, uint(b.exp-a.exp)))
	}
}

// DyAbs returns |d|.
func DyAbs(d Dyadic) Dyadic {
	if d.big == nil {
		d.hi &^= dySign
		return d
	}
	if d.big.Sign() > 0 {
		return d
	}
	return Dyadic{big: new(big.Int).Abs(d.big), exp: d.exp}
}

// DyNeg returns −d.
func DyNeg(d Dyadic) Dyadic {
	if d.big == nil {
		if d.lo != 0 {
			d.hi ^= dySign
		}
		return d
	}
	return Dyadic{big: new(big.Int).Neg(d.big), exp: d.exp}
}

// rat converts to big.Rat, for the callers whose arithmetic genuinely leaves
// the Dyadic set — a moment integral dividing by (i+1)(j+1), a factorial
// denominator. It is the ONE boundary between this file and math/big's general
// fractions, and it is exact: a non-negative exponent scales an integer, and a
// negative one puts an odd mantissa over a power of two, which is already in
// lowest terms. Because it is, the denominator is written through Rat.Denom's
// documented reference into the receiver rather than handed to SetFrac, whose
// GCD would only rediscover that it is 1; after SetInt the denominator is an
// initialised 1, so Denom returns the receiver's own.
func (d Dyadic) Rat() *big.Rat {
	if d.IsZero() {
		return new(big.Rat)
	}
	var t big.Int
	var buf dyBuf
	r := new(big.Rat).SetInt(d.view(&t, &buf))
	if d.exp >= 0 {
		r.Num().Lsh(r.Num(), uint(d.exp))
	} else {
		r.Denom().Lsh(r.Denom(), uint(-d.exp))
	}
	return r
}

// DyOfRat lifts a big.Rat this package knows to be Dyadic — one whose
// denominator is a power of two. It reports false for any other fraction rather
// than rounding one, since a rounded value would be a proof about a number the
// caller never held.
func DyOfRat(r *big.Rat) (Dyadic, bool) {
	if r == nil {
		return Dyadic{}, false
	}
	den := r.Denom()
	shift := den.TrailingZeroBits()
	if den.BitLen() != int(shift)+1 {
		return Dyadic{}, false
	}
	return fromBig(new(big.Int).Set(r.Num()), -int(shift)), true
}

// float64 returns the value as a float64 plus whether that conversion was
// exact, matching big.Rat.Float64's own contract so a caller rounding outward
// can tell whether it must.
//
// An inline mantissa whose magnitude lies in [2^-1022, 2^1024) converts
// without math/big. Its top 64 bits, with a sticky bit standing for every
// lower one, round to nearest-even exactly where the whole mantissa would —
// rounding to 53 bits drops eleven bits below the sticky one — and scaling by
// a power of two is then exact, except that a value rounding up to 2^1024
// becomes the infinity big.Float also rounds it to. The conversion is exact
// exactly when the odd mantissa has at most 53 bits. A smaller magnitude,
// whose subnormal result would make the scaling round a second time, and a
// larger one go through big.Float.
func (d Dyadic) Float64() (float64, bool) {
	if d.IsZero() {
		return 0, true
	}
	n := d.bitLen()
	if d.big == nil && n-1+d.exp >= -1022 && n+d.exp <= 1024 {
		top, shift := d.lo, 0
		if hi := d.hi &^ dySign; hi != 0 {
			shift = n - 64
			sticky := d.lo<<(64-shift) != 0
			top = hi<<(64-shift) | d.lo>>shift
			if sticky {
				top |= 1
			}
		}
		f := math.Ldexp(float64(top), shift+d.exp)
		if d.neg() {
			f = -f
		}
		return f, n <= 53
	}
	// The precision must hold the WHOLE mantissa: a big.Float that rounded in
	// SetInt would report its own last conversion as exact and hide the bit it
	// already dropped, which is precisely the claim a directed rounding must
	// not be given.
	var t big.Int
	var buf dyBuf
	prec := max(uint(n), 53)
	f := new(big.Float).SetPrec(prec).SetInt(d.view(&t, &buf))
	f.SetMantExp(f, d.exp)
	out, acc := f.Float64()
	return out, acc == big.Exact
}

// DyV3 is a vector of the payload's own floats taken EXACTLY — the only
// arithmetic allowed to prove a degeneracy. It is Dyadic's vector, component
// for component, and it replaced a [3]*big.Rat whose every operation paid a
// Lehmer GCD to rediscover an exponent this representation states.
type DyV3 [3]Dyadic

// DyVec lifts a held vector exactly. Its caller has already proven the vector
// finite (finiteVec), which is what makes the per-component lift total.
func DyVec(v r3.Vec) DyV3 {
	return DyV3{MustDyOf(v.X), MustDyOf(v.Y), MustDyOf(v.Z)}
}

// DvSub returns a − b componentwise.
func DvSub(a, b DyV3) DyV3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return DyV3{DySubScalar(a0, b0), DySubScalar(a1, b1), DySubScalar(a2, b2)}
}

// DvAdd returns a + b componentwise.
func DvAdd(a, b DyV3) DyV3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return DyV3{DyAdd(a0, b0), DyAdd(a1, b1), DyAdd(a2, b2)}
}

// DvScale multiplies every vector component by an exact scalar.
func DvScale(v DyV3, s Dyadic) DyV3 {
	return DyV3{DyMul(v[0], s), DyMul(v[1], s), DyMul(v[2], s)}
}

// DvTransform applies a held rigid transform in exact dyadic arithmetic.
// The caller has already established that the transform's components are finite.
func DvTransform(t r3.Transform, p DyV3) DyV3 {
	b := t.Basis()
	return DvAdd(DyVec(t.Translation()), DvAdd(DvScale(DyVec(b.EX), p[0]),
		DvAdd(DvScale(DyVec(b.EY), p[1]), DvScale(DyVec(b.EZ), p[2]))))
}

// DvCross returns a × b exactly.
func DvCross(a, b DyV3) DyV3 {
	return DyV3{
		DySubScalar(DyMul(a[1], b[2]), DyMul(a[2], b[1])),
		DySubScalar(DyMul(a[2], b[0]), DyMul(a[0], b[2])),
		DySubScalar(DyMul(a[0], b[1]), DyMul(a[1], b[0])),
	}
}

// DvDot returns a · b exactly, summed in component order from zero.
func DvDot(a, b DyV3) Dyadic {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	out := DyAdd(DyZero(), DyMul(a0, b0))
	out = DyAdd(out, DyMul(a1, b1))
	return DyAdd(out, DyMul(a2, b2))
}

// DvIsZero reports whether every component is exactly zero.
func DvIsZero(a DyV3) bool {
	return a[0].IsZero() && a[1].IsZero() && a[2].IsZero()
}

// DvEqual compares exact vector components.
func DvEqual(a, b DyV3) bool {
	for axis := range 3 {
		if DyCmp(a[axis], b[axis]) != 0 {
			return false
		}
	}
	return true
}

// DySqrtSeed is ratSqrtSeed over a Dyadic: a float64 near sqrt(d), used only to
// START the directed walks below, never to decide them. It carries the same
// even-exponent trick its rational twin does — a Dyadic already holds its
// binary exponent, so the split the rational version had to compute is a field
// read here.
func DySqrtSeed(d Dyadic) float64 {
	// The mantissa is normalised into [0.5, 1) first and its own exponent
	// folded into the total, so a value near either end of the float64 range
	// roots from a fraction rather than from an integer the conversion would
	// saturate.
	var t big.Int
	var buf dyBuf
	mant := new(big.Float).SetPrec(64)
	exp := d.exp + new(big.Float).SetPrec(64).SetInt(d.view(&t, &buf)).MantExp(mant)
	if exp%2 != 0 {
		// Halving an odd exponent is not an integer, so shift one power of two
		// into the mantissa, which still roots cleanly.
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// DySquareAtMost reports whether f² <= d, decided exactly.
func DySquareAtMost(f float64, d Dyadic) bool {
	square, ok := DyOf(f)
	if !ok {
		return false
	}
	return DyCmp(DyMul(square, square), d) <= 0
}

// DySquareEquals reports whether f² == d, decided exactly.
func DySquareEquals(f float64, d Dyadic) bool {
	square, ok := DyOf(f)
	if !ok {
		return false
	}
	return DyCmp(DyMul(square, square), d) == 0
}

// DySqrtDown returns a float f with f*f <= d, proven by exact comparison —
// ratSqrtDown's contract, over this file's arithmetic. The float sqrt seeds the
// answer; the exact test decides it, so no platform's sqrt accuracy can widen
// or invert the bracket.
func DySqrtDown(d Dyadic) float64 {
	if d.Sign() <= 0 {
		return 0
	}
	f := DySqrtSeed(d)
	if isNonFinite(f) {
		// sqrt(d) is at or beyond the top of the range, so the largest float
		// there starts the walk; the exact test still decides it.
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if DySquareAtMost(f, d) {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

// DySqrtUp returns a float f with f*f >= d, proven by exact comparison —
// ratSqrtUp's contract, over this file's arithmetic. It returns +Inf only where
// sqrt(d) genuinely exceeds MaxFloat64.
func DySqrtUp(d Dyadic) float64 {
	if d.Sign() <= 0 {
		return 0
	}
	f := DySqrtSeed(d)
	if isNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if !DySquareAtMost(f, d) || DySquareEquals(f, d) {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

// DyInt lifts an integer exactly.
func DyInt(v int64) Dyadic { return dyInt64(v, 0) }

// DyShift returns d × 2^n. It is the only scaling this arithmetic performs
// without a multiplication, and the only division it performs at all: a
// division by a power of two is a negative n, which is why a quadrature whose
// nodes and weights are binary fractions never leaves the Dyadic set.
func DyShift(d Dyadic, n int) Dyadic {
	if d.IsZero() {
		return Dyadic{}
	}
	d.exp += n
	return d
}

// DyFloatDown returns the largest float64 at or below d, or the value unchanged
// where it is already a float — ratFloatDown's contract, over this file's
// arithmetic. A saturating infinity is returned as it stands, a REFUSAL rather
// than a bound, exactly as its rational twin does.
func DyFloatDown(d Dyadic) float64 {
	f, exact := d.Float64()
	if isNonFinite(f) || exact {
		return f
	}
	if fr, ok := DyOf(f); ok && DyCmp(fr, d) <= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(-1))
}

// DyFloatUp returns the smallest float64 at or above d — ratFloatUp's contract,
// over this file's arithmetic.
func DyFloatUp(d Dyadic) float64 {
	f, exact := d.Float64()
	if isNonFinite(f) || exact {
		return f
	}
	if fr, ok := DyOf(f); ok && DyCmp(fr, d) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

// DyadicFloatError returns |exact − held| rounded upward — rationalFloatError's
// contract, over this file's arithmetic. A held value that does not lift is an
// unbounded error, never a zero.
func DyadicFloatError(exact Dyadic, held float64) float64 {
	heldDy, ok := DyOf(held)
	if !ok {
		return math.Inf(1)
	}
	return DyFloatUp(DyAbs(DySubScalar(exact, heldDy)))
}

// DyNearestUp converts d to the NEAREST float64 and steps it one ulp toward
// +Inf when that conversion was inexact — the publication rule
// rationalFloatError and ratL1Upper apply to a big.Rat, bit for bit. It is not
// DyFloatUp, the tight ceiling: where the nearest float already lies above d,
// this answer is one ulp above DyFloatUp's, and a caller that must reproduce
// the rational twins' published value needs this one.
func DyNearestUp(d Dyadic) float64 {
	f, exact := d.Float64()
	if !exact {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// DyRoundedFloatError returns |exact − held| under DyNearestUp's rounding —
// rationalFloatError's contract and published value over this file's
// arithmetic. A held value that does not lift is an unbounded error, never a
// zero.
func DyRoundedFloatError(exact Dyadic, held float64) float64 {
	heldDy, ok := DyOf(held)
	if !ok {
		return math.Inf(1)
	}
	return DyNearestUp(DyAbs(DySubScalar(exact, heldDy)))
}

// DyLerp is ratLerp over this file's arithmetic: the exact value of
// P(t) = start + t·(end − start), a polynomial in three held floats and hence
// a Dyadic. At the two natural bounds the answer is the record's own
// coordinate, exactly as ratLerp and lerp2 read it, and a non-finite far
// endpoint still refuses there. ok is false exactly where ratLerp answers nil.
func DyLerp(start, end, t float64) (Dyadic, bool) {
	if t == 0 || t == 1 {
		near, far := start, end
		if t == 1 {
			near, far = end, start
		}
		if math.IsNaN(far) || math.IsInf(far, 0) {
			return Dyadic{}, false
		}
		return DyOf(near)
	}
	s, okS := DyOf(start)
	e, okE := DyOf(end)
	dt, okT := DyOf(t)
	if !okS || !okE || !okT {
		return Dyadic{}, false
	}
	return DyAdd(s, DyMul(dt, DySubScalar(e, s))), true
}

// DyL1Upper is ratL1Upper over this file's arithmetic: the exact sum of the
// values' magnitudes, published through DyNearestUp.
func DyL1Upper(values ...Dyadic) float64 {
	total := DyZero()
	for _, value := range values {
		total = DyAdd(total, DyAbs(value))
	}
	return DyNearestUp(total)
}

// Mant returns the mantissa for internal proof consumers, or nil for zero. A
// mantissa held inline is copied into a new big.Int, so a caller on a hot path
// passes its own storage to MantInto instead. The result must not be mutated:
// a mantissa held in a big.Int is returned as it stands.
func (d Dyadic) Mant() *big.Int {
	switch {
	case d.big != nil:
		return d.big
	case d.IsZero():
		return nil
	default:
		return d.MantInto(new(big.Int))
	}
}

// MantInto sets z to the mantissa (zero for a zero value) and returns z.
func (d Dyadic) MantInto(z *big.Int) *big.Int {
	if d.big != nil {
		return z.Set(d.big)
	}
	var t big.Int
	var buf dyBuf
	return z.Set(d.view(&t, &buf))
}

// Exp returns the binary exponent for internal proof consumers.
func (d Dyadic) Exp() int { return d.exp }

func isNonFinite(f float64) bool { return !finite(f) }

// SqrtAdjustLimit bounds directed rounding from a correctly rounded square root.
const SqrtAdjustLimit = 8
