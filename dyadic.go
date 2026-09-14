package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the package's exact BINARY-SCALED arithmetic: dyadic, the exact
// scalar every proof over held float64 coordinates is carried in, and dyV3, the
// exact vector built on it.
//
// Every float64 is exactly a dyadic rational — a mantissa times a power of two
// (math.Frexp) — and the closure of that set under +, − and × is itself. So a
// determinant, a cross product, a dot product or any other polynomial in held
// coordinates is dyadic too, and needs no general fraction to represent it.
//
// big.Rat represents such a value correctly but pays for generality it never
// uses: it reduces to lowest terms after EVERY operation, and that reduction is
// a Lehmer GCD over the full numerator and denominator. On a dyadic value the
// answer is always a power of two, so the GCD computes something the exponent
// already states. Stripping the mantissa's trailing zero bits reaches the same
// reduced form with a bit count and a shift (norm).
//
// So the rule this file exists to enforce: a quantity whose whole derivation is
// +, − and × over held floats is a dyadic, never a big.Rat. A quantity that
// genuinely LEAVES that set — a moment integral's division by (i+1)(j+1), a
// factorial denominator, any ratio of two computed values — converts at exactly
// the point it divides (rat), and every such point is the boundary between this
// file's arithmetic and math/big's.
//
// Nothing here is a tolerance, an approximation or a widened float path. A
// dyadic holds the same number big.Rat held, bit for bit, and every comparison
// it answers is the comparison big.Rat answered.

// dyadic is an exact binary-scaled rational: mant × 2^exp, with mant an
// arbitrary-precision integer and exp a binary exponent.
//
// The representation is kept REDUCED — a non-zero mant is odd — so that two
// dyadics are equal exactly when their fields are (norm). Zero is the one value
// with an even mantissa, held as mant 0 at exp 0.
//
// The zero VALUE of the struct (a nil mant) is a valid zero, which is what lets
// a dyV3 be declared with var and filled in component by component the way its
// big.Rat predecessor could be.
type dyadic struct {
	mant *big.Int
	exp  int
}

// dyZero is the additive identity, and what a dyadic's zero value denotes.
func dyZero() dyadic { return dyadic{} }

// sign reports the value's sign, which is its mantissa's: the scale factor
// 2^exp is positive for every exp.
func (d dyadic) sign() int {
	if d.mant == nil {
		return 0
	}
	return d.mant.Sign()
}

// isZero reports whether the value is exactly zero.
func (d dyadic) isZero() bool { return d.sign() == 0 }

// norm reduces d to the canonical form this type promises: a zero mantissa
// carries exponent 0, and a non-zero one is made odd by shifting its trailing
// zero bits into the exponent. It mutates d's own mantissa, so it is called
// only on a mantissa this package just allocated.
func (d dyadic) norm() dyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return dyadic{}
	}
	if shift := d.mant.TrailingZeroBits(); shift > 0 {
		d.mant.Rsh(d.mant, shift)
		d.exp += int(shift)
	}
	return d
}

// dyOf lifts a float64 exactly, reporting false for a NaN or an infinity, which
// no exact proof may consume. The lift is exact by construction: math.Frexp
// splits the value into a fraction in [0.5, 1) and a binary exponent, and
// scaling that fraction by 2^53 makes it an integer without moving a bit.
func dyOf(f float64) (dyadic, bool) {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return dyadic{}, false
	}
	if f == 0 {
		return dyadic{}, true
	}
	frac, exp := math.Frexp(f)
	return dyadic{mant: big.NewInt(int64(frac * (1 << 53))), exp: exp - 53}.norm(), true
}

// mustDyOf is dyOf for a value the caller has already proven finite. A
// non-finite value reaching it is a missing gate in the caller, and the zero it
// returns would be a silently wrong proof, so it is never called on an
// unchecked float.
func mustDyOf(f float64) dyadic {
	d, ok := dyOf(f)
	if !ok {
		return dyadic{}
	}
	return d
}

// dyAlign restates a and b over one common exponent — the smaller of the two,
// so neither mantissa loses a bit — and returns the two restated mantissas
// beside the exponent they now share.
func dyAlign(a, b dyadic) (*big.Int, *big.Int, int) {
	am, bm := a.mant, b.mant
	if am == nil {
		am = new(big.Int)
	}
	if bm == nil {
		bm = new(big.Int)
	}
	switch {
	case a.isZero():
		return new(big.Int), new(big.Int).Set(bm), b.exp
	case b.isZero():
		return new(big.Int).Set(am), new(big.Int), a.exp
	case a.exp == b.exp:
		return new(big.Int).Set(am), new(big.Int).Set(bm), a.exp
	case a.exp > b.exp:
		return new(big.Int).Lsh(am, uint(a.exp-b.exp)), new(big.Int).Set(bm), b.exp
	default:
		return new(big.Int).Set(am), new(big.Int).Lsh(bm, uint(b.exp-a.exp)), a.exp
	}
}

// dyAdd returns a + b exactly.
func dyAdd(a, b dyadic) dyadic {
	am, bm, exp := dyAlign(a, b)
	return dyadic{mant: am.Add(am, bm), exp: exp}.norm()
}

// dySubScalar returns a − b exactly.
func dySubScalar(a, b dyadic) dyadic {
	am, bm, exp := dyAlign(a, b)
	return dyadic{mant: am.Sub(am, bm), exp: exp}.norm()
}

// dyMul returns a × b exactly. Exponents add, so no alignment is needed and the
// product of two reduced mantissas is already reduced.
func dyMul(a, b dyadic) dyadic {
	if a.isZero() || b.isZero() {
		return dyadic{}
	}
	return dyadic{mant: new(big.Int).Mul(a.mant, b.mant), exp: a.exp + b.exp}
}

// dyCmp compares a against b, returning -1, 0 or +1 the way big.Rat.Cmp does.
func dyCmp(a, b dyadic) int {
	am, bm, _ := dyAlign(a, b)
	return am.Cmp(bm)
}

// dyAbs returns |d|.
func dyAbs(d dyadic) dyadic {
	if d.mant == nil {
		return dyadic{}
	}
	return dyadic{mant: new(big.Int).Abs(d.mant), exp: d.exp}
}

// dyNeg returns −d.
func dyNeg(d dyadic) dyadic {
	if d.mant == nil {
		return dyadic{}
	}
	return dyadic{mant: new(big.Int).Neg(d.mant), exp: d.exp}
}

// rat converts to big.Rat, for the callers whose arithmetic genuinely leaves
// the dyadic set — a moment integral dividing by (i+1)(j+1), a factorial
// denominator. It is the ONE boundary between this file and math/big's general
// fractions, and it is exact: a non-negative exponent scales an integer, and a
// negative one puts an odd mantissa over a power of two, which is already in
// lowest terms.
func (d dyadic) rat() *big.Rat {
	if d.mant == nil || d.mant.Sign() == 0 {
		return new(big.Rat)
	}
	if d.exp >= 0 {
		return new(big.Rat).SetInt(new(big.Int).Lsh(d.mant, uint(d.exp)))
	}
	return new(big.Rat).SetFrac(d.mant, new(big.Int).Lsh(big.NewInt(1), uint(-d.exp)))
}

// dyOfRat lifts a big.Rat this package knows to be dyadic — one whose
// denominator is a power of two. It reports false for any other fraction rather
// than rounding one, since a rounded value would be a proof about a number the
// caller never held.
func dyOfRat(r *big.Rat) (dyadic, bool) {
	if r == nil {
		return dyadic{}, false
	}
	den := r.Denom()
	shift := den.TrailingZeroBits()
	if den.BitLen() != int(shift)+1 {
		return dyadic{}, false
	}
	return dyadic{mant: new(big.Int).Set(r.Num()), exp: -int(shift)}.norm(), true
}

// float64 returns the value as a float64 plus whether that conversion was
// exact, matching big.Rat.Float64's own contract so a caller rounding outward
// can tell whether it must.
func (d dyadic) float64() (float64, bool) {
	if d.mant == nil || d.mant.Sign() == 0 {
		return 0, true
	}
	// The precision must hold the WHOLE mantissa: a big.Float that rounded in
	// SetInt would report its own last conversion as exact and hide the bit it
	// already dropped, which is precisely the claim a directed rounding must
	// not be given.
	prec := max(uint(d.mant.BitLen()), 53)
	f := new(big.Float).SetPrec(prec).SetInt(d.mant)
	f.SetMantExp(f, d.exp)
	out, acc := f.Float64()
	return out, acc == big.Exact
}

// dyV3 is a vector of the payload's own floats taken EXACTLY — the only
// arithmetic allowed to prove a degeneracy. It is dyadic's vector, component
// for component, and it replaced a [3]*big.Rat whose every operation paid a
// Lehmer GCD to rediscover an exponent this representation states.
type dyV3 [3]dyadic

// dyVec lifts a held vector exactly. Its caller has already proven the vector
// finite (finiteVec), which is what makes the per-component lift total.
func dyVec(v r3.Vec) dyV3 {
	return dyV3{mustDyOf(v.X), mustDyOf(v.Y), mustDyOf(v.Z)}
}

// dvSub returns a − b componentwise.
func dvSub(a, b dyV3) dyV3 {
	var out dyV3
	for i := range out {
		out[i] = dySubScalar(a[i], b[i])
	}
	return out
}

// dvAdd returns a + b componentwise.
func dvAdd(a, b dyV3) dyV3 {
	var out dyV3
	for i := range out {
		out[i] = dyAdd(a[i], b[i])
	}
	return out
}

// dvCross returns a × b exactly.
func dvCross(a, b dyV3) dyV3 {
	return dyV3{
		dySubScalar(dyMul(a[1], b[2]), dyMul(a[2], b[1])),
		dySubScalar(dyMul(a[2], b[0]), dyMul(a[0], b[2])),
		dySubScalar(dyMul(a[0], b[1]), dyMul(a[1], b[0])),
	}
}

// dvDot returns a · b exactly.
func dvDot(a, b dyV3) dyadic {
	out := dyZero()
	for i := range a {
		out = dyAdd(out, dyMul(a[i], b[i]))
	}
	return out
}

// dvIsZero reports whether every component is exactly zero.
func dvIsZero(a dyV3) bool {
	return a[0].isZero() && a[1].isZero() && a[2].isZero()
}

// dySqrtSeed is ratSqrtSeed over a dyadic: a float64 near sqrt(d), used only to
// START the directed walks below, never to decide them. It carries the same
// even-exponent trick its rational twin does — a dyadic already holds its
// binary exponent, so the split the rational version had to compute is a field
// read here.
func dySqrtSeed(d dyadic) float64 {
	// The mantissa is normalised into [0.5, 1) first and its own exponent
	// folded into the total, so a value near either end of the float64 range
	// roots from a fraction rather than from an integer the conversion would
	// saturate.
	mant := new(big.Float).SetPrec(64)
	exp := d.exp + new(big.Float).SetPrec(64).SetInt(d.mant).MantExp(mant)
	if exp%2 != 0 {
		// Halving an odd exponent is not an integer, so shift one power of two
		// into the mantissa, which still roots cleanly.
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// dySquareAtMost reports whether f² <= d, decided exactly.
func dySquareAtMost(f float64, d dyadic) bool {
	square, ok := dyOf(f)
	if !ok {
		return false
	}
	return dyCmp(dyMul(square, square), d) <= 0
}

// dySquareEquals reports whether f² == d, decided exactly.
func dySquareEquals(f float64, d dyadic) bool {
	square, ok := dyOf(f)
	if !ok {
		return false
	}
	return dyCmp(dyMul(square, square), d) == 0
}

// dySqrtDown returns a float f with f*f <= d, proven by exact comparison —
// ratSqrtDown's contract, over this file's arithmetic. The float sqrt seeds the
// answer; the exact test decides it, so no platform's sqrt accuracy can widen
// or invert the bracket.
func dySqrtDown(d dyadic) float64 {
	if d.sign() <= 0 {
		return 0
	}
	f := dySqrtSeed(d)
	if isNonFinite(f) {
		// sqrt(d) is at or beyond the top of the range, so the largest float
		// there starts the walk; the exact test still decides it.
		f = math.MaxFloat64
	}
	for range sqrtAdjustLimit {
		if dySquareAtMost(f, d) {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

// dySqrtUp returns a float f with f*f >= d, proven by exact comparison —
// ratSqrtUp's contract, over this file's arithmetic. It returns +Inf only where
// sqrt(d) genuinely exceeds MaxFloat64.
func dySqrtUp(d dyadic) float64 {
	if d.sign() <= 0 {
		return 0
	}
	f := dySqrtSeed(d)
	if isNonFinite(f) {
		f = math.MaxFloat64
	}
	for range sqrtAdjustLimit {
		if !dySquareAtMost(f, d) || dySquareEquals(f, d) {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

// dyInt lifts an integer exactly.
func dyInt(v int64) dyadic {
	if v == 0 {
		return dyadic{}
	}
	return dyadic{mant: big.NewInt(v), exp: 0}.norm()
}

// dyShift returns d × 2^n. It is the only scaling this arithmetic performs
// without a multiplication, and the only division it performs at all: a
// division by a power of two is a negative n, which is why a quadrature whose
// nodes and weights are binary fractions never leaves the dyadic set.
func dyShift(d dyadic, n int) dyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return dyadic{}
	}
	return dyadic{mant: new(big.Int).Set(d.mant), exp: d.exp + n}
}

// dyFloatDown returns the largest float64 at or below d, or the value unchanged
// where it is already a float — ratFloatDown's contract, over this file's
// arithmetic. A saturating infinity is returned as it stands, a REFUSAL rather
// than a bound, exactly as its rational twin does.
func dyFloatDown(d dyadic) float64 {
	f, exact := d.float64()
	if isNonFinite(f) || exact {
		return f
	}
	if fr, ok := dyOf(f); ok && dyCmp(fr, d) <= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(-1))
}

// dyFloatUp returns the smallest float64 at or above d — ratFloatUp's contract,
// over this file's arithmetic.
func dyFloatUp(d dyadic) float64 {
	f, exact := d.float64()
	if isNonFinite(f) || exact {
		return f
	}
	if fr, ok := dyOf(f); ok && dyCmp(fr, d) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

// dyadicFloatError returns |exact − held| rounded upward — rationalFloatError's
// contract, over this file's arithmetic. A held value that does not lift is an
// unbounded error, never a zero.
func dyadicFloatError(exact dyadic, held float64) float64 {
	heldDy, ok := dyOf(held)
	if !ok {
		return math.Inf(1)
	}
	return dyFloatUp(dyAbs(dySubScalar(exact, heldDy)))
}
