package proof

import (
	"math"
	"math/big"

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

// Dyadic is an exact binary-scaled rational: mant × 2^exp, with mant an
// arbitrary-precision integer and exp a binary exponent.
//
// The representation is kept REDUCED — a non-zero mant is odd — so that two
// dyadics are equal exactly when their fields are (norm). Zero is the one value
// with an even mantissa, held as mant 0 at exp 0.
//
// The zero VALUE of the struct (a nil mant) is a valid zero, which is what lets
// a DyV3 be declared with var and filled in component by component the way its
// big.Rat predecessor could be.
type Dyadic struct {
	mant *big.Int
	exp  int
}

// DyZero is the additive identity, and what a Dyadic's zero value denotes.
func DyZero() Dyadic { return Dyadic{} }

// sign reports the value's sign, which is its mantissa's: the scale factor
// 2^exp is positive for every exp.
func (d Dyadic) Sign() int {
	if d.mant == nil {
		return 0
	}
	return d.mant.Sign()
}

// isZero reports whether the value is exactly zero.
func (d Dyadic) IsZero() bool { return d.Sign() == 0 }

// norm reduces d to the canonical form this type promises: a zero mantissa
// carries exponent 0, and a non-zero one is made odd by shifting its trailing
// zero bits into the exponent. It mutates d's own mantissa, so it is called
// only on a mantissa this package just allocated.
func (d Dyadic) norm() Dyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return Dyadic{}
	}
	if shift := d.mant.TrailingZeroBits(); shift > 0 {
		d.mant.Rsh(d.mant, shift)
		d.exp += int(shift)
	}
	return d
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
	return DyOfFiniteInto(f, new(big.Int)), true
}

// DyOfFiniteInto lifts a finite, nonzero float into caller-owned integer
// storage. Its callers gate zero and non-finite inputs before reaching it.
// Reusing mant changes only storage; the Frexp scale and norm match DyOf.
func DyOfFiniteInto(f float64, mant *big.Int) Dyadic {
	frac, exp := math.Frexp(f)
	mant.SetInt64(int64(frac * (1 << 53)))
	return Dyadic{mant: mant, exp: exp - 53}.norm()
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
func DyAdd(a, b Dyadic) Dyadic {
	switch {
	case a.IsZero() && b.IsZero():
		return Dyadic{}
	case a.IsZero():
		return Dyadic{mant: new(big.Int).Set(b.mant), exp: b.exp}
	case b.IsZero():
		return Dyadic{mant: new(big.Int).Set(a.mant), exp: a.exp}
	case a.exp == b.exp:
		return Dyadic{mant: new(big.Int).Add(a.mant, b.mant), exp: a.exp}.norm()
	case a.exp > b.exp:
		out := new(big.Int).Lsh(a.mant, uint(a.exp-b.exp))
		return Dyadic{mant: out.Add(out, b.mant), exp: b.exp}.norm()
	default:
		out := new(big.Int).Lsh(b.mant, uint(b.exp-a.exp))
		return Dyadic{mant: out.Add(a.mant, out), exp: a.exp}.norm()
	}
}

// DySubScalar returns a − b exactly.
func DySubScalar(a, b Dyadic) Dyadic {
	switch {
	case a.IsZero() && b.IsZero():
		return Dyadic{}
	case a.IsZero():
		return Dyadic{mant: new(big.Int).Neg(b.mant), exp: b.exp}
	case b.IsZero():
		return Dyadic{mant: new(big.Int).Set(a.mant), exp: a.exp}
	case a.exp == b.exp:
		return Dyadic{mant: new(big.Int).Sub(a.mant, b.mant), exp: a.exp}.norm()
	case a.exp > b.exp:
		out := new(big.Int).Lsh(a.mant, uint(a.exp-b.exp))
		return Dyadic{mant: out.Sub(out, b.mant), exp: b.exp}.norm()
	default:
		out := new(big.Int).Lsh(b.mant, uint(b.exp-a.exp))
		return Dyadic{mant: out.Sub(a.mant, out), exp: a.exp}.norm()
	}
}

// DyMul returns a × b exactly. Exponents add, so no alignment is needed and the
// product of two reduced mantissas is already reduced.
func DyMul(a, b Dyadic) Dyadic {
	if a.IsZero() || b.IsZero() {
		return Dyadic{}
	}
	return Dyadic{mant: new(big.Int).Mul(a.mant, b.mant), exp: a.exp + b.exp}
}

// DyCmp compares a against b, returning -1, 0 or +1 the way big.Rat.Cmp does.
//
// Two values of opposite signs compare by sign, and two of one sign whose
// magnitudes' leading bits sit at different binary positions compare by that
// position: |m|·2^e lies in [2^(n+e−1), 2^(n+e)) for an n-bit mantissa m. Only
// values whose leading bits coincide pay the aligning shift.
func DyCmp(a, b Dyadic) int {
	switch {
	case a.IsZero():
		return -b.Sign()
	case b.IsZero():
		return a.Sign()
	case a.exp == b.exp:
		return a.mant.Cmp(b.mant)
	}
	sign := a.mant.Sign()
	if sign != b.mant.Sign() {
		return sign
	}
	if top, other := a.mant.BitLen()+a.exp, b.mant.BitLen()+b.exp; top != other {
		if top > other {
			return sign
		}
		return -sign
	}
	switch {
	case a.exp > b.exp:
		return new(big.Int).Lsh(a.mant, uint(a.exp-b.exp)).Cmp(b.mant)
	default:
		return a.mant.Cmp(new(big.Int).Lsh(b.mant, uint(b.exp-a.exp)))
	}
}

// DyAbs returns |d|.
func DyAbs(d Dyadic) Dyadic {
	if d.mant == nil {
		return Dyadic{}
	}
	return Dyadic{mant: new(big.Int).Abs(d.mant), exp: d.exp}
}

// DyNeg returns −d.
func DyNeg(d Dyadic) Dyadic {
	if d.mant == nil {
		return Dyadic{}
	}
	return Dyadic{mant: new(big.Int).Neg(d.mant), exp: d.exp}
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
	if d.mant == nil || d.mant.Sign() == 0 {
		return new(big.Rat)
	}
	r := new(big.Rat).SetInt(d.mant)
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
	return Dyadic{mant: new(big.Int).Set(r.Num()), exp: -int(shift)}.norm(), true
}

// float64 returns the value as a float64 plus whether that conversion was
// exact, matching big.Rat.Float64's own contract so a caller rounding outward
// can tell whether it must.
func (d Dyadic) Float64() (float64, bool) {
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
	var out DyV3
	for i := range out {
		out[i] = DySubScalar(a[i], b[i])
	}
	return out
}

// DvAdd returns a + b componentwise.
func DvAdd(a, b DyV3) DyV3 {
	var out DyV3
	for i := range out {
		out[i] = DyAdd(a[i], b[i])
	}
	return out
}

// DvCross returns a × b exactly.
func DvCross(a, b DyV3) DyV3 {
	return DyV3{
		DySubScalar(DyMul(a[1], b[2]), DyMul(a[2], b[1])),
		DySubScalar(DyMul(a[2], b[0]), DyMul(a[0], b[2])),
		DySubScalar(DyMul(a[0], b[1]), DyMul(a[1], b[0])),
	}
}

// DvDot returns a · b exactly.
func DvDot(a, b DyV3) Dyadic {
	out := DyZero()
	for i := range a {
		out = DyAdd(out, DyMul(a[i], b[i]))
	}
	return out
}

// DvIsZero reports whether every component is exactly zero.
func DvIsZero(a DyV3) bool {
	return a[0].IsZero() && a[1].IsZero() && a[2].IsZero()
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
func DyInt(v int64) Dyadic {
	if v == 0 {
		return Dyadic{}
	}
	return Dyadic{mant: big.NewInt(v), exp: 0}.norm()
}

// DyShift returns d × 2^n. It is the only scaling this arithmetic performs
// without a multiplication, and the only division it performs at all: a
// division by a power of two is a negative n, which is why a quadrature whose
// nodes and weights are binary fractions never leaves the Dyadic set.
func DyShift(d Dyadic, n int) Dyadic {
	if d.mant == nil || d.mant.Sign() == 0 {
		return Dyadic{}
	}
	return Dyadic{mant: new(big.Int).Set(d.mant), exp: d.exp + n}
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

// Mant returns the held mantissa for internal proof consumers.
func (d Dyadic) Mant() *big.Int { return d.mant }

// Exp returns the binary exponent for internal proof consumers.
func (d Dyadic) Exp() int { return d.exp }

func isNonFinite(f float64) bool { return !finite(f) }

// SqrtAdjustLimit bounds directed rounding from a correctly rounded square root.
const SqrtAdjustLimit = 8
