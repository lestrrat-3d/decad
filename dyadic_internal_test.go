package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// dyadicProbeFloats are the held coordinates every case below is built from:
// round values, values no binary fraction represents exactly, subnormal and
// near-overflow magnitudes, both signs, and zero. They are deliberately a
// spread of BINARY exponents, since the exponent is what this representation
// carries separately and what a test of it must vary.
var dyadicProbeFloats = []float64{
	0, 1, -1, 0.5, -0.5, 2, 1024, -1024,
	0.1, -0.1, 1.0 / 3.0, -2.0 / 3.0, 12.7, -3.30000000001,
	math.Pi, -math.E, 1e-300, -1e-300, 1e300, -1e300,
	math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
	math.MaxFloat64, -math.MaxFloat64,
	math.Nextafter(1, 2), math.Nextafter(1, 0),
}

// ratOfDyadic is the test's own independent reading of a dyadic: mant × 2^exp
// composed through big.Rat rather than through the type's own rat method, so a
// bug shared by construction and conversion cannot hide behind itself.
func ratOfDyadic(t *testing.T, d dyadic) *big.Rat {
	t.Helper()
	if d.mant == nil {
		return new(big.Rat)
	}
	out := new(big.Rat).SetInt(d.mant)
	scale := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(abs(d.exp))))
	if d.exp >= 0 {
		return out.Mul(out, scale)
	}
	return out.Quo(out, scale)
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

// TestDyadicLiftsEveryFloatExactly is the representation's own premise: a
// float64 IS a dyadic rational, so the lift loses nothing and reads back as the
// number big.Rat holds for it.
func TestDyadicLiftsEveryFloatExactly(t *testing.T) {
	t.Parallel()
	for _, f := range dyadicProbeFloats {
		d, ok := dyOf(f)
		require.True(t, ok, "%v is finite and must lift", f)
		require.Zero(t, ratOfDyadic(t, d).Cmp(floatRat(f)), "the lift of %v must equal its exact rational", f)
		require.Zero(t, d.rat().Cmp(floatRat(f)), "rat must return the same number the lift holds, for %v", f)

		back, exact := d.float64()
		require.True(t, exact, "a value lifted from a float64 must convert back exactly, for %v", f)
		require.Equal(t, f, back, "the round trip must return the same float, for %v", f)
	}
}

// TestDyadicRefusesNonFiniteFloats pins the gate: no exact proof may consume a
// NaN or an infinity, so the lift reports rather than inventing a number.
func TestDyadicRefusesNonFiniteFloats(t *testing.T) {
	t.Parallel()
	for _, f := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		_, ok := dyOf(f)
		require.False(t, ok, "%v must not lift", f)
	}
}

// TestMustDyOfPanicsOnANonFiniteFloat pins the contract that a broken CALLER
// claim fails loudly. Answering a zero instead would put an exact, confident,
// wrong number into a proof and let it publish a bound nothing established,
// which is the one failure this package must never have; the panic names the
// caller that built the bad value. Every caller gates with finiteVec first, so
// nothing reaches this in normal operation.
func TestMustDyOfPanicsOnANonFiniteFloat(t *testing.T) {
	t.Parallel()
	for _, f := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		require.Panics(t, func() { mustDyOf(f) }, "%v must not lift silently", f)
	}
	require.NotPanics(t, func() { mustDyOf(math.MaxFloat64) }, "a finite value still lifts")
	require.Panics(t, func() { dyVec(r3.Vec{X: 1, Y: math.Inf(1), Z: 3}) },
		"a vector carrying a non-finite component must not lift silently either")
}

// TestDyadicStaysReduced pins the canonical form the type promises: a non-zero
// mantissa is odd, so two dyadics hold equal numbers exactly when their fields
// agree. Without it a value would have unboundedly many representations and its
// mantissas would grow with every alignment.
func TestDyadicStaysReduced(t *testing.T) {
	t.Parallel()
	reduced := func(d dyadic, what string) {
		t.Helper()
		if d.mant == nil || d.mant.Sign() == 0 {
			require.Zero(t, d.exp, "%s: a zero must carry exponent 0", what)
			return
		}
		require.Zero(t, d.mant.TrailingZeroBits(), "%s: a non-zero mantissa must be odd", what)
	}
	for _, f := range dyadicProbeFloats {
		reduced(mustDyOf(f), "the lift of a float")
	}
	// 3 + 5 = 8 is the case that forces the point: both operands are odd and
	// their sum is a pure power of two, so an unreduced result would carry a
	// mantissa three bits wider than it needs.
	reduced(dyAdd(mustDyOf(3), mustDyOf(5)), "3+5")
	reduced(dySubScalar(mustDyOf(8), mustDyOf(7)), "8-7")
	reduced(dyAdd(mustDyOf(0.5), mustDyOf(0.5)), "0.5+0.5")
	reduced(dySubScalar(mustDyOf(1), mustDyOf(1)), "1-1")

	// A cancelling sum is exactly zero, not a zero mantissa at some exponent.
	z := dySubScalar(mustDyOf(math.Pi), mustDyOf(math.Pi))
	require.True(t, z.isZero())
	require.Zero(t, z.exp)
}

// TestDyadicArithmeticMatchesBigRat is the substitution's whole justification:
// over a spread of held floats, every dyadic operation returns the number
// big.Rat returns for it, compared EXACTLY rather than within a tolerance.
func TestDyadicArithmeticMatchesBigRat(t *testing.T) {
	t.Parallel()
	for _, a := range dyadicProbeFloats {
		for _, b := range dyadicProbeFloats {
			da, db := mustDyOf(a), mustDyOf(b)
			ra, rb := floatRat(a), floatRat(b)

			require.Zero(t, ratOfDyadic(t, dyAdd(da, db)).Cmp(new(big.Rat).Add(ra, rb)),
				"%v + %v", a, b)
			require.Zero(t, ratOfDyadic(t, dySubScalar(da, db)).Cmp(new(big.Rat).Sub(ra, rb)),
				"%v - %v", a, b)
			require.Zero(t, ratOfDyadic(t, dyMul(da, db)).Cmp(new(big.Rat).Mul(ra, rb)),
				"%v * %v", a, b)
			require.Equal(t, ra.Cmp(rb), dyCmp(da, db), "cmp(%v, %v)", a, b)
			require.Equal(t, ra.Sign(), da.sign(), "sign(%v)", a)
			require.Zero(t, ratOfDyadic(t, dyAbs(da)).Cmp(new(big.Rat).Abs(ra)), "abs(%v)", a)
			require.Zero(t, ratOfDyadic(t, dyNeg(da)).Cmp(new(big.Rat).Neg(ra)), "neg(%v)", a)
		}
	}
}

// TestDyadicVectorMatchesBigRatVector is the same equality one level up, at the
// cross, dot and difference every exact predicate in this package is built
// from. A cross product is where a wrong exponent would show first, since its
// two products must be aligned before they are subtracted.
func TestDyadicVectorMatchesBigRatVector(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x9E3779B97F4A7C15, 0xBF58476D1CE4E5B9))
	pick := func() float64 {
		// A held coordinate spread over 60 binary exponents, so the alignment
		// shifts the arithmetic performs are genuinely large rather than
		// uniformly small.
		return (rng.Float64() - 0.5) * math.Ldexp(1, rng.IntN(60)-30)
	}
	vec := func() r3.Vec { return r3.Vec{X: pick(), Y: pick(), Z: pick()} }

	// The reference is written out in big.Rat here rather than called from the
	// package: it is the implementation this type REPLACED, kept as the test's
	// own independent oracle so the comparison is against math/big's answer
	// and not against another copy of the code under test.
	type ratRef [3]*big.Rat
	refVec := func(v r3.Vec) ratRef { return ratRef{floatRat(v.X), floatRat(v.Y), floatRat(v.Z)} }
	refSub := func(a, b ratRef) ratRef {
		var out ratRef
		for i := range out {
			out[i] = new(big.Rat).Sub(a[i], b[i])
		}
		return out
	}
	refCross := func(a, b ratRef) ratRef {
		mul := func(x, y *big.Rat) *big.Rat { return new(big.Rat).Mul(x, y) }
		return ratRef{
			new(big.Rat).Sub(mul(a[1], b[2]), mul(a[2], b[1])),
			new(big.Rat).Sub(mul(a[2], b[0]), mul(a[0], b[2])),
			new(big.Rat).Sub(mul(a[0], b[1]), mul(a[1], b[0])),
		}
	}
	refDot := func(a, b ratRef) *big.Rat {
		out := new(big.Rat)
		for i := range a {
			out.Add(out, new(big.Rat).Mul(a[i], b[i]))
		}
		return out
	}

	for range 300 {
		va, vb := vec(), vec()
		da, db := dyVec(va), dyVec(vb)
		ra, rb := refVec(va), refVec(vb)

		require.Zero(t, ratOfDyadic(t, dvDot(da, db)).Cmp(refDot(ra, rb)), "dot of %v and %v", va, vb)

		gotCross, wantCross := dvCross(da, db), refCross(ra, rb)
		for i := range gotCross {
			require.Zero(t, ratOfDyadic(t, gotCross[i]).Cmp(wantCross[i]),
				"cross component %d of %v and %v", i, va, vb)
		}
		gotSub, wantSub := dvSub(da, db), refSub(ra, rb)
		for i := range gotSub {
			require.Zero(t, ratOfDyadic(t, gotSub[i]).Cmp(wantSub[i]),
				"difference component %d of %v and %v", i, va, vb)
		}
		gotAdd := dvAdd(da, db)
		for i := range gotAdd {
			require.Zero(t, ratOfDyadic(t, gotAdd[i]).Cmp(new(big.Rat).Add(ra[i], rb[i])),
				"sum component %d of %v and %v", i, va, vb)
		}
		require.True(t, dvIsZero(dvSub(da, da)), "a self-difference is zero")
		require.False(t, dvIsZero(dvSub(da, db)), "two distinct random vectors differ")
	}
}

// TestDyadicSqrtBracketsMatchTheRationalOnes pins the directed square-root
// walks against the rational ones they restate: the same float, proven by the
// same exact comparison, for a spread of magnitudes.
func TestDyadicSqrtBracketsMatchTheRationalOnes(t *testing.T) {
	t.Parallel()
	for _, f := range dyadicProbeFloats {
		if f <= 0 {
			continue
		}
		d, q := mustDyOf(f), floatRat(f)
		require.Equal(t, ratSqrtDown(q), dySqrtDown(d), "sqrt down of %v", f)
		require.Equal(t, ratSqrtUp(q), dySqrtUp(d), "sqrt up of %v", f)

		// The bracket is PROVEN, not merely close: the down leg squares to at
		// most the value and the up leg to at least it, decided exactly.
		require.True(t, dySquareAtMost(dySqrtDown(d), d), "the down leg must square to at most %v", f)
		if up := dySqrtUp(d); !isNonFinite(up) {
			require.False(t, dyCmp(dyMul(mustDyOf(up), mustDyOf(up)), d) < 0,
				"the up leg must square to at least %v", f)
		}
	}
	require.Zero(t, dySqrtDown(mustDyOf(-1)), "a negative value brackets at zero, as its rational twin does")
	require.Zero(t, dySqrtUp(mustDyOf(-1)))
}

// TestDyadicDirectedRoundingMatchesTheRationalOnes pins dyFloatDown/dyFloatUp
// against ratFloatDown/ratFloatUp on values that are deliberately NOT floats:
// a mid-ulp third of a sum is where a directed rounding either steps or does
// not, and where an off-by-one would show.
func TestDyadicDirectedRoundingMatchesTheRationalOnes(t *testing.T) {
	t.Parallel()
	for _, f := range dyadicProbeFloats {
		d := mustDyOf(f)
		// A value needing more than 53 significant bits: the float itself plus
		// one ulp of its own smallest neighbour, which no float64 holds.
		wide := dyAdd(dyMul(d, d), dyShift(mustDyOf(1), -1080))
		require.Equal(t, ratFloatDown(ratOfDyadic(t, wide)), dyFloatDown(wide), "float down of the widened %v", f)
		require.Equal(t, ratFloatUp(ratOfDyadic(t, wide)), dyFloatUp(wide), "float up of the widened %v", f)
		require.Equal(t, f, dyFloatDown(d), "a value that IS a float rounds to itself, for %v", f)
		require.Equal(t, f, dyFloatUp(d), "a value that IS a float rounds to itself, for %v", f)
	}
}

// TestDyadicShiftAndIntAreExact pins the two constructors the quadratures use:
// scaling by a power of two moves no bit, and a small integer lifts exactly.
func TestDyadicShiftAndIntAreExact(t *testing.T) {
	t.Parallel()
	for _, v := range []int64{0, 1, -1, 3, -7, 24, 1 << 40, -(1 << 40)} {
		require.Zero(t, ratOfDyadic(t, dyInt(v)).Cmp(new(big.Rat).SetInt64(v)), "lift of %d", v)
	}
	for _, n := range []int{-1080, -8, -1, 0, 1, 8, 1080} {
		for _, f := range dyadicProbeFloats {
			d := dyShift(mustDyOf(f), n)
			want := new(big.Rat).Mul(floatRat(f), new(big.Rat).SetFrac(
				new(big.Int).Lsh(big.NewInt(1), uint(max(n, 0))),
				new(big.Int).Lsh(big.NewInt(1), uint(max(-n, 0)))))
			require.Zero(t, ratOfDyadic(t, d).Cmp(want), "%v shifted by %d", f, n)
		}
	}
}

// TestDyadicOfRatAcceptsOnlyBinaryFractions pins the inbound boundary: a
// big.Rat whose denominator is a power of two is a dyadic and lifts exactly; a
// third is not, and is REFUSED rather than rounded, since a rounded lift would
// make a proof about a number the caller never held.
func TestDyadicOfRatAcceptsOnlyBinaryFractions(t *testing.T) {
	t.Parallel()
	for _, r := range []*big.Rat{
		big.NewRat(1, 2), big.NewRat(-7, 8), big.NewRat(5, 1), new(big.Rat),
		floatRat(0.1), floatRat(math.MaxFloat64),
	} {
		d, ok := dyOfRat(r)
		require.True(t, ok, "%s has a power-of-two denominator", r.RatString())
		require.Zero(t, ratOfDyadic(t, d).Cmp(r), "%s must lift exactly", r.RatString())
	}
	for _, r := range []*big.Rat{big.NewRat(1, 3), big.NewRat(2, 6), big.NewRat(-5, 12)} {
		_, ok := dyOfRat(r)
		require.False(t, ok, "%s is not a binary fraction and must be refused", r.RatString())
	}
	_, ok := dyOfRat(nil)
	require.False(t, ok, "a nil rational is not a number to lift")
}

// TestDyadicZeroValueIsUsable pins the property the port depends on: the
// struct's zero value is a valid exact zero, so a dyV3 can be declared with var
// and filled in component by component the way its big.Rat predecessor could.
func TestDyadicZeroValueIsUsable(t *testing.T) {
	t.Parallel()
	var d dyadic
	require.True(t, d.isZero())
	require.Zero(t, d.sign())
	require.Zero(t, d.rat().Sign())

	var v dyV3
	require.True(t, dvIsZero(v))
	require.True(t, dvIsZero(dvCross(v, dyVec(r3.Vec{X: 1, Y: 2, Z: 3}))))
	require.True(t, dvDot(v, dyVec(r3.Vec{X: 1, Y: 2, Z: 3})).isZero())

	one := mustDyOf(1)
	require.Zero(t, dyCmp(dyAdd(d, one), one), "adding the zero value changes nothing")
	require.Zero(t, dyCmp(dySubScalar(one, d), one), "subtracting it changes nothing")
	require.True(t, dyMul(d, one).isZero(), "multiplying by it gives zero")
}
