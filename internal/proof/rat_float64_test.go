package proof_test

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"os"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// ratFloat64Randoms is the number of random floats each differential test
// draws. DECAD_RATFLOAT64_RANDOM raises it for a one-off long run.
func ratFloat64Randoms() int {
	if v, err := strconv.Atoi(os.Getenv("DECAD_RATFLOAT64_RANDOM")); err == nil && v > 0 {
		return v
	}
	return 2000
}

// legacyXhpFractionFloat64 is xhpFractionFloat64 as it was written over
// big.Int.QuoRem.
func legacyXhpFractionFloat64(num, den *big.Int) float64 {
	if num.Sign() == 0 {
		return 0
	}
	var a, b, quotient, remainder big.Int
	a.Abs(num)
	b.Set(den)
	exp := a.BitLen() - b.BitLen()
	if shift := 54 - exp; shift > 0 {
		a.Lsh(&a, uint(shift))
	} else if shift < 0 {
		b.Lsh(&b, uint(-shift))
	}
	quotient.QuoRem(&a, &b, &remainder)
	mantissa := quotient.Uint64()
	haveRem := remainder.Sign() != 0
	if mantissa>>54 == 1 {
		if mantissa&1 != 0 {
			haveRem = true
		}
		mantissa >>= 1
		exp++
	}
	if -1074 <= exp && exp <= -1022 {
		shift := uint(-1022 - (exp - 1))
		haveRem = haveRem || mantissa&(1<<shift-1) != 0
		mantissa >>= shift
		exp = -1021
	}
	if mantissa&1 != 0 && (haveRem || mantissa&2 != 0) {
		mantissa++
		if mantissa >= 1<<54 {
			mantissa >>= 1
			exp++
		}
	}
	mantissa >>= 1
	value := math.Ldexp(float64(mantissa), exp-53)
	if num.Sign() < 0 {
		return -value
	}
	return value
}

// randomBits returns a random integer of exactly n bits.
func randomBits(rng *rand.Rand, n int) *big.Int {
	out := new(big.Int)
	for out.BitLen() < n {
		out.Lsh(out, 64).Or(out, new(big.Int).SetUint64(rng.Uint64()))
	}
	out.Rsh(out, uint(out.BitLen()-n))
	return out.SetBit(out, n-1, 1)
}

func bigPow2(k int) *big.Int { return new(big.Int).Lsh(big.NewInt(1), uint(k)) }

// tieNumerators returns numerators n so that n / 2^j straddles a rounding tie
// of the 53-bit significand: an odd 54-bit value shifted by s (the exact
// halfway point), and that value one unit above and below, and with one far
// low bit set (which breaks the tie upward).
func tieNumerators(rng *rand.Rand) []*big.Int {
	odd := new(big.Int).SetUint64(rng.Uint64()&(1<<54-1) | 1<<53 | 1)
	s := uint(rng.IntN(1300))
	base := new(big.Int).Lsh(odd, s)
	out := []*big.Int{
		base,
		new(big.Int).Add(base, big.NewInt(1)),
		new(big.Int).Sub(base, big.NewInt(1)),
		new(big.Int).Add(base, bigPow2(int(s)/2)),
	}
	if s > 0 {
		out = append(out, new(big.Int).Add(base, new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(int(s))))))
	}
	return out
}

// ratFloat64Cases lists the rationals the differential tests feed both
// spellings: the float edges with their neighbours and midpoints, ties, huge
// and tiny numerators and denominators, powers of two, and fractions that are
// not reduced when built through big.Int.
func ratFloat64Cases(rng *rand.Rand, random int) []*big.Rat {
	var out []*big.Rat
	add := func(q *big.Rat) {
		out = append(out, q, new(big.Rat).Neg(q))
	}
	around := func(f float64) {
		fr := proof.FloatRat(f)
		if fr == nil {
			return
		}
		add(fr)
		for _, dir := range []float64{math.Inf(-1), math.Inf(1)} {
			n := math.Nextafter(f, dir)
			nr := proof.FloatRat(n)
			if nr == nil {
				continue
			}
			add(nr)
			mid := new(big.Rat).Quo(new(big.Rat).Add(fr, nr), big.NewRat(2, 1))
			add(mid)
			gap := new(big.Rat).Sub(nr, fr)
			gap.Abs(gap)
			for _, d := range []int64{1 << 20, 1 << 40} {
				sliver := new(big.Rat).Quo(gap, new(big.Rat).SetInt64(d))
				add(new(big.Rat).Add(mid, sliver))
				add(new(big.Rat).Sub(mid, sliver))
				add(new(big.Rat).Add(fr, sliver))
				add(new(big.Rat).Sub(fr, sliver))
			}
		}
	}
	for _, f := range []float64{
		0, 1, 3, 0.1, 1 << 53, 1<<53 + 2, math.SmallestNonzeroFloat64, 2 * math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52), math.Float64frombits(1<<52 + 1),
		math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0), math.Pi,
	} {
		around(f)
	}
	for e := -1076; e <= 1025; e++ {
		around(math.Ldexp(1, e))
		around(math.Ldexp(1.5, e))
	}
	for range random {
		around(finiteBits(rng))
	}
	// The overflow boundary and the 2^-1075 flush boundary as exact rationals.
	maxPlusHalf := new(big.Int).Sub(bigPow2(1024), bigPow2(970))
	for _, delta := range []int64{-1, 0, 1} {
		n := new(big.Int).Add(maxPlusHalf, big.NewInt(delta))
		add(new(big.Rat).SetInt(n))
		add(new(big.Rat).SetFrac(new(big.Int).Lsh(n, 1075), bigPow2(1075)))
		for _, k := range []int{1074, 1075, 1076} {
			add(new(big.Rat).SetFrac(big.NewInt(1+delta), bigPow2(k)))
			add(new(big.Rat).SetFrac(big.NewInt(3+delta), bigPow2(k)))
		}
	}
	// Exactly representable past the float range: Float64 answers an infinity
	// and reports it inexact even though no remainder exists.
	for _, k := range []int{1024, 1025, 1030, 1100, 2000} {
		add(new(big.Rat).SetInt(bigPow2(k)))
		add(new(big.Rat).SetInt(new(big.Int).Mul(big.NewInt(3), bigPow2(k))))
		add(new(big.Rat).SetFrac(bigPow2(k+50), bigPow2(50)))
	}
	for range random / 4 {
		for _, n := range tieNumerators(rng) {
			add(new(big.Rat).SetFrac(n, bigPow2(rng.IntN(1500))))
			add(new(big.Rat).SetFrac(n, new(big.Int).SetUint64(rng.Uint64()|1)))
		}
		num := new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()), uint(rng.IntN(1500)))
		num.Add(num, big.NewInt(rng.Int64N(7)))
		add(new(big.Rat).SetFrac(num, bigPow2(rng.IntN(1500))))
		add(new(big.Rat).SetInt(num))
		add(new(big.Rat).SetFrac(num, new(big.Int).Lsh(new(big.Int).SetUint64(rng.Uint64()|1), uint(rng.IntN(1500)))))
		add(new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<40)+1), new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(1500)))))
	}
	return out
}

func TestRatFloat64MatchesFloat64(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(401, 403))
	check := func(q *big.Rat) {
		t.Helper()
		numBefore := new(big.Int).Set(q.Num())
		denBefore := new(big.Int).Set(q.Denom())
		numPtr, signBefore := q.Num(), q.Sign()
		wantF, wantExact := q.Float64()
		gotF, gotExact := proof.RatFloat64(q)
		require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF), "q=%v want %v got %v", q, wantF, gotF)
		require.Equal(t, wantExact, gotExact, "flag q=%v", q)
		require.Zero(t, numBefore.Cmp(q.Num()), "numerator written")
		require.Zero(t, denBefore.Cmp(q.Denom()), "denominator written")
		require.Same(t, numPtr, q.Num())
		require.Equal(t, signBefore, q.Sign())
	}
	cases := ratFloat64Cases(rng, ratFloat64Randoms())
	for _, q := range cases {
		check(q)
		// The same value through FloatRat's unstored-denominator form.
		if f, ok := q.Float64(); ok && f != 0 {
			check(proof.FloatRat(f))
		}
	}
	// A positive subnormal quotient that flushes to zero keeps Float64's flag.
	flushed := new(big.Rat).SetFrac(big.NewInt(1), bigPow2(1080))
	f, exact := proof.RatFloat64(flushed)
	require.Zero(t, f)
	require.True(t, exact)
	// A zero-value Rat has an unstored denominator and a zero numerator.
	check(new(big.Rat))
	check(proof.FloatRat(1))
	check(proof.FloatRat(math.Ldexp(1, 900)))
	check(proof.FloatRat(-math.Ldexp(3, -1000)))
	t.Logf("%d rationals", 2*len(cases))
}

func TestRatFloat64NilPanicsLikeFloat64(t *testing.T) {
	t.Parallel()
	run := func(fn func()) (v any) {
		defer func() { v = recover() }()
		fn()
		return nil
	}
	want := run(func() { _, _ = (*big.Rat)(nil).Float64() })
	require.NotNil(t, want)
	got := run(func() { _, _ = proof.RatFloat64(nil) })
	require.Equal(t, fmt.Sprint(want), fmt.Sprint(got))
}

func TestXhpVecMatchesLegacyQuotient(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(411, 413))
	check := func(num, den *big.Int) {
		t.Helper()
		want := legacyXhpFractionFloat64(num, den)
		got := proof.XhpVec(proof.Xhp{X: num, Y: new(big.Int).Neg(num), Z: new(big.Int), W: den})
		require.Equal(t, math.Float64bits(want), math.Float64bits(got.X), "num=%v den=%v", num, den)
		wantNeg := legacyXhpFractionFloat64(new(big.Int).Neg(num), den)
		require.Equal(t, math.Float64bits(wantNeg), math.Float64bits(got.Y), "neg num=%v den=%v", num, den)
		require.Zero(t, math.Float64bits(got.Z))
	}
	for _, q := range ratFloat64Cases(rng, ratFloat64Randoms()/2) {
		n, d := q.Num(), q.Denom()
		check(n, d)
		// Not reduced: scale both by a power of two and by an odd factor.
		c := bigPow2(rng.IntN(70))
		c.Mul(c, new(big.Int).SetUint64(rng.Uint64()|1))
		check(new(big.Int).Mul(n, c), new(big.Int).Mul(d, c))
		check(new(big.Int).Lsh(n, uint(rng.IntN(300))), new(big.Int).Lsh(d, uint(rng.IntN(300))))
	}
	// A power-of-two denominator of every size against numerators of every
	// width from 1 to 2000 bits.
	for k := 0; k <= 1200; k += 1 + rng.IntN(5) {
		for _, bitsN := range []int{1, 2, 52, 53, 54, 55, 56, 64, 65, 127, 128, 129, 1000, 2000} {
			n := randomBits(rng, bitsN)
			check(n, bigPow2(k))
			check(new(big.Int).Lsh(n, 3), bigPow2(k))
		}
	}
}

func TestXhpVecNonPositiveDenominatorMatchesLegacy(t *testing.T) {
	t.Parallel()
	run := func(fn func() float64) (v float64, p any) {
		defer func() { p = recover() }()
		v = fn()
		return v, nil
	}
	nums := []*big.Int{big.NewInt(0), big.NewInt(1), big.NewInt(-7), bigPow2(200), new(big.Int).Neg(bigPow2(70))}
	dens := []*big.Int{big.NewInt(0), big.NewInt(-1), big.NewInt(-8), new(big.Int).Neg(bigPow2(100)), big.NewInt(-3)}
	for _, n := range nums {
		for _, d := range dens {
			wantV, wantP := run(func() float64 { return legacyXhpFractionFloat64(n, d) })
			gotV, gotP := run(func() float64 {
				return proof.XhpVec(proof.Xhp{X: n, Y: new(big.Int), Z: new(big.Int), W: d}).X
			})
			require.Equal(t, fmt.Sprint(wantP), fmt.Sprint(gotP), "n=%v d=%v", n, d)
			require.Equal(t, math.Float64bits(wantV), math.Float64bits(gotV), "n=%v d=%v", n, d)
		}
	}
}

// legacyRationalFloatError is RationalFloatError as it was written over
// big.Rat.Float64.
func legacyRationalFloatError(exact *big.Rat, held float64) float64 {
	heldRat := proof.FloatRat(held)
	if exact == nil || heldRat == nil {
		return math.Inf(1)
	}
	d := proof.SubRat(new(big.Rat), exact, heldRat)
	d.Abs(d)
	out, exactFloat := d.Float64()
	if d.Sign() > 0 && out == 0 {
		return math.SmallestNonzeroFloat64
	}
	if !exactFloat {
		out = proof.NextUp(out)
	}
	return out
}

func TestRationalFloatErrorMatchesLegacyFloat64(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(421, 423))
	check := func(exact *big.Rat, held float64) {
		t.Helper()
		before := new(big.Rat).Set(exact)
		want := legacyRationalFloatError(exact, held)
		got := proof.RationalFloatError(exact, held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "exact=%v held=%v", before, held)
		require.Zero(t, before.Cmp(exact))
	}
	helds := []float64{0, math.Copysign(0, -1), 1, -1, 0.1, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.MaxFloat64, -math.MaxFloat64, 0x1p-1022, 0x1p-1000, 0x1p500}
	for range ratFloat64Randoms() / 10 {
		helds = append(helds, finiteBits(rng))
	}
	for _, held := range helds {
		hr := proof.FloatRat(held)
		for _, q := range []*big.Rat{
			new(big.Rat), hr, new(big.Rat).Neg(hr),
			new(big.Rat).Add(hr, new(big.Rat).SetFrac(big.NewInt(1), bigPow2(1074))),
			new(big.Rat).Add(hr, new(big.Rat).SetFrac(big.NewInt(1), bigPow2(1075))),
			new(big.Rat).Add(hr, new(big.Rat).SetFrac(big.NewInt(1), bigPow2(1076))),
			new(big.Rat).Add(hr, new(big.Rat).SetFrac(big.NewInt(3), bigPow2(1076))),
			new(big.Rat).Add(hr, new(big.Rat).SetFrac(big.NewInt(1), bigPow2(1100))),
			new(big.Rat).Add(hr, new(big.Rat).SetInt(bigPow2(1024))),
			new(big.Rat).Sub(hr, new(big.Rat).SetInt(bigPow2(1030))),
			new(big.Rat).Add(hr, new(big.Rat).SetInt(new(big.Int).Sub(bigPow2(1024), bigPow2(970)))),
			new(big.Rat).Add(hr, big.NewRat(1, 3)),
			new(big.Rat).Mul(hr, big.NewRat(1000001, 1000000)),
		} {
			check(q, held)
		}
	}
	for _, q := range ratFloat64Cases(rng, ratFloat64Randoms()/10) {
		check(q, finiteBits(rng))
		check(q, 0)
	}
	require.True(t, math.IsInf(proof.RationalFloatError(nil, 1), 1))
	require.True(t, math.IsInf(proof.RationalFloatError(big.NewRat(1, 3), math.Inf(1)), 1))
}
