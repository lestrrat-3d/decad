package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func lazyRandInt(rng *rand.Rand, maxBits int) *big.Int {
	bits := 1 + rng.IntN(maxBits)
	words := make([]byte, (bits+7)/8)
	for i := range words {
		words[i] = byte(rng.Uint32())
	}
	out := new(big.Int).SetBytes(words)
	out.Rsh(out, uint(len(words)*8-bits))
	if rng.IntN(2) == 0 {
		out.Neg(out)
	}
	return out
}

// lazyRandRat returns a rational of a random shape: zero, a power-of-two
// denominator, an integer, or a general fraction.
func lazyRandRat(rng *rand.Rand) *big.Rat {
	switch rng.IntN(8) {
	case 0:
		return new(big.Rat)
	case 1:
		d := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(700)))
		return new(big.Rat).SetFrac(lazyRandInt(rng, 600), d)
	case 2:
		return new(big.Rat).SetInt(lazyRandInt(rng, 600))
	}
	d := lazyRandInt(rng, 600)
	if d.Sign() < 0 {
		d.Neg(d)
	}
	if d.Sign() == 0 {
		d.SetInt64(1)
	}
	return new(big.Rat).SetFrac(lazyRandInt(rng, 600), d)
}

// lazyUnreduced writes r as n*k/(d*k) for a random odd k, so the value is
// unchanged and the fraction is not in lowest terms.
func lazyUnreduced(rng *rand.Rand, r *big.Rat) proofarith.LazyRat {
	x := proofarith.LazyOf(r)
	if rng.IntN(3) == 0 {
		return x
	}
	k := lazyRandInt(rng, 80)
	k.Abs(k).SetBit(k, 0, 1)
	kr := proofarith.LazyOf(new(big.Rat).SetInt(k))
	return proofarith.LazyMul(x, proofarith.LazyQuo(kr, kr))
}

// lazyEqual reports whether every check of requireLazyEqual passes.
func lazyEqual(want *big.Rat, got proofarith.LazyRat) bool {
	r := got.Rat()
	return want.Cmp(r) == 0 && want.Num().Cmp(r.Num()) == 0 && want.Denom().Cmp(r.Denom()) == 0
}

// requireLazyEqual compares with plain Go first and runs the checks below only on a
// mismatch, so a passing call skips t.Helper and require.
func requireLazyEqual(t *testing.T, want *big.Rat, got proofarith.LazyRat) {
	if lazyEqual(want, got) {
		return
	}
	t.Helper()
	r := got.Rat()
	require.Zero(t, want.Cmp(r))
	require.Zero(t, want.Num().Cmp(r.Num()))
	require.Zero(t, want.Denom().Cmp(r.Denom()))
}

func TestLazyRatOperationsMatchBigRat(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	for range 20000 {
		ra, rb := lazyRandRat(rng), lazyRandRat(rng)
		a, b := lazyUnreduced(rng, ra), lazyUnreduced(rng, rb)
		requireLazyEqual(t, ra, a)
		requireLazyEqual(t, new(big.Rat).Add(ra, rb), proofarith.LazyAdd(a, b))
		requireLazyEqual(t, new(big.Rat).Sub(ra, rb), proofarith.LazySub(a, b))
		requireLazyEqual(t, new(big.Rat).Mul(ra, rb), proofarith.LazyMul(a, b))
		requireLazyEqual(t, new(big.Rat).Neg(ra), proofarith.LazyNeg(a))
		if rb.Sign() != 0 {
			requireLazyEqual(t, new(big.Rat).Quo(ra, rb), proofarith.LazyQuo(a, b))
		}
		require.Equal(t, ra.Cmp(rb), proofarith.LazyCmp(a, b))
		require.Equal(t, ra.Sign(), a.Sign())
		n, d := a.Parts()
		require.Positive(t, d.Sign())
		require.Zero(t, ra.Cmp(new(big.Rat).SetFrac(n, d)))
	}
}

func lazyRandInterval(rng *rand.Rand) (proofarith.RatInterval, proofarith.LazyInterval) {
	a, b := lazyRandRat(rng), lazyRandRat(rng)
	if rng.IntN(6) == 0 {
		a, b = new(big.Rat), new(big.Rat)
	}
	if a.Cmp(b) > 0 {
		a, b = b, a
	}
	return proofarith.Interval(a, b), proofarith.LazyInterval{Lo: lazyUnreduced(rng, a), Hi: lazyUnreduced(rng, b)}
}

// requireLazyIntervalEqual takes the same fast path as requireLazyEqual.
func requireLazyIntervalEqual(t *testing.T, want proofarith.RatInterval, got proofarith.LazyInterval) {
	if lazyEqual(want.Lo, got.Lo) && lazyEqual(want.Hi, got.Hi) {
		return
	}
	t.Helper()
	requireLazyEqual(t, want.Lo, got.Lo)
	requireLazyEqual(t, want.Hi, got.Hi)
}

func TestLazyIntervalOperationsMatchRatIntervals(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	signs := map[[2]int]int{}
	for range 20000 {
		ra, a := lazyRandInterval(rng)
		rb, b := lazyRandInterval(rng)
		signs[[2]int{ra.Lo.Sign(), ra.Hi.Sign()}]++
		requireLazyIntervalEqual(t, proofarith.MulInterval(ra, rb), proofarith.LazyIntervalMul(a, b))
		requireLazyIntervalEqual(t, proofarith.AddInterval(ra, rb), proofarith.LazyIntervalAdd(a, b))
		requireLazyIntervalEqual(t, proofarith.SubInterval(ra, rb), proofarith.LazyIntervalSub(a, b))
		w := lazyRandRat(rng)
		w.Abs(w)
		requireLazyIntervalEqual(t,
			proofarith.OwnedInterval(new(big.Rat).Sub(ra.Lo, w), new(big.Rat).Add(ra.Hi, w)),
			proofarith.LazyIntervalWiden(a, proofarith.LazyOf(w)))
	}
	// Every sign pattern of an interval operand occurs, so MulInterval's nine
	// branches and its straddle case are all compared.
	for _, pattern := range [][2]int{{1, 1}, {-1, -1}, {-1, 1}, {0, 1}, {-1, 0}, {0, 0}} {
		require.Positive(t, signs[pattern], "sign pattern %v", pattern)
	}
}

func lazyFloatCases(rng *rand.Rand) []*big.Rat {
	var out []*big.Rat
	mants := []*big.Int{
		big.NewInt(1), big.NewInt(3),
		new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(1)), // halfway
		new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(3)), // halfway
		new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 53), big.NewInt(1)),
		new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 54), big.NewInt(1)),
	}
	for e := -1080; e <= 1100; e += 1 {
		for _, m := range mants {
			out = append(out, ratPow2(m, e), ratPow2(new(big.Int).Neg(m), e))
		}
		if e%7 == 0 {
			out = append(out, ratPow2(lazyRandInt(rng, 120), e))
		}
	}
	// Non-dyadic quotients near the range edges.
	for _, e := range []int{-1022, -1021, -1020, 1022, 1023, 1024} {
		out = append(out, new(big.Rat).Quo(ratPow2(big.NewInt(1), e), big.NewRat(3, 1)),
			new(big.Rat).Mul(ratPow2(big.NewInt(1), e), big.NewRat(7, 3)))
	}
	return out
}

func ratPow2(m *big.Int, e int) *big.Rat {
	if e >= 0 {
		return new(big.Rat).SetInt(new(big.Int).Lsh(m, uint(e)))
	}
	return new(big.Rat).SetFrac(m, new(big.Int).Lsh(big.NewInt(1), uint(-e)))
}

func TestLazyRatFloat64MatchesBigRat(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	okCount, outCount := 0, 0
	for _, r := range lazyFloatCases(rng) {
		x := lazyUnreduced(rng, r)
		f, exact, ok := x.Float64()
		ref, _ := r.Float64()
		if !ok {
			outCount++
			// Out of range only when the rounded magnitude is below 2^-1020
			// or from 2^1023 up.
			require.True(t, math.Abs(ref) < 0x1p-1020 || math.Abs(ref) >= 0x1p1023,
				"in-range value %v reported out of range", r)
			continue
		}
		okCount++
		want, wantExact := r.Float64()
		require.Equal(t, math.Float64bits(want), math.Float64bits(f), "value %v", r)
		require.Equal(t, wantExact, exact, "value %v", r)
	}
	require.Positive(t, okCount)
	require.Positive(t, outCount)
}

func TestLazyFloatErrorMatchesRationalFloatError(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	for range 20000 {
		held := math.Ldexp(rng.Float64()-0.5, rng.IntN(60)-30)
		switch rng.IntN(10) {
		case 0:
			held = 0
		case 1:
			held = math.Ldexp(rng.Float64(), -1060+rng.IntN(30))
		case 2:
			held = math.Ldexp(rng.Float64(), 1000+rng.IntN(23))
		}
		heldRat := proofarith.FloatRat(held)
		var exact *big.Rat
		switch rng.IntN(5) {
		case 0:
			exact = new(big.Rat).Set(heldRat)
		case 1:
			exact = lazyRandRat(rng)
		default:
			// A tiny perturbation of the held value.
			gap := new(big.Rat).SetFrac(lazyRandInt(rng, 200), new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(1200))))
			exact = new(big.Rat).Add(heldRat, gap)
		}
		want := proofarith.RationalFloatError(exact, held)
		got := proofarith.LazyFloatError(lazyUnreduced(rng, exact), held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "exact %v held %v", exact, held)
	}
	require.True(t, math.IsInf(proofarith.LazyFloatError(proofarith.LazyOf(new(big.Rat)), math.NaN()), 1))
}
