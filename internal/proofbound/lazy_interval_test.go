package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func lazyBoundInt(rng *rand.Rand, maxBits int) *big.Int {
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

func lazyBoundRat(rng *rand.Rand) *big.Rat {
	switch rng.IntN(6) {
	case 0:
		return new(big.Rat)
	case 1:
		d := new(big.Int).Lsh(big.NewInt(1), uint(rng.IntN(300)))
		return new(big.Rat).SetFrac(lazyBoundInt(rng, 200), d)
	}
	d := lazyBoundInt(rng, 200)
	d.Abs(d)
	if d.Sign() == 0 {
		d.SetInt64(1)
	}
	return new(big.Rat).SetFrac(lazyBoundInt(rng, 200), d)
}

func lazyBoundUnreduced(rng *rand.Rand, r *big.Rat) proofarith.LazyRat {
	x := proofarith.LazyOf(r)
	if rng.IntN(3) == 0 {
		return x
	}
	k := lazyBoundInt(rng, 60)
	k.Abs(k).SetBit(k, 0, 1)
	kr := proofarith.LazyOf(new(big.Rat).SetInt(k))
	return proofarith.LazyMul(x, proofarith.LazyQuo(kr, kr))
}

// lazyBoundInterval returns an interval in both forms; the lazy endpoints are
// unreduced. Zero intervals, one-signed and straddling intervals all occur.
func lazyBoundInterval(rng *rand.Rand) (proofbound.RatInterval, proofarith.LazyInterval) {
	a, b := lazyBoundRat(rng), lazyBoundRat(rng)
	switch rng.IntN(8) {
	case 0:
		a, b = new(big.Rat), new(big.Rat)
	case 1:
		a.Abs(a)
		b.Abs(b)
	case 2:
		a.Abs(a).Neg(a)
		b.Abs(b).Neg(b)
	}
	if a.Cmp(b) > 0 {
		a, b = b, a
	}
	return proofbound.Interval(a, b),
		proofarith.LazyInterval{Lo: lazyBoundUnreduced(rng, a), Hi: lazyBoundUnreduced(rng, b)}
}

func requireLazyBoundEqual(t *testing.T, want proofbound.RatInterval, got proofarith.LazyInterval) {
	t.Helper()
	for _, pair := range []struct {
		want *big.Rat
		got  proofarith.LazyRat
	}{{want.Lo, got.Lo}, {want.Hi, got.Hi}} {
		r := pair.got.Rat()
		require.Zero(t, pair.want.Cmp(r))
		require.Zero(t, pair.want.Num().Cmp(r.Num()))
		require.Zero(t, pair.want.Denom().Cmp(r.Denom()))
	}
}

func TestLazyIntervalOperationsMatchProofbound(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	for range 5000 {
		ra, a := lazyBoundInterval(rng)
		rb, b := lazyBoundInterval(rng)
		requireLazyBoundEqual(t, proofbound.IntervalSquare(ra), proofarith.LazyIntervalSquare(a))
		want, wantOK := proofbound.IntervalQuo(ra, rb)
		got, gotOK := proofarith.LazyIntervalQuo(a, b)
		require.Equal(t, wantOK, gotOK)
		if wantOK {
			requireLazyBoundEqual(t, want, got)
		}
		wantSqrt, wantOK := proofbound.IntervalSqrt(ra)
		gotSqrt, gotOK := proofbound.LazyIntervalSqrt(a)
		require.Equal(t, wantOK, gotOK)
		if wantOK {
			requireLazyBoundEqual(t, wantSqrt, gotSqrt)
		}
		require.Equal(t, math.Float64bits(proofbound.RatFloatUp(rb.Hi)),
			math.Float64bits(proofbound.LazyRatFloatUp(b.Hi)))
		require.Equal(t, math.Float64bits(proofbound.RatFloatUp(proofbound.IntervalAbsUpper(ra))),
			math.Float64bits(proofbound.LazyIntervalAbsUpperUp(a)))
		width := new(big.Rat).Sub(ra.Hi, ra.Lo)
		require.Equal(t, math.Float64bits(proofbound.RatFloatUp(width)),
			math.Float64bits(proofbound.LazyIntervalWidthUp(a)))
		held := math.Ldexp(rng.Float64()-0.5, rng.IntN(400)-200)
		require.Equal(t, math.Float64bits(proofbound.IntervalFloatError(ra, held)),
			math.Float64bits(proofbound.LazyIntervalFloatError(a, held)))
		w := lazyBoundRat(rng)
		w.Abs(w)
		requireLazyBoundEqual(t, proofbound.IntervalWiden(ra, w), proofarith.LazyIntervalWiden(a, proofarith.LazyOf(w)))
	}
}

func TestLazyRatFloatUpHandlesRangeEdges(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	for e := -1100; e <= 1100; e++ {
		for _, m := range []int64{1, 3, 1<<53 + 1, 1<<53 + 3, 1<<54 - 1} {
			var r *big.Rat
			if e >= 0 {
				r = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(m), uint(e)))
			} else {
				r = new(big.Rat).SetFrac(big.NewInt(m), new(big.Int).Lsh(big.NewInt(1), uint(-e)))
			}
			for _, v := range []*big.Rat{r, new(big.Rat).Quo(r, big.NewRat(3, 1)), new(big.Rat).Neg(r)} {
				if v.Sign() < 0 {
					continue
				}
				x := lazyBoundUnreduced(rng, v)
				require.Equal(t, math.Float64bits(proofbound.RatFloatUp(v)),
					math.Float64bits(proofbound.LazyRatFloatUp(x)), "value %v", v)
			}
		}
	}
}

func TestLazyRadSinCosSpanMatchesRadSinCosSpan(t *testing.T) {
	rng := rand.New(rand.NewPCG(37, 41))
	for range 40 {
		lo := big.NewRat(rng.Int64N(2000)-1000, 1+rng.Int64N(500))
		width := new(big.Rat).SetFrac(big.NewInt(rng.Int64N(50)), new(big.Int).Lsh(big.NewInt(1), uint(10+rng.IntN(40))))
		x := proofbound.Interval(lo, new(big.Rat).Add(lo, width))
		lx := proofarith.LazyInterval{Lo: lazyBoundUnreduced(rng, x.Lo), Hi: lazyBoundUnreduced(rng, x.Hi)}
		ws, wc, wok := proofbound.RadSinCosSpan(x)
		gs, gc, gok := proofbound.LazyRadSinCosSpan(lx)
		require.Equal(t, wok, gok)
		requireLazyBoundEqual(t, ws, gs)
		requireLazyBoundEqual(t, wc, gc)
	}
	// A reversed interval refuses.
	_, _, ok := proofbound.LazyRadSinCosSpan(proofarith.LazyInterval{
		Lo: proofarith.LazyOf(big.NewRat(2, 1)), Hi: proofarith.LazyOf(big.NewRat(1, 1))})
	require.False(t, ok)
	// A zero lower end takes the exact path.
	zero := proofarith.LazyOf(new(big.Rat))
	s, c, ok := proofbound.LazyRadSinCosSpan(proofarith.LazyInterval{Lo: zero, Hi: zero})
	require.True(t, ok)
	ws, wc, _ := proofbound.RadSinCosSpan(proofbound.PointInterval(new(big.Rat)))
	requireLazyBoundEqual(t, ws, s)
	requireLazyBoundEqual(t, wc, c)
}

// ratDistanceReference is the lowest-terms computation RatDistanceUp replaces.
func ratDistanceReference(a, b *big.Rat) float64 {
	return proofbound.RatFloatUp(new(big.Rat).Abs(new(big.Rat).Sub(a, b)))
}

// ratDistanceDyadic is m * 2^e.
func ratDistanceDyadic(m *big.Int, e int) *big.Rat {
	if e >= 0 {
		return new(big.Rat).SetInt(new(big.Int).Lsh(m, uint(e)))
	}
	return new(big.Rat).SetFrac(m, new(big.Int).Lsh(big.NewInt(1), uint(-e)))
}

// ratDistanceBigDen is a rational with an odd denominator of about bits bits,
// the shape of a projective cap centroid.
func ratDistanceBigDen(rng *rand.Rand, numBits, denBits int) *big.Rat {
	n := lazyBoundInt(rng, numBits)
	d := lazyBoundInt(rng, denBits)
	d.Abs(d).SetBit(d, denBits, 1).SetBit(d, 0, 1)
	return new(big.Rat).SetFrac(n, d)
}

// ratDistanceTargets are exact non-negative differences at every rounding
// edge: zero, representable values, exact halfway points, one unit of a large
// odd denominator past a float, the subnormal range and below it, both ends of
// LazyRat.Float64's range, the top binade and overflow.
func ratDistanceTargets(rng *rand.Rand) []*big.Rat {
	out := []*big.Rat{new(big.Rat)}
	odd := ratDistanceBigDen(rng, 8, 200)
	odd.Abs(odd)
	if odd.Sign() == 0 {
		odd.SetFrac64(1, 3)
	}
	for _, e := range []int{-1200, -1130, -1100, -1080, -1076, -1075, -1074, -1073, -1060, -1023,
		-1022, -1021, -1020, -1000, -600, -54, -53, -1, 0, 1, 52, 53, 600, 970, 1000, 1020, 1021, 1022, 1023} {
		for _, m := range []*big.Int{
			big.NewInt(1), big.NewInt(3),
			big.NewInt(1<<53 - 1), big.NewInt(1<<53 + 1), big.NewInt(1<<54 - 1),
			new(big.Int).SetUint64(rng.Uint64() | 1),
		} {
			x := ratDistanceDyadic(m, e-53)
			out = append(out, x)
			// A strict hair above and below x moves an exact or halfway value
			// to the side the rounding must respect.
			hair := ratDistanceDyadic(big.NewInt(1), e-200)
			hair.Mul(hair, odd)
			out = append(out, new(big.Rat).Add(x, hair))
			if lo := new(big.Rat).Sub(x, hair); lo.Sign() > 0 {
				out = append(out, lo)
			}
		}
	}
	maxF := new(big.Rat).SetFloat64(math.MaxFloat64)
	halfULP := ratDistanceDyadic(big.NewInt(1), 1023-53)
	out = append(out,
		maxF,
		new(big.Rat).Add(maxF, ratDistanceDyadic(big.NewInt(1), 900)),
		new(big.Rat).Add(maxF, halfULP),
		new(big.Rat).Sub(new(big.Rat).Add(maxF, halfULP), ratDistanceDyadic(big.NewInt(1), 900)),
		ratDistanceDyadic(big.NewInt(3), 1024),
		new(big.Rat).Quo(big.NewRat(1, 1), new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(3), 1100))),
	)
	return out
}

// requireRatUnwritten checks r still has the value, sign and lowest-terms
// parts of its snapshot.
func requireRatUnwritten(t *testing.T, before, r *big.Rat, name string) {
	t.Helper()
	require.Zero(t, before.Num().Cmp(r.Num()), "%s's numerator was written", name)
	require.Zero(t, before.Denom().Cmp(r.Denom()), "%s's denominator was written", name)
}

func requireRatDistanceMatches(t *testing.T, a, b *big.Rat) {
	t.Helper()
	aBefore, bBefore := new(big.Rat).Set(a), new(big.Rat).Set(b)
	exact := new(big.Rat).Abs(new(big.Rat).Sub(a, b))
	want := ratDistanceReference(a, b)
	got := proofbound.RatDistanceUp(a, b)
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), "|%s - %s|", aBefore, bBefore)
	requireRatUnwritten(t, aBefore, a, "a")
	requireRatUnwritten(t, bBefore, b, "b")
	if math.IsInf(got, 1) {
		return
	}
	require.GreaterOrEqual(t, proofarith.FloatRat(got).Cmp(exact), 0, "underestimates |%s - %s|", aBefore, bBefore)
	if got > 0 {
		require.Negative(t, proofarith.FloatRat(proofarith.NextDown(got)).Cmp(exact), "not the least upper float")
	}
}

func TestRatDistanceUpMatchesRatFloatUp(t *testing.T) {
	rng := rand.New(rand.NewPCG(53, 1074))
	var fast, fallback int
	countPath := func(a, b *big.Rat) {
		d := proofarith.LazySub(proofarith.LazyOf(a), proofarith.LazyOf(b))
		if _, _, ok := d.Float64(); ok {
			fast++
		} else {
			fallback++
		}
	}
	check := func(a, b *big.Rat) {
		countPath(a, b)
		requireRatDistanceMatches(t, a, b)
		requireRatDistanceMatches(t, b, a)
	}
	t.Run("equal inputs", func(t *testing.T) {
		for range 50 {
			a := ratDistanceBigDen(rng, 200, 1+rng.IntN(3000))
			check(a, a)
			check(a, new(big.Rat).Set(a))
		}
		check(new(big.Rat), new(big.Rat))
		check(&big.Rat{}, big.NewRat(0, 7))
	})
	t.Run("edge differences", func(t *testing.T) {
		for _, x := range ratDistanceTargets(rng) {
			for _, b := range []*big.Rat{
				new(big.Rat),
				big.NewRat(-5, 1),
				ratDistanceBigDen(rng, 160, 200),
				ratDistanceBigDen(rng, 1000, 1000),
			} {
				// b + x and b - x both differ from b by exactly x, so the
				// difference cancels b's large numerator and denominator.
				check(new(big.Rat).Add(b, x), b)
				check(new(big.Rat).Sub(b, x), b)
			}
		}
	})
	t.Run("fallback cutoffs", func(t *testing.T) {
		// The fast path covers [2^-1021, 2^1023): the lower cutoff is on it
		// and the upper cutoff is on the fallback.
		big3 := ratDistanceBigDen(rng, 4000, 4000)
		for _, c := range []struct {
			e        int
			wantFast bool
		}{{-1021, true}, {1023, false}} {
			x := ratDistanceDyadic(big.NewInt(1), c.e)
			d := proofarith.LazySub(proofarith.LazyOf(x), proofarith.LazyOf(new(big.Rat)))
			_, _, ok := d.Float64()
			require.Equal(t, c.wantFast, ok, "path of 2^%d", c.e)
			check(x, new(big.Rat))
			// The same difference reached through a nonzero subtraction.
			check(new(big.Rat).Add(big3, x), big3)
			check(new(big.Rat).Sub(big3, x), big3)
		}
	})
	t.Run("shared denominator", func(t *testing.T) {
		for range 300 {
			d := lazyBoundInt(rng, 4000)
			d.Abs(d).SetBit(d, 0, 1)
			a := new(big.Rat).SetFrac(lazyBoundInt(rng, 4000), d)
			b := new(big.Rat).SetFrac(lazyBoundInt(rng, 4000), d)
			check(a, b)
		}
	})
	t.Run("unequal large denominators", func(t *testing.T) {
		for range 40 {
			check(ratDistanceBigDen(rng, 160, 200), ratDistanceBigDen(rng, 15000, 15000))
		}
		for range 300 {
			check(lazyBoundRat(rng), lazyBoundRat(rng))
		}
	})
	require.Positive(t, fast, "no pair took the unreduced path")
	require.Positive(t, fallback, "no pair took the lowest-terms fallback")
}
