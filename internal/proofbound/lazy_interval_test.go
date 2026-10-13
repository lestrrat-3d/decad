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
