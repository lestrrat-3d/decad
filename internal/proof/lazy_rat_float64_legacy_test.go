package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// legacyLazyFloat64 is LazyRat.Float64 as it was computed with big.Float.
func legacyLazyFloat64(x proofarith.LazyRat) (float64, bool, bool) {
	n, d := x.Parts()
	if n.Sign() == 0 {
		return 0, true, true
	}
	var nf, df, q big.Float
	nf.SetInt(n)
	df.SetInt(d)
	q.SetPrec(53).SetMode(big.ToNearestEven).Quo(&nf, &df)
	if exp := q.MantExp(nil); exp < -1020 || exp > 1023 {
		return 0, false, false
	}
	f, _ := q.Float64()
	return f, q.Acc() == big.Exact, true
}

// lazyFracOf builds n/d (d > 0) without cancelling odd common factors.
func lazyFracOf(n, d *big.Int) proofarith.LazyRat {
	return proofarith.LazyQuo(
		proofarith.LazyOf(new(big.Rat).SetInt(n)),
		proofarith.LazyOf(new(big.Rat).SetInt(d)))
}

func requireLazyFloat64Legacy(t *testing.T, x proofarith.LazyRat) {
	f0, e0, ok0 := legacyLazyFloat64(x)
	f1, e1, ok1 := x.Float64()
	if math.Float64bits(f0) == math.Float64bits(f1) && e0 == e1 && ok0 == ok1 {
		return
	}
	n, d := x.Parts()
	t.Helper()
	require.Failf(t, "Float64 differs from legacy", "n=%s d=%s legacy %v %v %v new %v %v %v",
		n, d, f0, e0, ok0, f1, e1, ok1)
}

// TestLazyRatFloat64MatchesBigFloatLegacy compares Float64 with the big.Float
// implementation over 60000 seeded random fractions plus every exponent from
// 2^-1080 to 2^1030 around the range edges, with halfway and non-dyadic cases.
func TestLazyRatFloat64MatchesBigFloatLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 5))
	for range 60000 {
		nb, db := 1+rng.IntN(300), 1+rng.IntN(300)
		if rng.IntN(8) == 0 {
			nb, db = 1+rng.IntN(1200), 1+rng.IntN(1200)
		}
		n := lazyRandInt(rng, nb)
		n.Abs(n).Add(n, big.NewInt(1))
		d := lazyRandInt(rng, db)
		d.Abs(d).Add(d, big.NewInt(1))
		switch rng.IntN(4) {
		case 0:
			d.SetBit(new(big.Int), rng.IntN(1200), 1)
		case 1:
			n.Lsh(n, uint(rng.IntN(100)))
		}
		if rng.IntN(2) == 0 {
			n.Neg(n)
		}
		requireLazyFloat64Legacy(t, lazyFracOf(n, d))
	}
	requireLazyFloat64Legacy(t, lazyFracOf(new(big.Int), big.NewInt(7)))
	three, seven := big.NewInt(3), big.NewInt(7)
	for e := -1080; e <= 1030; e++ {
		for _, m := range []int64{1, 3, 1<<53 - 1, 1<<53 + 1, 1<<54 - 1, 1<<54 + 1, 1 << 52} {
			for _, s := range []int64{1, -1} {
				n := big.NewInt(m * s)
				d := big.NewInt(1)
				if e >= 0 {
					n.Lsh(n, uint(e))
				} else {
					d.Lsh(d, uint(-e))
				}
				requireLazyFloat64Legacy(t, lazyFracOf(n, d))
				requireLazyFloat64Legacy(t, lazyFracOf(new(big.Int).Mul(n, three), new(big.Int).Mul(d, three)))
				plusOne := new(big.Int).Add(new(big.Int).Mul(n, seven), big.NewInt(1))
				requireLazyFloat64Legacy(t, lazyFracOf(plusOne, new(big.Int).Mul(d, seven)))
			}
		}
	}
}

// lazyBitsInt returns a random positive integer of exactly bits bits.
func lazyBitsInt(rng *rand.Rand, bits int) *big.Int {
	out := new(big.Int)
	for i := range bits - 1 {
		out.SetBit(out, i, uint(rng.IntN(2)))
	}
	return out.SetBit(out, bits-1, 1)
}

// BenchmarkLazyRatFloat64 covers the operand shapes the cross-bore proof
// passes: numerator and denominator of similar width (the numerator is
// scaled up), a power-of-two denominator, and a short wide-numerator case.
func BenchmarkLazyRatFloat64(b *testing.B) {
	rng := rand.New(rand.NewPCG(437, 439))
	odd := func(x *big.Int) *big.Int { return x.SetBit(x, 0, 1) }
	cases := []struct {
		name string
		x    proofarith.LazyRat
	}{
		{"num200_den130", lazyFracOf(new(big.Int).Neg(lazyBitsInt(rng, 200)), odd(lazyBitsInt(rng, 130)))},
		{"num570_den610", lazyFracOf(lazyBitsInt(rng, 570), odd(lazyBitsInt(rng, 610)))},
		{"num5700_den5730", lazyFracOf(new(big.Int).Neg(lazyBitsInt(rng, 5700)), odd(lazyBitsInt(rng, 5730)))},
		{"num570_pow2den", lazyFracOf(odd(lazyBitsInt(rng, 570)), new(big.Int).Lsh(big.NewInt(1), 600))},
	}
	for _, c := range cases {
		b.Run(c.name+"/current", func(b *testing.B) {
			b.ReportAllocs()
			for range b.N {
				_, _, _ = c.x.Float64()
			}
		})
		b.Run(c.name+"/legacy", func(b *testing.B) {
			b.ReportAllocs()
			for range b.N {
				_, _, _ = legacyLazyFloat64(c.x)
			}
		})
	}
}
