package freeform_test

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func TestRpIntegral01MatchesRationalAndOwnsResult(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(31))
	for sample := range 150 {
		p := make(polynomial.RatPoly, sample%49)
		before := make(polynomial.RatPoly, len(p))
		want := new(big.Rat)
		for i := range p {
			n := new(big.Int).SetUint64(rng.Uint64())
			d := big.NewInt(int64(2*(i%17) + 1))
			exp := rng.Intn(1201) - 600
			if exp < 0 {
				d.Lsh(d, uint(-exp))
			} else {
				n.Lsh(n, uint(exp))
			}
			if i%2 == 0 {
				n.Neg(n)
			}
			if i%5 == 0 {
				n.SetInt64(0)
			}
			p[i] = new(big.Rat).SetFrac(n, d)
			if i > 0 && i%7 == 0 {
				p[i] = p[i-1]
			}
			before[i] = new(big.Rat).Set(p[i])
			want.Add(want, new(big.Rat).Quo(p[i], big.NewRat(int64(i)+1, 1)))
		}
		got := freeform.RpIntegral01(p)
		require.Zero(t, want.Num().Cmp(got.Num()), "sample %d numerator", sample)
		require.Zero(t, want.Denom().Cmp(got.Denom()), "sample %d denominator", sample)
		got.SetInt64(123)
		for i := range p {
			require.Zero(t, before[i].Num().Cmp(p[i].Num()), "sample %d coefficient %d numerator", sample, i)
			require.Zero(t, before[i].Denom().Cmp(p[i].Denom()), "sample %d coefficient %d denominator", sample, i)
		}
	}
}
