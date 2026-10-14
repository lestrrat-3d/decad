package motionbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// adversarialRat draws zero, an integer, a float lift, a fraction over a
// smooth denominator, or a huge numerator over a huge denominator, of either
// sign.
func adversarialRat(rng *rand.Rand) *big.Rat {
	sign := int64(1 - 2*rng.IntN(2))
	switch rng.IntN(6) {
	case 0:
		return new(big.Rat)
	case 1:
		return new(big.Rat).SetInt64(sign * rng.Int64N(30))
	case 2:
		return proofarith.FloatRat(rng.NormFloat64())
	case 3:
		return proofarith.FloatRat(math.Ldexp(rng.NormFloat64(), rng.IntN(200)-100))
	case 4:
		return new(big.Rat).SetFrac(big.NewInt(sign*rng.Int64N(1000)), big.NewInt(3*(2*rng.Int64N(300)+1)))
	}
	num := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)+1), uint(rng.IntN(200)))
	den := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<40)*2+1), uint(rng.IntN(200)))
	return new(big.Rat).SetFrac(num.Mul(num, big.NewInt(sign)), den)
}

func adversarialInterval(rng *rand.Rand) proofbound.RatInterval {
	lo, hi := adversarialRat(rng), adversarialRat(rng)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	return proofbound.RatInterval{Lo: lo, Hi: hi}
}

func sameRat(x, y *big.Rat) bool {
	return x.Num().Cmp(y.Num()) == 0 && x.Denom().Cmp(y.Denom()) == 0
}

// TestMagnitudeSquaredUpperMatchesRawRat compares MagnitudeSquaredUpper with
// the sum of squared larger magnitudes taken through big.Rat's own Cmp, Mul
// and Add, over intervals that include zero, equal-magnitude opposite ends
// and ends of very different size.
func TestMagnitudeSquaredUpperMatchesRawRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(31, 37))
	for range 3000 {
		entries := make([]proofbound.RatInterval, rng.IntN(10))
		for i := range entries {
			entries[i] = adversarialInterval(rng)
			if rng.IntN(5) == 0 {
				entries[i].Lo = new(big.Rat).Neg(entries[i].Hi)
			}
		}
		want := new(big.Rat)
		for _, e := range entries {
			m := new(big.Rat).Abs(e.Lo)
			if hi := new(big.Rat).Abs(e.Hi); hi.Cmp(m) > 0 {
				m = hi
			}
			want.Add(want, m.Mul(m, m))
		}
		got := motionbound.MagnitudeSquaredUpper(entries...)
		require.True(t, sameRat(got, want), "got %v, want %v", got, want)
	}
}

// TestMulPointsMatchesIntervalProduct requires ScaledIvMat.MulPoints to agree
// with the interval matrix product when every entry of the right factor is a
// point, and to refuse a factor with one entry that is not.
func TestMulPointsMatchesIntervalProduct(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(41, 43))
	for range 300 {
		var left, right motionbound.IvMat
		for i := range left {
			for j := range left[i] {
				left[i][j] = adversarialInterval(rng)
				right[i][j] = proofbound.PointInterval(adversarialRat(rng))
			}
		}
		got, ok := motionbound.NewScaledIvMat(left).MulPoints(right)
		require.True(t, ok)
		for i := range 3 {
			for j := range 3 {
				want := proofbound.PointInterval(new(big.Rat))
				for k := range 3 {
					want = proofbound.IntervalAdd(want, proofbound.IntervalMul(left[i][k], right[k][j]))
				}
				entry := got.Entry(i, j)
				require.True(t, sameRat(entry.Lo, want.Lo) && sameRat(entry.Hi, want.Hi),
					"entry (%d, %d): got [%v, %v], want [%v, %v]", i, j, entry.Lo, entry.Hi, want.Lo, want.Hi)
			}
		}

		i, j := rng.IntN(3), rng.IntN(3)
		right[i][j] = proofbound.RatInterval{Lo: big.NewRat(-1, 3), Hi: big.NewRat(1, 7)}
		_, ok = motionbound.NewScaledIvMat(left).MulPoints(right)
		require.False(t, ok, "a non-point entry must be refused")
	}
}
