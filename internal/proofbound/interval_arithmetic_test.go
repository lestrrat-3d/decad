package proofbound_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestIntervalQuoMatchesAllCorners(t *testing.T) {
	ends := []int64{-7, -3, 0, 2, 9}
	for i, lowA := range ends {
		for _, highA := range ends[i:] {
			a := proofbound.Interval(big.NewRat(lowA, 3), big.NewRat(highA, 3))
			for j, lowB := range ends {
				for _, highB := range ends[j:] {
					b := proofbound.Interval(big.NewRat(lowB, 5), big.NewRat(highB, 5))
					got, ok := proofbound.IntervalQuo(a, b)
					if lowB <= 0 && highB >= 0 {
						require.False(t, ok)
						continue
					}
					require.True(t, ok)
					corners := [4]*big.Rat{
						new(big.Rat).Quo(a.Lo, b.Lo),
						new(big.Rat).Quo(a.Lo, b.Hi),
						new(big.Rat).Quo(a.Hi, b.Lo),
						new(big.Rat).Quo(a.Hi, b.Hi),
					}
					lo, hi := corners[0], corners[0]
					for _, corner := range corners[1:] {
						if corner.Cmp(lo) < 0 {
							lo = corner
						}
						if corner.Cmp(hi) > 0 {
							hi = corner
						}
					}
					require.Zero(t, got.Lo.Cmp(lo), "a=%v b=%v", a, b)
					require.Zero(t, got.Hi.Cmp(hi), "a=%v b=%v", a, b)
				}
			}
		}
	}
}

// legacyIntervalQuo is IntervalQuo's corner choice with big.Rat's own Quo.
func legacyIntervalQuo(a, b proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if b.Lo.Sign() <= 0 && b.Hi.Sign() >= 0 {
		return proofbound.RatInterval{}, false
	}
	var loNum, loDen, hiNum, hiDen *big.Rat
	if b.Lo.Sign() > 0 {
		switch {
		case a.Lo.Sign() >= 0:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Hi, a.Hi, b.Lo
		case a.Hi.Sign() <= 0:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Lo, a.Hi, b.Hi
		default:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Lo, a.Hi, b.Lo
		}
	} else {
		switch {
		case a.Lo.Sign() >= 0:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Hi, a.Lo, b.Lo
		case a.Hi.Sign() <= 0:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Lo, a.Lo, b.Hi
		default:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Hi, a.Lo, b.Hi
		}
	}
	return proofbound.IntervalOwned(new(big.Rat).Quo(loNum, loDen), new(big.Rat).Quo(hiNum, hiDen)), true
}

// TestIntervalQuoMatchesBigRatQuo requires IntervalQuo's endpoints to carry
// the same numerator and denominator big.Rat's Quo gives, over every sign
// case of a and every divisor that does not straddle zero.
func TestIntervalQuoMatchesBigRatQuo(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(13, 17))
	endpoint := func() *big.Rat {
		n := randBits(rng, 1+rng.IntN(300))
		d := randBits(rng, 1+rng.IntN(100))
		d.Add(d, big.NewInt(1))
		d.Mul(d, new(big.Int).Exp(big.NewInt(3), big.NewInt(int64(rng.IntN(40))), nil))
		return new(big.Rat).SetFrac(n, d)
	}
	// pair returns an interval whose sign class is -1 (<=0), 1 (>=0) or 0 (straddles).
	pair := func(class int) proofbound.RatInterval {
		x, y := endpoint(), endpoint()
		if x.Cmp(y) > 0 {
			x, y = y, x
		}
		switch class {
		case 1:
			return proofbound.Interval(x, y)
		case -1:
			return proofbound.Interval(y.Neg(y), x.Neg(x))
		}
		return proofbound.Interval(x.Neg(x), y)
	}
	for range 3000 {
		a := pair(rng.IntN(3) - 1)
		b := pair(1)
		if rng.IntN(2) == 0 {
			b = pair(-1)
		}
		if rng.IntN(3) == 0 { // include zero endpoints on a
			a.Lo = new(big.Rat)
		}
		if b.Lo.Sign() == 0 || b.Hi.Sign() == 0 {
			continue
		}
		got, ok := proofbound.IntervalQuo(a, b)
		want, wantOK := legacyIntervalQuo(a, b)
		require.Equal(t, wantOK, ok)
		if !ok {
			continue
		}
		require.Zero(t, got.Lo.Num().Cmp(want.Lo.Num()), "lo num a=%v b=%v", a, b)
		require.Zero(t, got.Lo.Denom().Cmp(want.Lo.Denom()), "lo den a=%v b=%v", a, b)
		require.Zero(t, got.Hi.Num().Cmp(want.Hi.Num()), "hi num a=%v b=%v", a, b)
		require.Zero(t, got.Hi.Denom().Cmp(want.Hi.Denom()), "hi den a=%v b=%v", a, b)
	}
}

// randBits returns a non-negative integer of at most bits bits.
func randBits(rng *rand.Rand, bits int) *big.Int {
	n := new(big.Int)
	for done := 0; done < bits; done += 64 {
		n.Lsh(n, 64)
		n.Or(n, new(big.Int).SetUint64(rng.Uint64()))
	}
	return n.Rsh(n, uint((bits+63)/64*64-bits))
}
