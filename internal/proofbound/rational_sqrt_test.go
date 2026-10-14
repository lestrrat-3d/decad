package proofbound

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// oldRatSquareAtMost is the body RatSquareAtMost had before the integer
// comparison: square the float as a big.Rat and compare with q.
func oldRatSquareAtMost(f float64, q *big.Rat) bool {
	square := proofarith.FloatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) <= 0
}

// oldRatSquareEquals is the body RatSquareEquals had before the integer
// comparison.
func oldRatSquareEquals(f float64, q *big.Rat) bool {
	square := proofarith.FloatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) == 0
}

// oldRatSqrtUp is RatSqrtUp with the two big.Rat squarings per step.
func oldRatSqrtUp(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := RatSqrtSeed(q)
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if !oldRatSquareAtMost(f, q) || oldRatSquareEquals(f, q) {
			return f
		}
		f = proofarith.NextUp(f)
	}
	return math.Inf(1)
}

// oldRatSqrtDown is RatSqrtDown with the big.Rat squaring per step.
func oldRatSqrtDown(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := RatSqrtSeed(q)
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if oldRatSquareAtMost(f, q) {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

// squareCmpFloats returns floats that cover the edges of ratSquareCmp: both
// zeros, subnormals, the smallest normal, the largest finite float, the
// non-finite values, powers of two, and random exponents across the range.
func squareCmpFloats(rng *rand.Rand, random int) []float64 {
	floats := []float64{
		0, math.Copysign(0, -1), 1, -1, 2, 0.5, 3, 4, 0.25, 1e300, 1e-300,
		math.SmallestNonzeroFloat64, 4 * math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), 0x1p-1022, 0x1p-1023, math.Nextafter(1, 2), math.Nextafter(1, 0),
		math.MaxFloat64, -math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0),
		math.Inf(1), math.Inf(-1), math.NaN(),
	}
	for range random {
		floats = append(floats, math.Ldexp(rng.Float64()*2-1, rng.IntN(2200)-1100))
	}
	return floats
}

// squareCmpRats returns the inputs paired with f: the exact square (equality),
// a hair above and below it, a rational perfect square, a small fraction, a
// dyadic, a wide fraction, an integer, a denominator of one, zero and negatives.
func squareCmpRats(rng *rand.Rand, f float64, i int) []*big.Rat {
	var out []*big.Rat
	if sq := proofarith.FloatRat(f); sq != nil {
		sq.Mul(sq, sq)
		out = append(out, sq)
		// One part in 2^200 either side of the square, and the smallest rational step.
		eps := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 200))
		delta := new(big.Rat).Mul(sq, eps)
		if delta.Sign() == 0 {
			delta = eps
		}
		out = append(out, new(big.Rat).Add(sq, delta), new(big.Rat).Sub(sq, delta),
			new(big.Rat).Neg(sq))
	}
	a, b := rng.Int64N(1<<30)+1, rng.Int64N(1<<30)+1
	ratio := big.NewRat(a, b)
	out = append(out, new(big.Rat).Mul(ratio, ratio)) // perfect square of a rational
	switch i % 4 {
	case 0:
		out = append(out, big.NewRat(rng.Int64N(1<<40)+1, rng.Int64N(1<<40)+1))
	case 1:
		if g := proofarith.FloatRat(math.Ldexp(rng.Float64(), rng.IntN(2200)-1100)); g != nil {
			out = append(out, g)
		}
	case 2:
		num := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<50)+1), uint(rng.IntN(300)))
		den := new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<30)+1), uint(rng.IntN(300)))
		out = append(out, new(big.Rat).SetFrac(num, den))
	default:
		out = append(out, new(big.Rat).SetInt64(rng.Int64N(1<<40)), big.NewRat(-a, b))
	}
	return out
}

func TestRatSquareCmpMatchesBigRatSquare(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	cases := 0
	for i, f := range squareCmpFloats(rng, 4000) {
		for _, q := range squareCmpRats(rng, f, i) {
			cases++
			require.Equal(t, oldRatSquareAtMost(f, q), RatSquareAtMost(f, q), "atmost %v %v", f, q)
			require.Equal(t, oldRatSquareEquals(f, q), RatSquareEquals(f, q), "equals %v %v", f, q)
			if sq := proofarith.FloatRat(f); sq != nil {
				sq.Mul(sq, sq)
				c, ok := ratSquareCmp(f, q)
				require.True(t, ok)
				require.Equal(t, sq.Cmp(q), c, "cmp %v %v", f, q)
			} else {
				_, ok := ratSquareCmp(f, q)
				require.False(t, ok, "non-finite %v", f)
			}
		}
	}
	require.Greater(t, cases, 20000)
}

func TestRatSqrtUpMatchesOldWalk(t *testing.T) {
	rng := rand.New(rand.NewPCG(13, 17))
	qs := []*big.Rat{
		new(big.Rat), big.NewRat(-1, 1), big.NewRat(-9, 4), big.NewRat(1, 1), big.NewRat(2, 1),
		big.NewRat(4, 1), big.NewRat(9, 4), big.NewRat(1, 4), big.NewRat(1, 3), big.NewRat(10, 1),
		new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2100)),
		new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 2200)),
		new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 2048)),
		new(big.Rat).SetInt(new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 2048), big.NewInt(1))),
	}
	for i, f := range squareCmpFloats(rng, 4000) {
		qs = append(qs, squareCmpRats(rng, f, i)...)
	}
	for _, q := range qs {
		want, got := oldRatSqrtUp(q), RatSqrtUp(q)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "up %v", q)
		require.Equal(t, math.Float64bits(oldRatSqrtDown(q)), math.Float64bits(RatSqrtDown(q)), "down %v", q)
		if q.Sign() <= 0 {
			require.Zero(t, got, "non-positive %v", q)
			continue
		}
		if math.IsInf(got, 1) {
			continue // sqrt(q) exceeds MaxFloat64; nothing finite bounds it
		}
		// Upper bound, checked with the big.Rat square, independent of ratSquareCmp.
		sq := proofarith.FloatRat(got)
		sq.Mul(sq, sq)
		require.GreaterOrEqual(t, sq.Cmp(q), 0, "square of %v below %v", got, q)
		// Tight where the walk moved: the previous float's square is below q. When the
		// seed is already an upper bound the walk does not move and the seed stands.
		if seed := RatSqrtSeed(q); !IsNonFinite(seed) && got > seed {
			prev := proofarith.FloatRat(proofarith.NextDown(got))
			prev.Mul(prev, prev)
			require.Less(t, prev.Cmp(q), 0, "previous candidate of %v is not below %v", got, q)
		}
	}
}
