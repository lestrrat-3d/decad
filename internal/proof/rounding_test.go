package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// frexpFloatRat is FloatRat as it was written over math.Frexp.
func frexpFloatRat(value float64) *big.Rat {
	if value == 0 {
		return new(big.Rat)
	}
	frac, exp := math.Frexp(value)
	mant := int64(frac * (1 << 53))
	exp -= 53
	for mant&1 == 0 {
		mant >>= 1
		exp++
	}
	r := new(big.Rat).SetInt64(mant)
	if exp >= 0 {
		r.Num().Lsh(r.Num(), uint(exp))
	} else {
		r.Denom().Lsh(r.Denom(), uint(-exp))
	}
	return r
}

// bigRatFloatError is RationalFloatError as it was written over big.Rat.Sub.
func bigRatFloatError(exact *big.Rat, held float64) float64 {
	heldRat := frexpFloatRat(held)
	d := new(big.Rat).Sub(exact, heldRat)
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

func finiteBits(rng *rand.Rand) float64 {
	for {
		f := math.Float64frombits(rng.Uint64())
		if !math.IsNaN(f) && !math.IsInf(f, 0) {
			return f
		}
	}
}

func TestFloatRatMatchesFrexpBody(t *testing.T) {
	t.Parallel()
	special := []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, 3, -3, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.Float64frombits(1<<52 - 1), math.Float64frombits(1 << 52), -math.Float64frombits(1 << 52),
		math.MaxFloat64, -math.MaxFloat64, math.Nextafter(1, 2), math.Nextafter(1, 0), 1 << 60, 0.1,
	}
	check := func(f float64) {
		t.Helper()
		got, want := proof.FloatRat(f), frexpFloatRat(f)
		require.Zero(t, want.Cmp(got), "%v", f)
		require.Zero(t, want.Denom().Cmp(got.Denom()), "%v", f)
		require.Zero(t, want.Num().Cmp(got.Num()), "%v", f)
	}
	for _, f := range special {
		check(f)
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for range 100000 {
		check(finiteBits(rng))
	}
	require.Nil(t, proof.FloatRat(math.NaN()))
	require.Nil(t, proof.FloatRat(math.Inf(1)))
}

func TestRationalFloatErrorMatchesBigRatSub(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(3, 4))
	for i := range 20000 {
		held := finiteBits(rng)
		if i%3 == 0 {
			held = (rng.Float64() - 0.5) * math.Ldexp(1, rng.IntN(60)-30)
		}
		var exact *big.Rat
		switch rng.IntN(6) {
		case 0:
			exact = frexpFloatRat(held)
		case 1:
			// a near-tie: held plus a sliver
			exact = new(big.Rat).Add(frexpFloatRat(held), big.NewRat(1, rng.Int64N(1<<40)+1))
		case 2:
			// a difference that flushes below the smallest subnormal
			exact = new(big.Rat).Add(frexpFloatRat(held), new(big.Rat).SetFrac(big.NewInt(1),
				new(big.Int).Lsh(big.NewInt(3), uint(1076+rng.IntN(200)))))
		case 3:
			exact = new(big.Rat).Add(frexpFloatRat(held), frexpFloatRat(finiteBits(rng)))
		case 4:
			exact = big.NewRat(rng.Int64N(2001)-1000, rng.Int64N(999)+1)
		default:
			exact = new(big.Rat).SetFrac(big.NewInt(rng.Int64N(1<<50)-1<<49),
				new(big.Int).Lsh(big.NewInt(rng.Int64N(1<<30)|1), uint(rng.IntN(70))))
		}
		held0 := new(big.Rat).Set(exact)
		got := proof.RationalFloatError(exact, held)
		want := bigRatFloatError(exact, held)
		require.Equal(t, math.Float64bits(want), math.Float64bits(got), "exact=%v held=%v", held0, held)
		require.Zero(t, held0.Cmp(exact))
	}
	require.True(t, math.IsInf(proof.RationalFloatError(nil, 1), 1))
	require.True(t, math.IsInf(proof.RationalFloatError(big.NewRat(1, 3), math.NaN()), 1))
}
