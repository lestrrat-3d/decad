package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestDyadicFastPathsRandomizedMatchesBigRat exercises arbitrary finite
// float64 bit patterns, including subnormals and values near the largest
// finite exponent. The reference deliberately stays in math/big so the test
// does not repeat the proof.Dyadic implementation's alignment logic.
func TestDyadicFastPathsRandomizedMatchesBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x243F6A8885A308D3, 0x13198A2E03707344))
	randomFinite := func() float64 {
		for {
			bits := rng.Uint64() &^ (uint64(0x7ff) << 52)
			bits |= (rng.Uint64() & 0x7fe) << 52
			f := math.Float64frombits(bits)
			if math.IsNaN(f) || math.IsInf(f, 0) {
				continue
			}
			return f
		}
	}
	for range 1000 {
		a, b := randomFinite(), randomFinite()
		da, db := proof.MustDyOf(a), proof.MustDyOf(b)
		ra, rb := proof.FloatRat(a), proof.FloatRat(b)
		require.Zero(t, independentDyadicRat(proof.DyAdd(da, db)).Cmp(new(big.Rat).Add(ra, rb)), "%v + %v", a, b)
		require.Zero(t, independentDyadicRat(proof.DySubScalar(da, db)).Cmp(new(big.Rat).Sub(ra, rb)), "%v - %v", a, b)
		require.Equal(t, ra.Cmp(rb), proof.DyCmp(da, db), "cmp(%v, %v)", a, b)
	}

	// These operands force the widest exponent gap representable by finite
	// float64 values while keeping the reference rational practical to build.
	large := proof.DyShift(proof.MustDyOf(1), 1023)
	small := proof.DyShift(proof.MustDyOf(1), -1074)
	largeRat, smallRat := independentDyadicRat(large), independentDyadicRat(small)
	require.Equal(t, largeRat.Cmp(smallRat), proof.DyCmp(large, small))
	require.Zero(t, independentDyadicRat(proof.DyAdd(large, small)).Cmp(new(big.Rat).Add(largeRat, smallRat)))
	require.Zero(t, independentDyadicRat(proof.DySubScalar(large, small)).Cmp(new(big.Rat).Sub(largeRat, smallRat)))
}

// TestDyadicCompareAcrossExponents compares every pair of a grid of values
// whose mantissas and exponents differ, so their leading bits fall at the same
// binary position as often as at different ones, against big.Rat.
func TestDyadicCompareAcrossExponents(t *testing.T) {
	t.Parallel()
	var values []proof.Dyadic
	for _, mant := range []int64{1, 3, 5, 7, 11, 13, 255, 257, -1, -3, -7, -11, -255} {
		for exp := -4; exp <= 4; exp++ {
			values = append(values, proof.DyShift(proof.DyInt(mant), exp))
		}
	}
	values = append(values, proof.DyZero())
	for _, a := range values {
		for _, b := range values {
			want := independentDyadicRat(a).Cmp(independentDyadicRat(b))
			require.Equal(t, want, proof.DyCmp(a, b), "cmp(%v, %v)", independentDyadicRat(a), independentDyadicRat(b))
		}
	}
}

// TestDyadicFastPathsPreserveInputs ensures each result owns the only mutable
// big.Int touched by the operation.
func TestDyadicFastPathsPreserveInputs(t *testing.T) {
	t.Parallel()
	values := []proof.Dyadic{
		proof.MustDyOf(-math.MaxFloat64),
		proof.MustDyOf(-0.1),
		proof.DyZero(),
		proof.MustDyOf(math.SmallestNonzeroFloat64),
		proof.MustDyOf(math.MaxFloat64),
	}
	for _, a := range values {
		for _, b := range values {
			aExp, bExp := a.Exp(), b.Exp()
			am, bm := new(big.Int), new(big.Int)
			if a.Mant() != nil {
				am.Set(a.Mant())
			}
			if b.Mant() != nil {
				bm.Set(b.Mant())
			}
			proof.DyAdd(a, b)
			proof.DySubScalar(a, b)
			proof.DyCmp(a, b)
			if a.Mant() != nil {
				require.Equal(t, 0, a.Mant().Cmp(am))
			}
			if b.Mant() != nil {
				require.Equal(t, 0, b.Mant().Cmp(bm))
			}
			require.Equal(t, aExp, a.Exp())
			require.Equal(t, bExp, b.Exp())
		}
	}
}

func independentDyadicRat(d proof.Dyadic) *big.Rat {
	if d.Mant() == nil {
		return new(big.Rat)
	}
	out := new(big.Rat).SetInt(d.Mant())
	if d.Exp() >= 0 {
		return out.Mul(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(d.Exp()))))
	}
	return out.Quo(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(-d.Exp()))))
}
