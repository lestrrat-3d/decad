package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestDyadicFastPathsRandomizedMatchesBigRat exercises arbitrary finite
// float64 bit patterns, including subnormals and values near the largest
// finite exponent. The reference deliberately stays in math/big so the test
// does not repeat the dyadic implementation's alignment logic.
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
		da, db := mustDyOf(a), mustDyOf(b)
		ra, rb := floatRat(a), floatRat(b)
		require.Zero(t, independentDyadicRat(dyAdd(da, db)).Cmp(new(big.Rat).Add(ra, rb)), "%v + %v", a, b)
		require.Zero(t, independentDyadicRat(dySubScalar(da, db)).Cmp(new(big.Rat).Sub(ra, rb)), "%v - %v", a, b)
		require.Equal(t, ra.Cmp(rb), dyCmp(da, db), "cmp(%v, %v)", a, b)
	}

	// These operands force the widest exponent gap representable by finite
	// float64 values while keeping the reference rational practical to build.
	large := dyShift(mustDyOf(1), 1023)
	small := dyShift(mustDyOf(1), -1074)
	largeRat, smallRat := independentDyadicRat(large), independentDyadicRat(small)
	require.Equal(t, largeRat.Cmp(smallRat), dyCmp(large, small))
	require.Zero(t, independentDyadicRat(dyAdd(large, small)).Cmp(new(big.Rat).Add(largeRat, smallRat)))
	require.Zero(t, independentDyadicRat(dySubScalar(large, small)).Cmp(new(big.Rat).Sub(largeRat, smallRat)))
}

// TestDyadicFastPathsPreserveInputs ensures each result owns the only mutable
// big.Int touched by the operation.
func TestDyadicFastPathsPreserveInputs(t *testing.T) {
	t.Parallel()
	values := []dyadic{
		mustDyOf(-math.MaxFloat64),
		mustDyOf(-0.1),
		dyZero(),
		mustDyOf(math.SmallestNonzeroFloat64),
		mustDyOf(math.MaxFloat64),
	}
	for _, a := range values {
		for _, b := range values {
			aExp, bExp := a.exp, b.exp
			am, bm := new(big.Int), new(big.Int)
			if a.mant != nil {
				am.Set(a.mant)
			}
			if b.mant != nil {
				bm.Set(b.mant)
			}
			dyAdd(a, b)
			dySubScalar(a, b)
			dyCmp(a, b)
			if a.mant != nil {
				require.Equal(t, 0, a.mant.Cmp(am))
			}
			if b.mant != nil {
				require.Equal(t, 0, b.mant.Cmp(bm))
			}
			require.Equal(t, aExp, a.exp)
			require.Equal(t, bExp, b.exp)
		}
	}
}

func independentDyadicRat(d dyadic) *big.Rat {
	if d.mant == nil {
		return new(big.Rat)
	}
	out := new(big.Rat).SetInt(d.mant)
	if d.exp >= 0 {
		return out.Mul(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(d.exp))))
	}
	return out.Quo(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(-d.exp))))
}
