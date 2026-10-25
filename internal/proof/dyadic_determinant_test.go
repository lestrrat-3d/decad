package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func rationalFloatDeterminant(a, b, c r3.Vec) *big.Rat {
	var rows [3][3]*big.Rat
	for i, v := range [3]r3.Vec{a, b, c} {
		rows[i] = [3]*big.Rat{
			new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z),
		}
	}
	sum := new(big.Rat)
	for axis := range 3 {
		j, k := (axis+1)%3, (axis+2)%3
		cross := new(big.Rat).Sub(new(big.Rat).Mul(rows[1][j], rows[2][k]),
			new(big.Rat).Mul(rows[1][k], rows[2][j]))
		sum.Add(sum, new(big.Rat).Mul(rows[0][axis], cross))
	}
	return sum
}

func checkFloatDeterminant(t *testing.T, a, b, c r3.Vec, sample int) {
	t.Helper()
	want := rationalFloatDeterminant(a, b, c)
	got := proof.FloatDeterminant(a, b, c)
	require.Zero(t, want.Cmp(got.Rat()), "sample %d: determinant differs", sample)
	exponent := 0
	if want.Sign() != 0 {
		exponent = int(want.Num().TrailingZeroBits()) - (want.Denom().BitLen() - 1)
	}
	require.Equal(t, exponent, got.Exp(), "sample %d: result is not canonical", sample)
}

func TestFloatDeterminantMatchesRationalReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(199, 211))
	for sample := range 1000 {
		span := 20
		if sample%2 == 0 {
			span = 2000
		}
		var rows [3]r3.Vec
		for i := range rows {
			rows[i] = r3.Vec{
				X: math.Ldexp(rng.Float64()-0.5, rng.IntN(span+1)-span/2),
				Y: math.Ldexp(rng.Float64()-0.5, rng.IntN(span+1)-span/2),
				Z: math.Ldexp(rng.Float64()-0.5, rng.IntN(span+1)-span/2),
			}
		}
		if sample%8 == 0 {
			rows[1] = rows[0]
		}
		checkFloatDeterminant(t, rows[0], rows[1], rows[2], sample)
		checkFloatDeterminant(t, rows[2], rows[1], rows[0], sample+1000)
	}
}

func TestFloatDeterminantWordAndFallbackBoundaries(t *testing.T) {
	t.Parallel()
	a := r3.Vec{X: 1, Y: 1}
	b := r3.Vec{X: 1, Y: math.Nextafter(1, 2), Z: 1}
	for delta := range 521 {
		c := r3.Vec{X: math.Ldexp(1, delta), Y: 1, Z: -1}
		checkFloatDeterminant(t, a, b, c, delta)
		checkFloatDeterminant(t, c, b, a, delta+521)
	}
	checkFloatDeterminant(t, r3.Vec{}, r3.Vec{}, r3.Vec{}, 1042)
	checkFloatDeterminant(t, r3.Vec{X: math.SmallestNonzeroFloat64},
		r3.Vec{Y: math.MaxFloat64}, r3.Vec{Z: -1}, 1043)
}
