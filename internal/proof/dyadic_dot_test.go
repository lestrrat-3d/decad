package proof_test

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func checkSquaredNorm(t *testing.T, v proof.DyV3, sample int) {
	t.Helper()
	var before [3]*big.Rat
	want := new(big.Rat)
	for axis, component := range v {
		before[axis] = component.Rat()
		want.Add(want, new(big.Rat).Mul(before[axis], before[axis]))
	}
	got := proof.DvDot(v, v)
	require.Zero(t, want.Cmp(got.Rat()), "sample %d: squared norm differs", sample)
	if want.Sign() != 0 {
		require.Equal(t, uint(0), got.Mant().TrailingZeroBits(), "sample %d: unreduced mantissa", sample)
	}
	for axis, component := range v {
		require.Zero(t, before[axis].Cmp(component.Rat()), "sample %d: input axis %d changed", sample, axis)
	}
}

func TestDyadicSquaredNormMatchesRationalReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(181, 191))
	widths := []int{0, 1, 31, 32, 63, 64, 65, 100, 126, 127, 128, 200, 400}
	for sample := range 600 {
		var v proof.DyV3
		for axis := range v {
			width := widths[(sample+axis)%len(widths)]
			mantissa := new(big.Int)
			if width != 0 {
				buf := make([]byte, (width+7)/8)
				for i := range buf {
					buf[i] = byte(rng.UintN(256))
				}
				mantissa.SetBytes(buf)
				mantissa.Rsh(mantissa, uint(len(buf)*8-width))
				mantissa.SetBit(mantissa, width-1, 1)
				mantissa.SetBit(mantissa, 0, 1)
				if rng.IntN(2) == 0 {
					mantissa.Neg(mantissa)
				}
			}
			v[axis] = dyadicFromParts(t, mantissa, rng.IntN(2401)-1200)
		}
		checkSquaredNorm(t, v, sample)
	}
}

func TestDyadicSquaredNormWordBoundaries(t *testing.T) {
	t.Parallel()
	wide := new(big.Int).Sub(new(big.Int).Lsh(big.NewInt(1), 127), big.NewInt(1))
	for delta := range 260 {
		v := proof.DyV3{
			dyadicFromParts(t, wide, -160),
			dyadicFromParts(t, new(big.Int).Neg(wide), delta-160),
			dyadicFromParts(t, wide, -160),
		}
		// This covers every alignment within a word, all 64-bit carries,
		// the 512-bit accumulator limit, and the general-path fallback.
		checkSquaredNorm(t, v, delta)
		v[1] = dyadicFromParts(t, big.NewInt(-1), delta-160)
		checkSquaredNorm(t, v, delta+260)
	}
	checkSquaredNorm(t, proof.DyV3{}, 520)
	checkSquaredNorm(t, proof.DyV3{proof.DyZero(), dyadicFromParts(t, wide, 0), proof.DyZero()}, 521)
}
