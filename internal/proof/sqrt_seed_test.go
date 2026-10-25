package proof_test

import (
	"math"
	"math/big"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func fullDivisionSqrtSeed(n, d *big.Int, denomExp int) float64 {
	mant := new(big.Float).SetPrec(64)
	ratio := new(big.Float).SetPrec(64).Quo(new(big.Float).SetInt(n), new(big.Float).SetInt(d))
	exp := ratio.MantExp(mant) - denomExp
	if exp%2 != 0 {
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

func TestRatioSqrtSeedMatchesFullDivision(t *testing.T) {
	rng := rand.New(rand.NewSource(0x942ef5))
	accepted := 0
	for sample := range 6000 {
		nWidth, dWidth := 1+rng.Intn(4096), 1+rng.Intn(4096)
		n := new(big.Int).Rand(rng, new(big.Int).Lsh(big.NewInt(1), uint(nWidth)))
		d := new(big.Int).Rand(rng, new(big.Int).Lsh(big.NewInt(1), uint(dWidth)))
		n.SetBit(n, nWidth-1, 1)
		d.SetBit(d, dWidth-1, 1)
		nBefore, dBefore := new(big.Int).Set(n), new(big.Int).Set(d)
		for _, shift := range []int{rng.Intn(8193) - 4096, nWidth - dWidth + rng.Intn(8) - 4} {
			got, ok := proof.RatioSqrtSeed(n, d, shift)
			require.Zero(t, n.Cmp(nBefore))
			require.Zero(t, d.Cmp(dBefore))
			if !ok {
				continue
			}
			accepted++
			want := fullDivisionSqrtSeed(n, d, shift)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got), "sample %d, shift %d", sample, shift)
		}
	}
	require.Greater(t, accepted, 11000)
}

func TestRatioSqrtSeedWordAndRoundingEdges(t *testing.T) {
	for _, width := range []uint{1, 31, 32, 33, 53, 63, 64, 65, 95, 96, 127, 128, 129, 256, 513, 2048} {
		power := new(big.Int).Lsh(big.NewInt(1), width)
		for _, offset := range []int64{-1, 0, 1} {
			n := new(big.Int).Add(power, big.NewInt(offset))
			for _, d := range []*big.Int{big.NewInt(1), big.NewInt(3), power,
				new(big.Int).Add(power, big.NewInt(1)), new(big.Int).Sub(power, big.NewInt(1))} {
				for _, shift := range []int{-4096, -2149, -2148, -1074, -1, 0, 1, 1023, 2048, 4096} {
					got, ok := proof.RatioSqrtSeed(n, d, shift)
					if !ok {
						continue
					}
					want := fullDivisionSqrtSeed(n, d, shift)
					require.Equal(t, math.Float64bits(want), math.Float64bits(got),
						"width %d, offset %d, shift %d", width, offset, shift)
				}
			}
		}
	}
	// At a binary64 midpoint the eight-unit interval spans two rounded
	// mantissas, so the caller must retain the full-division fallback.
	for _, delta := range []int64{-7, -1, 0, 1, 7} {
		n := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 63), big.NewInt(1024+delta))
		_, ok := proof.RatioSqrtSeed(n, new(big.Int).Lsh(big.NewInt(1), 63), 0)
		require.False(t, ok, "midpoint offset %d", delta)
	}
	for _, n := range []*big.Int{new(big.Int), big.NewInt(-1)} {
		_, ok := proof.RatioSqrtSeed(n, big.NewInt(1), 0)
		require.False(t, ok)
	}
}

func TestRatioSqrtSeedDoesNotAllocate(t *testing.T) {
	n := new(big.Int).Lsh(big.NewInt(3), 2048)
	d := new(big.Int).Lsh(big.NewInt(7), 1024)
	got, ok := proof.RatioSqrtSeed(n, d, 512)
	require.True(t, ok)
	require.Positive(t, got)
	require.Zero(t, testing.AllocsPerRun(100, func() {
		_, _ = proof.RatioSqrtSeed(n, d, 512)
	}))
}
