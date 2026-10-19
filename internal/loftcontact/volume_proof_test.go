package loftcontact_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestLoftVolumeRationalCeiling(t *testing.T) {
	tooLarge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), loftcontact.VolumeComposerRatBits))
	require.False(t, loftcontact.VolumeRatFits(tooLarge))
	_, rounded := loftcontact.VolumeRoundUp(tooLarge)
	require.False(t, rounded)
	require.False(t, loftcontact.VolumeSumFits(big.NewRat(1, 1), tooLarge))
	require.False(t, loftcontact.VolumeCompareFits(big.NewRat(1, 1), tooLarge))
	require.True(t, loftcontact.VolumeRatFits(big.NewRat(1, 3)))
	require.True(t, loftcontact.VolumeSumFits(big.NewRat(1, 3), big.NewRat(1, 6)))
	require.True(t, loftcontact.VolumeCompareFits(big.NewRat(1, 3), big.NewRat(1, 6)))

	heldSevenTenths := proofarith.FloatRat(0.7)
	nearNumerator := new(big.Int).Lsh(heldSevenTenths.Num(),
		uint(8191-(heldSevenTenths.Denom().BitLen()-1)))
	nearNumerator.Add(nearNumerator, big.NewInt(1))
	nearFloat := new(big.Rat).SetFrac(nearNumerator,
		new(big.Int).Lsh(big.NewInt(1), 8191))
	require.True(t, loftcontact.VolumeRatFits(nearFloat))
	_, rounded = loftcontact.VolumeRoundUp(nearFloat)
	require.False(t, rounded)
	deepDenominator := new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Mul(big.NewInt(3), new(big.Int).Lsh(big.NewInt(1), 8138)))
	require.True(t, loftcontact.VolumeRatFits(deepDenominator))
	_, rounded = loftcontact.VolumeRoundUp(deepDenominator)
	require.False(t, rounded)

	for _, value := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3), proofarith.FloatRat(0.7)} {
		got, ok := loftcontact.VolumeRoundUp(value)
		require.True(t, ok)
		require.Equal(t, math.Float64bits(proofbound.RatFloatUp(value)), math.Float64bits(got))
	}
	extremeDyadic := new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 8191))
	got, ok := loftcontact.VolumeRoundUp(extremeDyadic)
	require.True(t, ok)
	require.Equal(t, math.SmallestNonzeroFloat64, got)
}
