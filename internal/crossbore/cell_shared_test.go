package crossbore

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/stretchr/testify/require"
)

func requireCrossIVEquals(t *testing.T, want, got crossIV) {
	t.Helper()
	require.Zero(t, want.Lo.Rat().Cmp(got.Lo.Rat()))
	require.Zero(t, want.Hi.Rat().Cmp(got.Hi.Rat()))
}

// TestCrossBoreSharedConstantsStayUnwritten pins the read-only contract of
// the package-level constant points: proving the same cell twice gives the
// same answer, and the constants still equal freshly built ones afterwards.
func TestCrossBoreSharedConstantsStayUnwritten(t *testing.T) {
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(2*HalfStations, 1))
	first, err := CellExactProof(5, 1, 0.5, 64, gap)
	require.NoError(t, err)
	second, err := CellExactProof(5, 1, 0.5, 64, gap)
	require.NoError(t, err)
	for c := range first.Coords {
		requireCrossIVEquals(t, first.Coords[c], second.Coords[c])
	}
	for c := range first.Derivs {
		requireCrossIVEquals(t, first.Derivs[c], second.Derivs[c])
	}
	require.Equal(t, first.Width, second.Width)
	require.Equal(t, first.Speed, second.Speed)
	require.Equal(t, first.BandLength, second.BandLength)

	for _, row := range []struct {
		shared crossIV
		fresh  float64
	}{
		{crossZero, 0}, {crossOne, 1}, {crossTwo, 2},
		{crossOuterScale, 1 + BandExtension}, {crossExtension, BandExtension},
		{crossTwistScale, 1 + 2*BandExtension},
	} {
		requireCrossIVEquals(t, crossPoint(row.fresh), row.shared)
	}

	// The band consumers read the scale constants; running them must not
	// move the constants or change a second reading.
	w1, w2 := bandWidthReading(t, &first), bandWidthReading(t, &second)
	require.Equal(t, w1, w2)
	requireCrossIVEquals(t, crossPoint(1+2*BandExtension), crossTwistScale)
	requireCrossIVEquals(t, crossPoint(BandExtension), crossExtension)
	requireCrossIVEquals(t, crossPoint(0), crossZero)
}

func bandWidthReading(t *testing.T, cell *CellExact) [2]float64 {
	t.Helper()
	spread := quadSpread(cell)
	dens, _, ok := bandDensity(cell)
	require.True(t, ok)
	return [2]float64{spread, dens[0]}
}
