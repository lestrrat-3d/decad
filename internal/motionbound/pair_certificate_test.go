package motionbound_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/stretchr/testify/require"
)

func TestCanSkipProjection(t *testing.T) {
	tests := []struct {
		name   string
		travel *big.Rat
		prior  *big.Rat
		want   bool
	}{
		{name: "equal bound tie", travel: big.NewRat(3, 1), prior: big.NewRat(3, 1), want: true},
		{name: "no prior bound", travel: big.NewRat(3, 1)},
		{name: "no travel proof", prior: big.NewRat(3, 1)},
		{name: "nonpositive travel", travel: big.NewRat(0, 1), prior: big.NewRat(3, 1)},
		{name: "travel below prior", travel: big.NewRat(2, 1), prior: big.NewRat(3, 1)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			require.Equal(t, tt.want, motionbound.CanSkipProjection(tt.travel, tt.prior))
		})
	}
}

func TestIntervalPairLowerProjectionImprovesMinimum(t *testing.T) {
	travel := motionbound.IntervalTravelLower(4, 4, big.NewRat(2, 1))
	projection := big.NewRat(5, 1)

	require.Equal(t, big.NewRat(3, 1), travel)
	require.Equal(t, projection, motionbound.IntervalPairLower(4, 4, big.NewRat(2, 1), projection))
}

func TestIntervalTravelLowerAbsentOrNonpositive(t *testing.T) {
	require.Nil(t, motionbound.IntervalTravelLower(4, 4, nil))
	require.Equal(t, big.NewRat(0, 1), motionbound.IntervalTravelLower(1, 1, big.NewRat(2, 1)))
	require.Nil(t, motionbound.IntervalPairLower(1, 1, big.NewRat(2, 1), nil))
}
