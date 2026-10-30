package revolveboretess

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRadiusChargesWorldAllowanceThroughFrameDefect(t *testing.T) {
	radius, ok := radiusAfterWorldAllowance(big.NewRat(5, 1), 0.25, big.NewRat(1, 1))
	require.True(t, ok)
	require.Zero(t, radius.Cmp(big.NewRat(11, 3)))
	_, ok = radiusAfterWorldAllowance(big.NewRat(5, 1), 1, big.NewRat(1, 1))
	require.False(t, ok)
}
