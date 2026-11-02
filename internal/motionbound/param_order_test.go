package motionbound

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestParamBoundsZeroTurnReturnIndependentBase(t *testing.T) {
	base := big.NewRat(7, 3)
	p := MotionParam{Turn: new(big.Rat), Base: base}

	lower := ParamLower(p)
	upper := ParamUpper(p)
	require.Equal(t, base, lower)
	require.Equal(t, base, upper)
	require.NotSame(t, base, lower)
	require.NotSame(t, base, upper)
	require.NotSame(t, lower, upper)

	lower.SetInt64(19)
	require.Equal(t, big.NewRat(7, 3), base)
	require.Equal(t, big.NewRat(7, 3), upper)

	upper.SetInt64(-2)
	require.Equal(t, big.NewRat(7, 3), base)
	require.Equal(t, big.NewRat(19, 1), lower)

	base.SetInt64(5)
	require.Equal(t, big.NewRat(19, 1), lower)
	require.Equal(t, big.NewRat(-2, 1), upper)
}
