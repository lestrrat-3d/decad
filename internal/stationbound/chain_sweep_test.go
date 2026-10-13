package stationbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/stretchr/testify/require"
)

func TestChainSweepArcStation(t *testing.T) {
	t.Run("zero turn is exact", func(t *testing.T) {
		p, gap, err := stationbound.ChainSweepArcStation(2, 3, 1, 0, 4)
		require.NoError(t, err)
		require.Equal(t, 2.0, p.X)
		require.Equal(t, 0.0, p.Y)
		require.Equal(t, 2.0, p.Z)
		// The sine and cosine intervals of a zero turn are not points, so the gap is
		// positive but far below one ulp of the coordinates.
		require.GreaterOrEqual(t, gap, 0.0)
		require.Less(t, gap, 1e-15)
	})
	t.Run("sixty degree station", func(t *testing.T) {
		p, gap, err := stationbound.ChainSweepArcStation(2, 3, 1, 1, 3)
		require.NoError(t, err)
		require.False(t, math.IsInf(gap, 0) || math.IsNaN(gap))
		require.LessOrEqual(t, math.Abs(p.X-(3-0.5)), gap+1e-15)
		require.LessOrEqual(t, math.Abs(p.Z-(2+math.Sqrt(3)/2)), gap+1e-15)
		require.Equal(t, 0.0, p.Y)
	})
}
