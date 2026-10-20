package breprecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/stretchr/testify/require"
)

func TestP8QuarterRadius(t *testing.T) {
	t.Parallel()

	for _, ok := range []float64{0.25, 0.5, 0.75, 1} {
		require.True(t, breprecord.P8QuarterRadius(ok), "%v", ok)
	}
	for _, bad := range []float64{0, -0.25, 0.125, 0.3, 1.25, 1.0000000000000002, math.NaN(), math.Inf(1)} {
		require.False(t, breprecord.P8QuarterRadius(bad), "%v", bad)
	}
}
