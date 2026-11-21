package loftrecord

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestStraightWallPlaneIncludesOppositeCornerAndRounding(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	const rounding = 1.0 / 1024
	for _, z := range []float64{0, -0.125, 0.125} {
		corners := [4]r3.Vec{{}, {X: 1}, {Y: 1}, {X: 1, Y: 1, Z: z}}
		bound, ok := straightWallDeparture(frame, corners, rounding)
		require.True(t, ok)
		want := rounding
		if z != 0 {
			want += 0.125
		}
		require.GreaterOrEqual(t, bound, want)
		require.InDelta(t, want, bound, 1e-15)
	}
}
