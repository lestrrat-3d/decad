package proofbound_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFrameBoxWorldAndMidStation(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)

	box, ok := proofbound.FrameBoxWorld(frame, frame.Origin(), [2]float64{0, 0}, [2]float64{1, 2})
	require.True(t, ok)
	for i, want := range [3][2]int64{{1, 2}, {2, 4}, {3, 3}} {
		require.Zero(t, box[i].Lo.Cmp(big.NewRat(want[0], 1)), "axis %d lo", i)
		require.Zero(t, box[i].Hi.Cmp(big.NewRat(want[1], 1)), "axis %d hi", i)
	}

	_, ok = proofbound.FrameBoxWorld(frame, frame.Origin(), [2]float64{1, 0}, [2]float64{0, 2})
	require.False(t, ok)
	_, ok = proofbound.FrameBoxWorld(frame, frame.Origin(), [2]float64{math.NaN(), 0}, [2]float64{1, 2})
	require.False(t, ok)

	mid, bound, ok := proofbound.BoxMidStation(box)
	require.True(t, ok)
	require.Equal(t, r3.Vec{X: 1.5, Y: 3, Z: 3}, mid)
	// The widest half extent is 1 (the y interval), so the 3-D allowance is
	// about sqrt(3); a relation, not a literal.
	require.GreaterOrEqual(t, bound, math.Sqrt(3))
	require.Less(t, bound, 1.8)

	reversed := box
	reversed[0].Lo, reversed[0].Hi = box[0].Hi, box[0].Lo
	_, _, ok = proofbound.BoxMidStation(reversed)
	require.False(t, ok)
}
