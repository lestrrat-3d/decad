package offset2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestSectionJoinsKeepHeldBits(t *testing.T) {
	walls := []survey2d.SideWalk{
		{SegmentWalk: survey2d.SegmentWalk{StartU: 0, StartV: 0, TanInU: 1, TanOutU: 1}},
		{SegmentWalk: survey2d.SegmentWalk{StartU: 10, StartV: 0, TanInV: 1, TanOutV: 1}},
		{SegmentWalk: survey2d.SegmentWalk{StartU: 10, StartV: 10, TanInU: -1, TanOutU: -1}},
		{SegmentWalk: survey2d.SegmentWalk{StartU: 0, StartV: 10, TanInV: -1, TanOutV: -1}},
	}
	budget := proofbound.NewWorkBudget(t.Context())
	inward, err := offset2d.JoinsBudget(budget, walls, 1, 0.75, 1e-9)
	require.NoError(t, err)
	actual := make([][2]uint64, len(inward))
	for i, j := range inward {
		actual[i] = [2]uint64{math.Float64bits(j.M.U), math.Float64bits(j.M.V)}
	}
	require.Equal(t, [][2]uint64{
		{0x3fe8000000000000, 0x3fe8000000000000},
		{0x4022800000000000, 0x3fe8000000000000},
		{0x4022800000000000, 0x4022800000000000},
		{0x3fe8000000000000, 0x4022800000000000},
	}, actual)
	outward, err := offset2d.JoinsBudget(budget, walls, -1, 0.75, 1e-9)
	require.NoError(t, err)
	for _, j := range outward {
		require.True(t, j.Arc)
	}
	require.Equal(t, [2]uint64{0xbfe8000000000000, 0},
		[2]uint64{math.Float64bits(outward[0].PA.U), math.Float64bits(outward[0].PA.V)})
	w := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{Radius: 1.3, Th0: 0.1, Th1: 1.1}}
	r, ok := offset2d.OffsetRadius(w, 1, 0.4, 1e-9)
	require.True(t, ok)
	require.Equal(t, uint64(0x3feccccccccccccd), math.Float64bits(r))
}
