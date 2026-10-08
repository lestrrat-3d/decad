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

// TestMirrorCornerJoin reads the corner a slant makes with its own mirror image
// across the u axis, at both of its ends. The slant from (20, 0) to (0, 10)
// leaves the axis at its start, where the mirror union shows a convex apex: a
// miter inward, on the axis at u = 20 − √5·t, and an arc outward from the
// slant's offset foot to the axis point (20 + t, 0). Walked the other way it
// arrives at the axis at its end with its material on the other side, so the
// mirror union shows a reflex notch there: the miter is outward, at the same
// u = 20 − √5·t, and the arc inward, running foot first to (20 + t, 0). A walk
// meeting the axis at a right angle is a G1 join at its own foot.
func TestMirrorCornerJoin(t *testing.T) {
	const tt = 1.0
	axis := offset2d.Curve{IsLine: true, DX: 1}
	n := 1 / math.Sqrt(5)
	leave := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: 20, EndU: 0, EndV: 10, TanInU: -20, TanInV: 10, TanOutU: -20, TanOutV: 10}}
	arrive := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: 0, StartV: 10, EndU: 20, TanInU: 20, TanInV: -10, TanOutU: 20, TanOutV: -10}}

	miter, err := offset2d.MirrorCornerJoin(leave, false, axis, 1, tt, 1e-9)
	require.NoError(t, err)
	require.False(t, miter.Arc || miter.G1)
	require.InDelta(t, 20-math.Sqrt(5)*tt, miter.M.U, 1e-12)
	require.InDelta(t, 0, miter.M.V, 1e-12)
	back, err := offset2d.MirrorCornerJoin(arrive, true, axis, -1, tt, 1e-9)
	require.NoError(t, err)
	require.False(t, back.Arc || back.G1)
	require.InDelta(t, 20-math.Sqrt(5)*tt, back.M.U, 1e-12)
	require.InDelta(t, 0, back.M.V, 1e-12)

	arc, err := offset2d.MirrorCornerJoin(leave, false, axis, -1, tt, 1e-9)
	require.NoError(t, err)
	require.True(t, arc.Arc)
	require.Equal(t, offset2d.Point{U: 20 + tt, V: 0}, arc.PA, `the arc starts on the axis`)
	require.InDelta(t, 20+tt*n, arc.PB.U, 1e-12)
	require.InDelta(t, 2*tt*n, arc.PB.V, 1e-12)
	end, err := offset2d.MirrorCornerJoin(arrive, true, axis, 1, tt, 1e-9)
	require.NoError(t, err)
	require.True(t, end.Arc)
	require.Equal(t, offset2d.Point{U: 20 + tt, V: 0}, end.PB, `the arc ends on the axis`)
	require.InDelta(t, 20+tt*n, end.PA.U, 1e-12)
	require.InDelta(t, 2*tt*n, end.PA.V, 1e-12)

	square := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: 20, EndU: 20, EndV: 10, TanInV: 1, TanOutV: 1}}
	g1, err := offset2d.MirrorCornerJoin(square, false, axis, 1, tt, 1e-9)
	require.NoError(t, err)
	require.True(t, g1.G1)
	require.Equal(t, offset2d.Point{U: 20 - tt, V: 0}, g1.M)
}
