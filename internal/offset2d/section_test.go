package offset2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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

	// A sphere meridian's arc about (5, 0) arrives at the axis at (0, 0), its
	// tangent read through the angle π: cos π is exact, sin π is not, so the
	// foot stepped along the float normal lands at v = −1.2e-16, below the
	// axis. The join takes the axis point itself. Shown to fail with the foot
	// in place of the axis point.
	pole := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: 10, EndU: 0, CU: 5, Radius: 5, Kind: survey2d.WalkCircular, Th0: 0, Th1: math.Pi,
		TanInV: 1, TanOutU: -math.Sin(math.Pi), TanOutV: math.Cos(math.Pi),
	}}
	g1, err = offset2d.MirrorCornerJoin(pole, true, axis, 1, tt, 1e-9)
	require.NoError(t, err)
	require.True(t, g1.G1)
	require.Equal(t, offset2d.Point{U: tt, V: 0}, g1.M)
}

// TestOpenWalkConsumed reads the concave arc of radius 13 about the origin,
// walked clockwise from (0,13) to (5,12), offset outward to radius 10.625:
//
//   - trimmed at the exact cut (0, 10.625) and extended past its end angle to
//     the cut (5, 9.375), it sweeps more than its source, which WalkConsumed
//     reads as consumed and OpenWalkConsumed accepts when the end is an
//     opening end, and only then;
//   - started instead at the angle 59.8°, past that extended end, it runs
//     backward and is consumed with the extension counted;
//   - an end read 100° past the source's end is no rim's extension, so the
//     reading counts nothing and the arc is consumed;
//   - a counter-clockwise arc of nearly a full turn, extended so its
//     allowance reaches a full turn, is consumed though its held sweep is
//     small;
//   - a line takes WalkConsumed's answer.
//
// Shown to fail: counting a reading up to π instead of π/2 accepts the 100°
// end; dropping the full-turn test accepts the nearly full arc.
func TestOpenWalkConsumed(t *testing.T) {
	const tol = 1e-9
	rho := 10.625
	arc := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: 0, StartV: 13, EndU: 5, EndV: 12, CU: 0, CV: 0, Radius: 13,
		Kind: survey2d.WalkCircular, Th0: math.Pi / 2, Th1: math.Atan2(12, 5),
	}}
	start := offset2d.Point{U: 0, V: rho}
	end := offset2d.Point{U: 5, V: 9.375}
	require.True(t, offset2d.WalkConsumed(arc, start, end, tol))
	require.False(t, offset2d.OpenWalkConsumed(arc, start, end, false, true, tol))
	require.False(t, offset2d.OpenWalkConsumed(arc, start, end, true, true, tol))
	require.True(t, offset2d.OpenWalkConsumed(arc, start, end, true, false, tol), "the extension lies at the end")
	require.True(t, offset2d.OpenWalkConsumed(arc, start, end, false, false, tol))

	at := func(r, deg float64) offset2d.Point {
		s, c := math.Sincos(deg * math.Pi / 180)
		return offset2d.Point{U: r * c, V: r * s}
	}
	require.True(t, offset2d.OpenWalkConsumed(arc, at(rho, 59.8), end, true, true, tol), "the start trims past the extended end")
	far := at(rho, math.Atan2(12, 5)*180/math.Pi-100)
	require.True(t, offset2d.OpenWalkConsumed(arc, start, far, false, true, tol), "a 100° reading is no extension")

	full := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: 1, EndU: math.Cos(-0.2), EndV: math.Sin(-0.2), Radius: 1,
		Kind: survey2d.WalkCircular, Th0: 0, Th1: 2*math.Pi - 0.2,
	}}
	require.True(t, offset2d.OpenWalkConsumed(full, at(1, 0.05*180/math.Pi), at(1, 0.1*180/math.Pi), false, true, tol),
		"an allowance of a full turn holds no recorded arc")

	line := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: 0, EndU: 10, TanInU: 1, TanOutU: 1}}
	back := offset2d.Point{U: -1}
	require.Equal(t, offset2d.WalkConsumed(line, offset2d.Point{U: 2}, back, tol),
		offset2d.OpenWalkConsumed(line, offset2d.Point{U: 2}, back, true, true, tol))
	require.True(t, offset2d.OpenWalkConsumed(line, offset2d.Point{U: 2}, back, true, true, tol))
}

// TestSectionJoinsNameTheCornerThatDoesNotMeet pins that a corner whose
// offset carriers do not meet refuses as ErrTopology naming that corner. The
// loop runs (0, 0) → (10, 0) and back, so both corners are cusps whose offset
// lines run parallel and never meet; the first corner resolved is the one at
// the first walk's start, (0, 0). InLoop adds the loop index and keeps the
// sentinel, and leaves any other error unchanged.
//
// Shown to fail: with SectionJoinsBudget returning the bare ErrTopology
// again, the message names no corner.
func TestSectionJoinsNameTheCornerThatDoesNotMeet(t *testing.T) {
	t.Parallel()
	walls := []survey2d.SideWalk{
		{SegmentWalk: survey2d.SegmentWalk{StartU: 0, StartV: 0, EndU: 10, TanInU: 1, TanOutU: 1}},
		{SegmentWalk: survey2d.SegmentWalk{StartU: 10, StartV: 0, TanInU: -1, TanOutU: -1}},
	}
	_, err := offset2d.SectionJoinsBudget(proofbound.NewWorkBudget(t.Context()), walls, 1, 0.5, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrTopology)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, `the offsets of the two walls meeting at (0, 0) do not intersect`)

	named := offset2d.InLoop(err, 2)
	require.ErrorIs(t, named, offset2d.ErrTopology)
	require.ErrorContains(t, named, `meeting at (0, 0) on loop 2 do not intersect`)
	require.Equal(t, offset2d.ErrDrop, offset2d.InLoop(offset2d.ErrDrop, 2))
}

// TestOpenChainNamesTheCornerThatDoesNotMeet pins that an open chain's
// interior corner whose offset carriers do not meet refuses as ErrTopology
// naming that corner. The chain runs (0, 0) → (10, 0) and back to (0, 0), both
// ends on the axis u = 0 at a right angle, so both end joins resolve and the
// interior corner (10, 0) is a cusp whose offset lines run parallel.
//
// Shown to fail: with OffsetOpenChain returning the bare ErrTopology again,
// the message names no corner.
func TestOpenChainNamesTheCornerThatDoesNotMeet(t *testing.T) {
	t.Parallel()
	line := func(u0, v0, u1, v1 float64) survey2d.SideWalk {
		du, dv := u1-u0, v1-v0
		return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
			StartU: u0, StartV: v0, EndU: u1, EndV: v1,
			TanInU: du, TanInV: dv, TanOutU: du, TanOutV: dv,
		}}
	}
	chain := []survey2d.SideWalk{line(0, 0, 10, 0), line(10, 0, 0, 0)}
	axis := offset2d.Curve{IsLine: true, DY: 1}
	mirror := offset2d.OpenEnd{Mirror: true}
	_, err := offset2d.OffsetOpenChain(proofbound.NewWorkBudget(t.Context()), chain, axis, mirror, mirror, 1, 0.5, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrTopology)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, `the offsets of the two walls meeting at (10, 0) do not intersect`)
}
