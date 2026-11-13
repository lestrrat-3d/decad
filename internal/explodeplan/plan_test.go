package explodeplan_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// blocker blocks any move that ends above z = limit.
type blocker struct{ limit float64 }

func (b blocker) Path(_ context.Context, _ int, _, to r3.Vec, _ []r3.Vec, _ []int) (explodeplan.PathState, int, error) {
	if to.Z > b.limit {
		return explodeplan.PathBlocked, 0, nil
	}
	return explodeplan.PathClear, -1, nil
}

func box(x0, y0, z0, x1, y1, z1 float64) explodeplan.Box {
	return explodeplan.Box{Min: r3.Vec{X: x0, Y: y0, Z: z0}, Max: r3.Vec{X: x1, Y: y1, Z: z1}}
}

func nan() float64 { return math.NaN() }

// run runs one implicit ClearJoined stage over every part but the ground.
func run(t *testing.T, boxes []explodeplan.Box, joints []explodeplan.Joint, gap float64, b explodeplan.Blocker) []explodeplan.Result {
	t.Helper()
	order, nodes := explodeplan.Walk(len(boxes), joints, 0)
	stages := []explodeplan.Stage{{Members: order, Gap: nan()}}
	partGap := make([]float64, len(boxes))
	for i := range partGap {
		partGap[i] = nan()
	}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Joints: joints, Ground: 0, Order: order, Nodes: nodes,
		Stages: stages, DefaultGap: gap, PartGap: partGap, Scale: 1, Blocker: b})
	require.NoError(t, err)
	return out.Parts
}

func TestRunAxis(t *testing.T) {
	boxes := []explodeplan.Box{box(0, 0, 0, 20, 20, 10), box(7, 7, -2, 13, 13, 15)}
	joints := []explodeplan.Joint{{A: 1, B: 0, Mode: explodeplan.ModeAxis, Dir: r3.Vec{Z: 1}, Source: 7}}
	res := run(t, boxes, joints, 10, blocker{limit: 100})
	require.Equal(t, r3.Vec{Z: 22}, res[1].Offset)
	require.Equal(t, 7, res[1].Source)
	require.Equal(t, explodeplan.PathClear, res[1].Path)
	require.InDelta(t, 12.0, res[1].Travel, 1e-12)
	require.InDelta(t, 22.0, res[1].Moved, 1e-12)
	require.InDelta(t, 10.0, res[1].Gap, 1e-12)
	require.Equal(t, r3.Vec{Z: 1}, res[1].Dir)
	require.Equal(t, explodeplan.PathNone, res[0].Path)
	require.Zero(t, res[1].Clearance, "joined parts do not count, and the pin is joined to the only other part")
	require.False(t, res[1].Short)

	// With the upward path blocked the pin goes down: 15 mm clear plus the gap.
	res = run(t, boxes, joints, 10, blocker{limit: 0})
	require.Equal(t, r3.Vec{Z: -25}, res[1].Offset)
	require.Equal(t, explodeplan.PathClear, res[1].Path, "the sense taken is the free one")
}

func TestRunChainAndUnreached(t *testing.T) {
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 10), box(20, 0, 0, 30, 10, 10), box(40, 0, 0, 50, 10, 10),
		box(0, 50, 0, 10, 60, 10)}
	joints := []explodeplan.Joint{
		{A: 1, B: 0, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: 1}},
		{A: 2, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: 1}},
	}
	res := run(t, boxes, joints, 5, blocker{limit: 100})
	// Part 1 already clears part 0, so only the gap; part 2 adds its own gap to part 1's offset.
	require.Equal(t, r3.Vec{X: 5}, res[1].Offset)
	require.Equal(t, r3.Vec{X: 10}, res[2].Offset)
	require.Equal(t, explodeplan.PathNone, res[3].Path)
	require.Equal(t, r3.Vec{}, res[3].Offset)
	// ModeAway moves B opposite to Dir when B is the child.
	res = run(t, boxes, []explodeplan.Joint{{A: 0, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: -1}}}, 5, blocker{limit: 100})
	require.Equal(t, r3.Vec{X: 5}, res[1].Offset)
}

func TestRunClearAllIntervals(t *testing.T) {
	// A row of standing boxes along X with gaps of 3, 8 and 30 mm between them; a 4 mm box
	// asked for a 5 mm gap fits only in the 30 mm space.
	boxes := []explodeplan.Box{box(-6, 0, 0, -2, 4, 4), box(0, -1, -1, 10, 5, 5), box(13, -1, -1, 23, 5, 5),
		box(31, -1, -1, 41, 5, 5), box(71, -1, -1, 81, 5, 5)}
	joints := []explodeplan.Joint{{A: 0, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: -1}}}
	// The box is part 0, the ground is part 1: the walk gives part 0 the joint to part 1.
	order, nodes := explodeplan.Walk(len(boxes), joints, 1)
	require.Equal(t, []int{0}, order)
	partGap := []float64{nan(), nan(), nan(), nan(), nan()}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Joints: joints, Ground: 1, Order: order, Nodes: nodes,
		Stages:     []explodeplan.Stage{{Members: []int{0}, Gap: 5, Clear: explodeplan.ClearAll, Dir: r3.Vec{X: 1}}},
		DefaultGap: 10, PartGap: partGap, Scale: 1, Blocker: blocker{limit: 100}})
	require.NoError(t, err)
	res := out.Parts
	// The intervals of t are (-3, 21), (10, 34), (28, 52) and (68, 92): the first three merge,
	// so the box stops at t = 52, between the third and the fourth box.
	require.InDelta(t, 52.0, res[0].Offset.X, 1e-9)
	require.Equal(t, explodeplan.StageDirection, res[0].Source)
	require.InDelta(t, 47.0, res[0].Travel, 1e-9, "the move less the 5 mm gap")
	require.InDelta(t, 52.0, res[0].Moved, 1e-9)
	// Its box now spans x 46 to 50: 5 mm from the third box and 21 from the fourth.
	require.InDelta(t, 5.0, res[0].Clearance, 1e-9)
	require.False(t, res[0].Short)
}

func TestRunStuck(t *testing.T) {
	// A standing box 3,000,000 mm long that the moving box would have to cross.
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 10), box(2, 2, 12, 4, 4, 14), box(-1e6, -1, -1, 2e6, 11, 20)}
	joints := []explodeplan.Joint{{A: 1, B: 0, Mode: explodeplan.ModeAway, Dir: r3.Vec{Z: 1}}}
	order, nodes := explodeplan.Walk(len(boxes), joints, 0)
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Joints: joints, Ground: 0, Order: order, Nodes: nodes,
		Stages:     []explodeplan.Stage{{Members: []int{1}, Gap: 1, Clear: explodeplan.ClearAll, Dir: r3.Vec{X: 1}}},
		DefaultGap: 10, PartGap: []float64{nan(), nan(), nan()}, Scale: 1, Blocker: blocker{limit: 100}})
	require.NoError(t, err)
	res := out.Parts
	require.Equal(t, explodeplan.PathStuck, res[1].Path)
	require.Equal(t, r3.Vec{}, res[1].Offset, "a stuck part stays at its start")
}

// runRound runs one derived round that lifts the parts of order, in that order, off the ground, part 0,
// by a 6 mm gap.
func runRound(t *testing.T, boxes []explodeplan.Box, order, copies []int) []explodeplan.Result {
	t.Helper()
	_, nodes := explodeplan.Walk(len(boxes), nil, 0)
	partGap := make([]float64, len(boxes))
	for i := range partGap {
		partGap[i] = nan()
	}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Ground: 0, Order: order, Nodes: nodes,
		Stages:     []explodeplan.Stage{{Members: order, Gap: nan(), Clear: explodeplan.ClearAll, Dir: r3.Vec{Z: 1}, Round: 1}},
		DefaultGap: 6, PartGap: partGap, Scale: 1, Blocker: blocker{limit: 1000}, Copies: copies})
	require.NoError(t, err)
	return out.Parts
}

func TestRunRoundCopies(t *testing.T) {
	// A hub (part 1) and two copies of one 22 mm square part (parts 2 and 3) beside it on a base. Seen
	// along Z the first copy's box clears the hub's, the second's overlaps its corner.
	boxes := []explodeplan.Box{box(-40, -40, -5, 40, 40, 0), box(-7.5, -7.5, 0, 7.5, 7.5, 8),
		box(10, -11, 1, 32, 11, 8), box(-21.5, 7.2, 1, 0.5, 29.2, 8)}

	// Each part alone: the second copy ends beyond the hub, the first does not.
	res := runRound(t, boxes, []int{1, 2, 3}, nil)
	require.InDelta(t, 6.0, res[1].Offset.Z, 1e-9, "the hub clears the base by the gap")
	require.InDelta(t, 5.0, res[2].Offset.Z, 1e-9)
	require.InDelta(t, 19.0, res[3].Offset.Z, 1e-9, "the box overlapping the hub's ends 6 mm beyond the hub")

	// As copies of one shape both end beyond the hub that one of them passes.
	res = runRound(t, boxes, []int{1, 2, 3}, []int{-1, -1, 0, 0})
	require.InDelta(t, 6.0, res[1].Offset.Z, 1e-9)
	require.InDelta(t, 19.0, res[2].Offset.Z, 1e-9)
	require.InDelta(t, 19.0, res[3].Offset.Z, 1e-9)
	require.InDelta(t, 13.0, res[2].Travel, 1e-9, "the move less the gap")
	require.Equal(t, explodeplan.PathClear, res[2].Path)

	// A part outside the round standing over the first copy only (part 4, z 20 to 22) is cleared by that
	// copy alone: each copy clears the parts of other stages by its own box.
	pillar := append(append([]explodeplan.Box{}, boxes...), box(26, -5, 20, 32, 5, 22))
	res = runRound(t, pillar, []int{1, 2, 3}, []int{-1, -1, 0, 0, -1})
	require.InDelta(t, 22.0+6-1, res[2].Offset.Z, 1e-9, "the first copy ends 6 mm above the pillar")
	require.InDelta(t, 19.0, res[3].Offset.Z, 1e-9, "the second copy ends beyond the hub only")

	// With the hub between the copies in the stage order, the copies leave together at the first one's
	// turn, and the hub, which the second copy's box overlaps, ends beyond both.
	res = runRound(t, boxes, []int{2, 1, 3}, []int{-1, -1, 0, 0})
	require.InDelta(t, 5.0, res[2].Offset.Z, 1e-9)
	require.InDelta(t, 5.0, res[3].Offset.Z, 1e-9)
	require.InDelta(t, 19.0, res[1].Offset.Z, 1e-9, "the hub ends 6 mm above the copies' tops at z 13")

	// Copies that overlap each other seen along Z, as washers stacked on one pin, still clear each other
	// one by one.
	stacked := []explodeplan.Box{box(-40, -40, -5, 40, 40, 0), box(-5, -5, 0, 5, 5, 1), box(-5, -5, 1, 5, 5, 2)}
	res = runRound(t, stacked, []int{1, 2}, []int{-1, 0, 0})
	require.InDelta(t, 6.0, res[1].Offset.Z, 1e-9)
	require.InDelta(t, 12.0, res[2].Offset.Z, 1e-9, "the upper washer ends 6 mm above the lower one's top at z 7")
}

func TestRunTogetherAndScale(t *testing.T) {
	// Two parts side by side on a base, lifted as one over a cover that overhangs both.
	boxes := []explodeplan.Box{box(0, 0, 0, 40, 10, 5), box(2, 2, 5, 8, 8, 9), box(12, 2, 5, 18, 8, 9), box(0, 0, 9, 40, 10, 12)}
	joints := []explodeplan.Joint{
		{A: 1, B: 0, Mode: explodeplan.ModeAway, Dir: r3.Vec{Z: 1}},
		{A: 2, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: 1}},
	}
	order, nodes := explodeplan.Walk(len(boxes), joints, 0)
	partGap := []float64{nan(), nan(), nan(), nan()}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Joints: joints, Ground: 0, Order: order, Nodes: nodes,
		Stages:     []explodeplan.Stage{{Members: []int{1, 2}, Gap: 3, Clear: explodeplan.ClearAll, Dir: r3.Vec{Z: 1}, Together: true}},
		DefaultGap: 10, PartGap: partGap, Scale: 2, Blocker: blocker{limit: 1000}})
	require.NoError(t, err)
	res := out.Parts
	// The group's box (z 5 to 9) clears the cover (z 9 to 12) grown by 3: it rises 10 mm... the base
	// top at 5 grown by 3 reaches 8, the cover to 15, so the group rises until its bottom is at 15.
	require.InDelta(t, 20.0, res[1].Offset.Z, 1e-9, "10 mm, doubled by the scale")
	require.Equal(t, res[1].Offset, res[2].Offset, "members keep their relative place")
	require.InDelta(t, 20.0, res[1].Moved, 1e-9)
}

func TestBoxDistanceAndProjectedCover(t *testing.T) {
	a, b := box(0, 0, 0, 10, 10, 10), box(13, 14, 0, 20, 20, 10)
	require.InDelta(t, 5.0, explodeplan.BoxDistance(a, b), 1e-12)
	require.InDelta(t, 0.0, explodeplan.BoxDistance(a, box(5, 5, 5, 20, 20, 20)), 1e-12)

	// Looking down -Z, the higher box is nearer; it covers half of the lower one's rectangle.
	low, high := box(0, 0, 0, 10, 10, 2), box(5, 0, 6, 15, 10, 8)
	front, cover := explodeplan.ProjectedCover(low, high, r3.Vec{Z: -1})
	require.Equal(t, 1, front)
	require.InDelta(t, 0.5, cover, 1e-12)
	front, cover = explodeplan.ProjectedCover(low, high, r3.Vec{Z: 1})
	require.Equal(t, 0, front, "looking up, the lower box is nearer")
	require.InDelta(t, 0.5, cover, 1e-12)
	_, cover = explodeplan.ProjectedCover(low, box(30, 0, 0, 40, 10, 2), r3.Vec{Z: -1})
	require.Zero(t, cover)
}

func TestPortAndMeet(t *testing.T) {
	a, b := box(0, 0, 0, 10, 10, 10), box(5, 5, 5, 15, 15, 15)
	require.Equal(t, r3.Vec{X: 7.5, Y: 7.5, Z: 7.5}, explodeplan.Port(a, b))
	require.True(t, explodeplan.Meet(a, b))
	c := box(20, 0, 0, 30, 10, 10)
	require.Equal(t, r3.Vec{X: 15, Y: 5, Z: 5}, explodeplan.Port(a, c))
	require.False(t, explodeplan.Meet(a, c))
	require.True(t, explodeplan.Meet(a, box(10, 0, 0, 20, 10, 10)))
}
