package filletband_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// The blind-pocket shell's floor band (internal/stackedshell) reads the
// pocket's rounded offset as fillet walks. Their offset loop must collapse
// back to the source square, and each of the four corners must be a sphere.
func TestBlindPocketFloorBandHasFourSphereOctants(t *testing.T) {
	t.Parallel()
	budget := proofbound.NewWorkBudget(t.Context())
	pocket := prismshell.BossRect{U0: 3, V0: 3, U1: 7, V1: 7}.Profile()
	rounded, err := offset2d.OffsetProfile(budget, pocket, -1, 1, offset2d.ShellTol)
	require.NoError(t, err)
	delta, err := offset2d.OffsetSectionDelta(budget, pocket, -1, 1, 0, offset2d.ShellTol)
	require.NoError(t, err)
	require.Zero(t, delta)
	read, err := breprecord.FilletLoopOf(budget, rounded.Outer, 1, "pocket floor", freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := filletband.OffsetJoins(budget, read.Walks, 1, 0, offset2d.ShellTol)
	require.NoError(t, err)
	collapsed, err := filletband.OffsetLoop(budget, read.Walks, joins, 1, offset2d.ShellTol)
	require.NoError(t, err)
	require.Equal(t, pocket.Outer.Segments, collapsed)
	spheres := 0
	for _, walk := range read.Walks {
		if filletband.SphereWalk(walk, 1) {
			spheres++
		}
	}
	require.Equal(t, 4, spheres)
}

// loopOf is a closed loop record through the given segments.
func loopOf(segs ...sectionrecord.CurveSegment) sectionrecord.LoopRecord {
	return sectionrecord.LoopRecord{Segments: segs}
}

func line(u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
}

// ccwArc is a counter-clockwise arc from (u0, v0) to (u1, v1) about (cu, cv).
func ccwArc(cu, cv, u0, v0, u1, v1 float64) sectionrecord.CurveSegment {
	return sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: cu, V: cv}, Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1}, TStart: 0, TEnd: 1}
}

// offsetJoinsOf reads loop's corner walks and its fillet joins at radius r,
// beside the plain offset joins offset2d builds for the same walks.
func offsetJoinsOf(t *testing.T, loop sectionrecord.LoopRecord, r float64) ([]offset2d.Join, []offset2d.Join) {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	joins, err := filletband.OffsetJoins(budget, cl.Walks, r, 0, offset2d.ShellTol)
	require.NoError(t, err)
	plain, err := offset2d.SectionJoinsBudget(budget, cl.Walks, 1, r, offset2d.ShellTol)
	require.NoError(t, err)
	return joins, plain
}

// TestOffsetJoinsHoldExactTangentFeet offsets P8's top loop (the 40 x 20
// rectangle with its corners rounded r = 3) by the fillet radius 2. Each of
// its eight corners is a G1 join of a line and a quarter arc, and each foot
// is the recorded corner moved 2 along a rational unit normal, so the cap
// contour holds it exactly: (2, 3) at the corner (0, 3), where the arc's
// tangent read through math.Sincos at the angle π puts offset2d's foot at
// (2, 2.9999999999999996). The contour then states no displacement.
// Shown to fail with exactFeet's G1 arm deleted (the (0, 3) foot stayed
// off by one ulp and the contour stated it).
func TestOffsetJoinsHoldExactTangentFeet(t *testing.T) {
	t.Parallel()
	loop := loopOf(
		line(3, 0, 37, 0), ccwArc(37, 3, 37, 0, 40, 3),
		line(40, 3, 40, 17), ccwArc(37, 17, 40, 17, 37, 20),
		line(37, 20, 3, 20), ccwArc(3, 17, 3, 20, 0, 17),
		line(0, 17, 0, 3), ccwArc(3, 3, 0, 3, 3, 0),
	)
	joins, _ := offsetJoinsOf(t, loop, 2)
	feet := map[sectionrecord.Point2]sectionrecord.Point2{
		{U: 3, V: 0}: {U: 3, V: 2}, {U: 37, V: 0}: {U: 37, V: 2},
		{U: 40, V: 3}: {U: 38, V: 3}, {U: 40, V: 17}: {U: 38, V: 17},
		{U: 37, V: 20}: {U: 37, V: 18}, {U: 3, V: 20}: {U: 3, V: 18},
		{U: 0, V: 17}: {U: 2, V: 17}, {U: 0, V: 3}: {U: 2, V: 3},
	}
	require.Len(t, joins, len(feet))
	for _, j := range joins {
		require.True(t, j.G1, "the join at (%v, %v)", j.VertU, j.VertV)
		want, ok := feet[sectionrecord.Point2{U: j.VertU, V: j.VertV}]
		require.True(t, ok, "a join at (%v, %v)", j.VertU, j.VertV)
		require.Equal(t, want, j.M, "the foot of the join at (%v, %v)", j.VertU, j.VertV)
	}
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	delta, err := filletband.ContourDelta(budget, cl.Walks, 2, 0, offset2d.ShellTol)
	require.NoError(t, err)
	require.Zero(t, delta)
}

// TestOffsetJoinsHoldExactConnectorFeet offsets a loop by 0.5 whose quarter
// arc about the origin ends at (0, 2) on a reflex corner, where a line
// leaves straight up. The connector arc runs from the arc's foot to the
// line's: (0, 1.5), where offset2d steps along the arc's tangent read at the
// angle π/2 and lands at U = -3.06e-17, and (-0.5, 2). Shown to fail with
// exactFeet's connector-arc arm deleted (PA kept the rounded U).
func TestOffsetJoinsHoldExactConnectorFeet(t *testing.T) {
	t.Parallel()
	joins, plain := offsetJoinsOf(t, loopOf(
		ccwArc(0, 0, 2, 0, 0, 2), line(0, 2, 0, 5), line(0, 5, -5, 5),
		line(-5, 5, -5, -5), line(-5, -5, 2, -5), line(2, -5, 2, 0),
	), 0.5)
	reflex := 0
	for i, j := range joins {
		if !j.Arc {
			continue
		}
		require.Equal(t, [2]float64{0, 2}, [2]float64{j.VertU, j.VertV})
		require.NotEqual(t, plain[i].PA, j.PA, "offset2d's own foot is a rounding off")
		require.Equal(t, sectionrecord.Point2{U: 0, V: 1.5}, j.PA)
		require.Equal(t, sectionrecord.Point2{U: -0.5, V: 2}, j.PB)
		reflex++
	}
	require.Equal(t, 1, reflex)
}

// TestOffsetJoinsKeepIrrationalFeet offsets a loop whose arc about the
// origin, of radius sqrt 2, ends at (1, 1) on a reflex corner. The arc's foot
// there steps along (1, 1)/sqrt 2, which no rational holds, so the connector
// keeps offset2d's float foot bit for bit: the exact rule never rounds a
// point onto a float.
func TestOffsetJoinsKeepIrrationalFeet(t *testing.T) {
	t.Parallel()
	joins, plain := offsetJoinsOf(t, loopOf(
		ccwArc(0, 0, 1, -1, 1, 1), line(1, 1, 1, 4), line(1, 4, -4, 4),
		line(-4, 4, -4, -4), line(-4, -4, 1, -1),
	), 0.25)
	reflex := 0
	for i, j := range joins {
		if !j.Arc {
			continue
		}
		require.Equal(t, plain[i].PA, j.PA)
		require.Equal(t, sectionrecord.Point2{U: 0.75, V: 1}, j.PB)
		reflex++
	}
	require.Equal(t, 1, reflex)
}
