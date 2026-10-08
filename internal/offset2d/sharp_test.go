package offset2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// lineWalk is a straight walk from (u0, v0) to (u1, v1).
func lineWalk(u0, v0, u1, v1 float64) survey2d.SideWalk {
	du, dv := u1-u0, v1-v0
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: u0, StartV: v0, EndU: u1, EndV: v1,
		TanInU: du, TanInV: dv, TanOutU: du, TanOutV: dv,
	}}
}

// polygonWalks walks the closed polygon pts in order.
func polygonWalks(pts ...[2]float64) []survey2d.SideWalk {
	walks := make([]survey2d.SideWalk, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		walks[i] = lineWalk(p[0], p[1], q[0], q[1])
	}
	return walks
}

// lWalks is docs/draft-design.md F4's L section, counter-clockwise, its one
// reflex corner at (10, 10).
func lWalks() []survey2d.SideWalk {
	return polygonWalks([2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 10},
		[2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20})
}

// TestSharpJoinsMiterTheReflexCorner pins the one row the sharp rule changes
// (docs/draft-design.md §2): the L section's reflex corner, which JoinsBudget
// closes with a connector arc, is the miter of its two moved walls, (10 − t,
// 10 − t), and every other corner is the same miter JoinsBudget takes.
func TestSharpJoinsMiterTheReflexCorner(t *testing.T) {
	const tt = 0.75
	budget := proofbound.NewWorkBudget(t.Context())
	walks := lWalks()
	sharp, err := offset2d.SharpJoinsBudget(budget, walks, 1, tt, 1e-9)
	require.NoError(t, err)
	shell, err := offset2d.JoinsBudget(budget, walks, 1, tt, 1e-9)
	require.NoError(t, err)
	for i, j := range sharp {
		require.False(t, j.Arc || j.G1)
		if i == 3 {
			require.True(t, shell[i].Arc, "the shell offset rounds the reflex corner")
			require.Equal(t, offset2d.Point{U: 10 - tt, V: 10 - tt}, j.M)
			continue
		}
		require.Equal(t, shell[i].M, j.M)
	}
}

// TestSharpJoinsKeepG1AndRefuseOthers covers the circular rows: a line leaving
// an arc tangentially moves along the shared normal, a line meeting an arc at
// a right angle is ErrCircularMiter (SD4), and two antiparallel lines (a cusp)
// are ErrTopology (SD15).
func TestSharpJoinsKeepG1AndRefuseOthers(t *testing.T) {
	const tt = 0.5
	// A counter-clockwise quarter arc of radius 5 about the origin ending at
	// (0, 5), its tangent there (−1, 0).
	arc := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind: survey2d.WalkCircular, CU: 0, CV: 0, Radius: 5, Th0: 0, Th1: math.Pi / 2,
		StartU: 5, StartV: 0, EndU: 0, EndV: 5, TanInU: 0, TanInV: 1, TanOutU: -1, TanOutV: 0,
	}}
	tangent := lineWalk(0, 5, -10, 5)
	j, err := offset2d.SharpCornerJoin(arc, tangent, 1, tt, 1e-9)
	require.NoError(t, err)
	require.True(t, j.G1)
	require.Equal(t, offset2d.Point{U: 0, V: 5 - tt}, j.M)

	square := lineWalk(0, 5, 0, 10)
	_, err = offset2d.SharpCornerJoin(arc, square, 1, tt, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrCircularMiter)

	budget := proofbound.NewWorkBudget(t.Context())
	_, err = offset2d.SharpJoinsBudget(budget, []survey2d.SideWalk{lineWalk(0, 0, 10, 0), lineWalk(10, 0, 0, 0)}, 1, tt, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrTopology)
}

// TestBuildSharpLoopConsumption reads the three outcomes of a square's offset
// as t grows: a smaller square, one wall consumed of a thin rectangle
// (ErrDrop, SD7), and every wall of the square consumed (ErrLoopConsumed,
// SD6).
func TestBuildSharpLoopConsumption(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	square := polygonWalks([2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 20}, [2]float64{0, 20})
	segs, joins, err := offset2d.BuildSharpLoop(budget, square, 1, 2, 1e-9)
	require.NoError(t, err)
	require.Len(t, segs, 4)
	require.Len(t, joins, 4)
	require.Equal(t, sectionrecord.Point2{U: 2, V: 2}, segs[0].(sectionrecord.LineSeg).Start)

	thin := polygonWalks([2]float64{0, 0}, [2]float64{40, 0}, [2]float64{40, 4}, [2]float64{0, 4})
	_, _, err = offset2d.BuildSharpLoop(budget, thin, 1, 3, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrDrop)
	require.NotErrorIs(t, err, offset2d.ErrLoopConsumed)

	_, _, err = offset2d.BuildSharpLoop(budget, square, 1, 12, 1e-9)
	require.ErrorIs(t, err, offset2d.ErrLoopConsumed)

	// The L's reflex corner stays sharp in the record: the far section is a
	// six-sided polygon, its notch corner at (10 − t, 10 − t).
	l, _, err := offset2d.BuildSharpLoop(budget, lWalks(), 1, 1, 1e-9)
	require.NoError(t, err)
	require.Len(t, l, 6)
	require.Equal(t, sectionrecord.Point2{U: 9, V: 9}, l[3].(sectionrecord.LineSeg).Start)
}
