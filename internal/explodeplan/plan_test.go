package explodeplan_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// blocker blocks any move that ends above z = limit.
type blocker struct{ limit float64 }

func (b blocker) Clear(_ context.Context, _ int, _, to r3.Vec, _ []r3.Vec, _ []int) (bool, error) {
	return to.Z <= b.limit, nil
}

func box(x0, y0, z0, x1, y1, z1 float64) explodeplan.Box {
	return explodeplan.Box{Min: r3.Vec{X: x0, Y: y0, Z: z0}, Max: r3.Vec{X: x1, Y: y1, Z: z1}}
}

func TestPlanAxis(t *testing.T) {
	boxes := []explodeplan.Box{box(0, 0, 0, 20, 20, 10), box(7, 7, -2, 13, 13, 15)}
	joints := []explodeplan.Joint{{A: 1, B: 0, Mode: explodeplan.ModeAxis, Dir: r3.Vec{Z: 1}, Source: 7}}
	res, err := explodeplan.Plan(t.Context(), boxes, joints, 0, 10, blocker{limit: 100})
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: 22}, res[1].Offset)
	require.Equal(t, 1, res[1].Depth)
	require.Equal(t, 7, res[1].Source)
	require.Equal(t, 0, res[0].Depth)
	require.Equal(t, -1, res[0].Joint)

	// With the upward path blocked the pin goes down: 15 mm clear plus the gap.
	res, err = explodeplan.Plan(t.Context(), boxes, joints, 0, 10, blocker{limit: 0})
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: -25}, res[1].Offset)
}

func TestPlanChainAndUnreached(t *testing.T) {
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 10), box(20, 0, 0, 30, 10, 10), box(40, 0, 0, 50, 10, 10),
		box(0, 50, 0, 10, 60, 10)}
	joints := []explodeplan.Joint{
		{A: 1, B: 0, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: 1}},
		{A: 2, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: 1}},
	}
	res, err := explodeplan.Plan(t.Context(), boxes, joints, 0, 5, blocker{limit: 100})
	require.NoError(t, err)
	// Part 1 already clears part 0, so only the gap; part 2 adds its own gap to part 1's offset.
	require.Equal(t, r3.Vec{X: 5}, res[1].Offset)
	require.Equal(t, r3.Vec{X: 10}, res[2].Offset)
	require.Equal(t, 2, res[2].Depth)
	require.Equal(t, -1, res[3].Depth)
	require.Equal(t, r3.Vec{}, res[3].Offset)
	// ModeAway moves B opposite to Dir when B is the child.
	joints = []explodeplan.Joint{{A: 0, B: 1, Mode: explodeplan.ModeAway, Dir: r3.Vec{X: -1}}}
	res, err = explodeplan.Plan(t.Context(), boxes, joints, 0, 5, blocker{limit: 100})
	require.NoError(t, err)
	require.Equal(t, r3.Vec{X: 5}, res[1].Offset)
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
