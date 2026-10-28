package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestTurnAt(t *testing.T) {
	t.Parallel()

	// An L-shaped loop, counter-clockwise about +z: a convex corner at 1, a
	// reflex one at 3, and vertex 6 on the straight run from 5 to 0.
	w := &work{verts: []r3.Vec{
		{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 2}, {X: 0, Y: 2}, {X: 0, Y: 1},
	}}
	loop := []int{0, 1, 2, 3, 4, 5, 6}
	up := r3.NewVec(0, 0, 1)
	const margin = 1e-9

	require.Equal(t, 1, w.turnAt(loop, 1, up, margin), "a convex corner")
	require.Equal(t, -1, w.turnAt(loop, 3, up, margin), "a reflex corner")
	require.Equal(t, 0, w.turnAt(loop, 6, up, margin), "a straight run is undecided")
	require.Equal(t, -1, w.turnAt(loop, 1, up.Scale(-1), margin), "the other side of the face")
	require.Equal(t, 0, w.turnAt([]int{0, 2, 4}, 1, up, margin), "a loop without the vertex")
	require.Equal(t, 0, w.turnAt(loop, 1, up, 1), "a turn inside the margin is undecided")
}

func TestChainEndCapArea(t *testing.T) {
	t.Parallel()

	const r, tP, tW = 1.0, 2.0, 0.5
	fillet := r * r * (1 - math.Pi/4)
	cases := []struct {
		name   string
		ec     chainEndCut
		fillet bool
		want   float64
	}{
		{"a convex end face gives up a fillet's cap", chainEndCut{}, true, fillet},
		{"a reflex end face takes a fillet's cap", chainEndCut{reflex: true}, true, -fillet},
		{"a convex end face gives up a chamfer's cap", chainEndCut{}, false, tP * tW / 2},
		{"a reflex end face takes a chamfer's cap", chainEndCut{reflex: true}, false, -tP * tW / 2},
		{"an arc end gives up its own corner", chainEndCut{arc: &arcEnd{cornerArea: 0.3}}, true, 0.3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			require.InDelta(t, c.want, c.ec.capArea(c.fillet, r, tP, tW), 1e-15)
		})
	}
}
