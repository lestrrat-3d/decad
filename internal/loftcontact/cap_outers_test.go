package loftcontact_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// capFace is a cap whose outer loop walks the given corners, every edge
// optionally walked backward, with the corners placed at height z.
func capFace(corners []r3.Vec, z float64, backward bool) *topo.Face {
	loop := &topo.Loop{Outer: true}
	for i, c := range corners {
		next := corners[(i+1)%len(corners)]
		a := &topo.Vertex{Position: r3.Vec{X: c.X, Y: c.Y, Z: z}}
		b := &topo.Vertex{Position: r3.Vec{X: next.X, Y: next.Y, Z: z}}
		if backward {
			a, b = b, a
		}
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: &topo.Edge{Start: a, End: b}, Forward: !backward})
	}
	return &topo.Face{Loops: []*topo.Loop{loop}}
}

func TestMatchingHeldCapOuters(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	square := []r3.Vec{{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4}}
	start := capFace(square, 0, false)

	require.True(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(square, 7, false)),
		"the heights differ and the loops match")
	rotated := []r3.Vec{square[2], square[3], square[0], square[1]}
	require.True(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(rotated, 7, false)),
		"the walk may start anywhere")
	require.True(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(square, 7, true)),
		"every edge may run the other way")
	reversedOrder := []r3.Vec{square[0], square[3], square[2], square[1]}
	require.True(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(reversedOrder, 7, false)),
		"the loop may walk the other way round")

	shifted := []r3.Vec{{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 4, Y: 4}, {X: 0, Y: 4.5}}
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(shifted, 7, false)),
		"one corner differs")
	triangle := square[:3]
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(triangle, 7, false)),
		"the edge counts differ")
	// Four edges that are not the same four: a repeat in place of a missing edge.
	twice := []r3.Vec{{X: 0, Y: 0}, {X: 4, Y: 0}, {X: 0, Y: 0}, {X: 4, Y: 0}}
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, capFace(twice, 7, false)),
		"an edge is used up")
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, capFace(twice, 0, false), start))

	noLoops := &topo.Face{}
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, noLoops, start))
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, noLoops))
	hole := capFace(square, 7, false)
	hole.Loops[0].Outer = false
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, hole))
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, hole, start))

	// Fewer edges that all appear in the start loop still differ.
	open := capFace(square, 7, false)
	open.Loops[0].CoEdges = open.Loops[0].CoEdges[:3]
	require.False(t, loftcontact.MatchingHeldCapOuters(frame, start, open))

	// The comparison is in the start frame's plane, so a frame tilted off the
	// caps' axis sees the height difference as a shift.
	tilted, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0.6, 0.8))
	require.NoError(t, err)
	require.False(t, loftcontact.MatchingHeldCapOuters(tilted, start, capFace(square, 7, false)))
	require.True(t, loftcontact.MatchingHeldCapOuters(tilted, start, capFace(square, 0, false)))
}
