package modifyselect_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMatchCorner(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 10}
	// Loop 0 is a closed circle (one closed walk); loop 1 has three corners;
	// loop 2 is one open walk, whose single junction is still a corner.
	closed := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{StartU: 7, StartV: 8, Closed: true}}
	loops := []survey2d.CornerLoop{
		{Walks: []survey2d.SideWalk{closed}},
		{Walks: []survey2d.SideWalk{walkAt(0, 0), walkAt(1, 2), walkAt(4, 5)}},
		{Walks: []survey2d.SideWalk{walkAt(9, 9)}},
	}
	lateral := func(u, v float64) *topo.Edge {
		return &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(u, v, 0), End: vertexAt(u, v, 10)}
	}
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	t.Run("lateral edge at a corner", func(t *testing.T) {
		t.Parallel()
		li, ci, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(1, 2))
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]int{1, 1}, [2]int{li, ci})
	})
	t.Run("first corner of a later loop", func(t *testing.T) {
		t.Parallel()
		li, ci, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(0, 0))
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]int{1, 0}, [2]int{li, ci})
	})
	t.Run("last corner", func(t *testing.T) {
		t.Parallel()
		li, ci, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(4, 5))
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]int{1, 2}, [2]int{li, ci})
	})
	t.Run("single open walk", func(t *testing.T) {
		t.Parallel()
		li, ci, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(9, 9))
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, [2]int{2, 0}, [2]int{li, ci})
	})
	t.Run("reversed edge", func(t *testing.T) {
		t.Parallel()
		e := lateral(1, 2)
		e.Start, e.End = e.End, e.Start
		_, ci, found, err := modifyselect.MatchCorner(budget(), pp, loops, e)
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, 1, ci)
	})
	t.Run("within the endpoint tolerance", func(t *testing.T) {
		t.Parallel()
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(1+5e-7, 2))
		require.NoError(t, err)
		require.True(t, found)
	})
	t.Run("past the endpoint tolerance", func(t *testing.T) {
		t.Parallel()
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(1+5e-6, 2))
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("a closed loop has no corner", func(t *testing.T) {
		t.Parallel()
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, lateral(7, 8))
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("cap edge", func(t *testing.T) {
		t.Parallel()
		e := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(1, 2, 0), End: vertexAt(4, 5, 0)}
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, e)
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("edge off the caps' levels", func(t *testing.T) {
		t.Parallel()
		e := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertexAt(1, 2, 0), End: vertexAt(1, 2, 5)}
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, e)
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("curved edge", func(t *testing.T) {
		t.Parallel()
		e := lateral(1, 2)
		e.Curve = surfacegeom.Circle3{}
		_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, e)
		require.NoError(t, err)
		require.False(t, found)
	})
	t.Run("missing vertices", func(t *testing.T) {
		t.Parallel()
		noStart, noEnd := lateral(1, 2), lateral(1, 2)
		noStart.Start, noEnd.End = nil, nil
		for _, e := range []*topo.Edge{noStart, noEnd} {
			_, _, found, err := modifyselect.MatchCorner(budget(), pp, loops, e)
			require.NoError(t, err)
			require.False(t, found)
		}
	})
	// Matching the last corner takes five budget steps: one per loop reached
	// (two) and one per corner of the second loop (three). A budget that fails at any of
	// them fails the match; the sixth call never comes.
	t.Run("budget failures at every step", func(t *testing.T) {
		t.Parallel()
		boom := errors.New("budget spent")
		for failAt := 1; failAt <= 5; failAt++ {
			steps := 0
			b := &proofbound.WorkBudget{StepFn: func() error {
				steps++
				if steps == failAt {
					return boom
				}
				return nil
			}}
			_, _, found, err := modifyselect.MatchCorner(b, pp, loops, lateral(4, 5))
			require.ErrorIs(t, err, boom, "failing step %d", failAt)
			require.False(t, found)
		}
		steps := 0
		b := &proofbound.WorkBudget{StepFn: func() error { steps++; return nil }}
		_, _, found, err := modifyselect.MatchCorner(b, pp, loops, lateral(4, 5))
		require.NoError(t, err)
		require.True(t, found)
		require.Equal(t, 5, steps)
	})
}
