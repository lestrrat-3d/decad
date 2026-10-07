package prismcells

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// sharedArcScene arranges a disk of radius 20 (operand A, its circle CCW)
// beside a region B outside it whose boundary runs along the disk's own
// circle from 90° to 170° as an arc, then out and around through
// (−30, ·), (−30, 30) and (0, 30) back to the arc's start. sketch names the
// shared span under B's arc and withdraws it from A's circle. B's outline is
// built clockwise, so its authored counter-clockwise walk runs every B
// entity backwards. It returns the cells and which one is the disk's.
func sharedArcScene(t *testing.T) ([]*sketch.Profile, map[sketch.Entity]Origin, int, *sketch.Arc, *sketch.Circle) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	a170 := 170 * math.Pi / 180
	centre := fixed(0, 0)
	top := fixed(0, 20)
	p170 := fixed(20*math.Cos(a170), 20*math.Sin(a170))
	r := fixed(-30, 20*math.Sin(a170))
	u := fixed(-30, 30)
	v := fixed(0, 30)
	tags := map[sketch.Entity]Origin{}
	circle := s.CreateCircle(centre, 20)
	tags[circle] = Origin{Hole: -1}
	arc := s.CreateArc(centre, top, p170)
	tags[arc] = Origin{IsB: true, Hole: -1, AuthoredReversed: true}
	for _, seg := range [][2]*sketch.Point{{p170, r}, {r, u}, {u, v}, {v, top}} {
		tags[s.CreateLine(seg[0], seg[1])] = Origin{IsB: true, Hole: -1, AuthoredReversed: true}
	}
	_, err = s.Solve(context.Background())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 2)
	disk := -1
	for i, p := range profiles {
		require.True(t, p.Valid)
		for _, e := range p.Outer {
			if e.Entity == sketch.Entity(circle) {
				disk = i
			}
		}
	}
	require.NotEqual(t, -1, disk)
	return profiles, tags, disk, arc, circle
}

// TestCoincidentEdgesReadsASharedArc pins the reading's premise on the
// shared-arc scene: one span, named under B's arc and withdrawn from A's
// circle, whose one edge both cells walk, and a gap inside sketch's own
// identity band (1e-12 of the radius): the arc and the circle share one
// recorded centre, and the arc's endpoints sit on radius 20 to within their
// own cos/sin rounding, which the gap reads.
func TestCoincidentEdgesReadsASharedArc(t *testing.T) {
	t.Parallel()
	profiles, tags, _, arc, circle := sharedArcScene(t)
	reading, ok, err := CoincidentEdges(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, reading.Spans, 1)
	require.Equal(t, sketch.Entity(arc), reading.Spans[0].Named)
	require.Equal(t, sketch.Entity(circle), reading.Spans[0].Losing)
	require.Len(t, reading.Edges, 1)
	require.True(t, reading.Partners(arc, circle))
	require.Less(t, reading.Gap(), 20e-12)
}

// TestClassifyReadsASharedArc is the flood the coincident reading stops.
// The cell outside the disk has no edge of A's own: its one A boundary is
// the shared span, emitted under B's arc. Propagating A's membership across
// that B edge from the disk would put the outside cell inside A, and an
// Intersect would publish it; read from A's circle instead, it is A's void.
// ClassifySplit, which reads the target side alone, must agree.
//
// Shown to fail with Classify's and ClassifySplit's coincident reading
// deleted (the outside cell read inside A).
func TestClassifyReadsASharedArc(t *testing.T) {
	t.Parallel()
	profiles, tags, disk, _, _ := sharedArcScene(t)
	outside := 1 - disk
	matterA, matterB, resolved, err := Classify(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, resolved)
	require.True(t, matterA[disk])
	require.False(t, matterB[disk])
	require.False(t, matterA[outside])
	require.True(t, matterB[outside])

	target, err := ClassifySplit(proofbound.NewWorkBudget(t.Context()), tags, profiles)
	require.NoError(t, err)
	require.True(t, target[disk])
	require.False(t, target[outside])
}
