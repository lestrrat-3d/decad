package stackedbrep_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/stretchr/testify/require"
)

type (
	pt  = stackedbrep.Point2
	seg = stackedbrep.CurveSegment
)

// loopOf restates one loop of segments on a fresh engine's first call, or
// on b when it is given.
func loopOf(t *testing.T, b *stackedbrep.Engine, segs ...seg) (stackedbrep.Loop, error) {
	t.Helper()
	if b == nil {
		b = stackedbrep.NewEngine([]float64{0, 1})
	}
	return b.LoopOf(brepgeom.Profile{Outer: stackedbrep.LoopRecord{Segments: segs}}, proofbound.NewWorkBudget(t.Context()))
}

// arc is the arc about c from start to end, counter-clockwise or clockwise.
// A clockwise arc is recorded from end to start, so it reads its radius from
// end.
func arc(c, start, end pt, ccw bool) seg {
	if ccw {
		return stackedbrep.ArcSeg{Center: c, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return stackedbrep.ArcSeg{Center: c, Start: end, End: start, TStart: 1, TEnd: 0}
}

func line(from, to pt) seg {
	return stackedbrep.LineSeg{Start: from, End: to, TStart: 0, TEnd: 1}
}

// vertices lists each unit's From in loop order.
func vertices(l stackedbrep.Loop) []pt {
	var out []pt
	for _, u := range l.Units {
		out = append(out, u.From)
	}
	return out
}

// TestLoopOfCircleJunctions covers docs/shell-opening-design.md §4.3's
// junctions at one recorded point. The lens of the radius-5 circles about
// (0,0) and (6,0) meets at (3,±4): each crossing is the point both walked
// ends hold, keyed by the side of the line through the centres it lies on.
// Shown to fail: with circleSide reading zero for every point, the lens's
// second crossing reached the first's key and missed; with the walked-end
// check deleted, the arcs ending an ulp apart built; with the table's point
// check deleted, the second loop took (3,4) for its (3, 4+ulp); with the
// one-recorded-point rule for a line and a circle deleted, the oblique
// tangent missed; and with the side of an oblique line's crossing read
// across the line (the centre's side) instead of along it, the chord's two
// crossings keyed one vertex.
func TestLoopOfCircleJunctions(t *testing.T) {
	t.Parallel()
	o, c := pt{}, pt{U: 6}
	top, bottom := pt{U: 3, V: 4}, pt{U: 3, V: -4}
	lens := []seg{arc(o, bottom, top, true), arc(c, top, bottom, true)}

	t.Run("two circles cross at their recorded points", func(t *testing.T) {
		t.Parallel()
		b := stackedbrep.NewEngine([]float64{0, 1})
		l, err := loopOf(t, b, lens...)
		require.NoError(t, err)
		require.Equal(t, []pt{bottom, top}, vertices(l))
		require.Equal(t, 0.0, b.Allow(), "the recorded points carry no allowance")
		// The same loop again reaches both keys at the same points.
		again, err := loopOf(t, b, lens...)
		require.NoError(t, err)
		require.Equal(t, vertices(l), vertices(again))
	})
	t.Run("walked ends that differ miss", func(t *testing.T) {
		t.Parallel()
		off := pt{U: 3, V: math.Nextafter(4, 5)}
		_, err := loopOf(t, nil, arc(o, bottom, top, true), arc(c, off, bottom, true))
		require.ErrorIs(t, err, brepgeom.ErrStackedWallMiss)
	})
	t.Run("a second point under one key misses", func(t *testing.T) {
		t.Parallel()
		b := stackedbrep.NewEngine([]float64{0, 1})
		_, err := loopOf(t, b, lens...)
		require.NoError(t, err)
		// Both arcs read their radius from (3,−4), so they key the lens's
		// circles; they meet at (3, 4+ulp), on the side (3,4) holds.
		off := pt{U: 3, V: math.Nextafter(4, 5)}
		_, err = loopOf(t, b, arc(o, bottom, off, true), arc(c, off, bottom, false))
		require.ErrorIs(t, err, brepgeom.ErrStackedWallMiss)
	})
	t.Run("an oblique line touching a circle at its recorded point", func(t *testing.T) {
		t.Parallel()
		// The line through (7,1) and (3,4) is perpendicular to (3,4) and
		// touches the radius-5 circle there, where the arc to (−4,3) starts.
		l, err := loopOf(t, nil,
			line(pt{U: 7, V: 1}, top),
			arc(o, top, pt{U: -4, V: 3}, true),
			line(pt{U: -4, V: 3}, pt{U: -4, V: -5}),
			line(pt{U: -4, V: -5}, pt{U: 7, V: -5}),
			line(pt{U: 7, V: -5}, pt{U: 7, V: 1}))
		require.NoError(t, err)
		require.Equal(t, []pt{{U: 7, V: 1}, top, {U: -4, V: 3}, {U: -4, V: -5}, {U: 7, V: -5}}, vertices(l))
	})
	t.Run("an oblique chord's two crossings are two vertices", func(t *testing.T) {
		t.Parallel()
		far := pt{U: -4, V: -3}
		l, err := loopOf(t, nil, arc(o, far, top, true), line(top, far))
		require.NoError(t, err)
		require.Equal(t, []pt{far, top}, vertices(l))
	})
}
