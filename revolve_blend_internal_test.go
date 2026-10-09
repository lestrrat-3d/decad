package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file drives revolveBlendAxis, the gate a revolve junction blend runs on
// its rewritten meridian (docs/modify-reach-design.md §7), with meridians a
// line-line blend cannot itself produce: a blend's new piece lies in the
// triangle its corner and two tangent feet span, all three strictly off the
// axis, so the public tests reach the spindle refusal but no new axis contact.
// Each case was shown to fail by making revolveBlendAxis return the
// receiver's axis without re-resolving it.

// blendAxisReceiver is the annular ring u ∈ [0, 10], v ∈ [5, 15] revolved a
// full turn about the u axis.
func blendAxisReceiver(t *testing.T) revolvePayload {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 5, 10, 15)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	body, err := New().Revolve(s, s.Profiles()[0], axis, FullRevolution{})
	require.NoError(t, err)
	rp, ok := body.payload.(revolvePayload)
	require.True(t, ok)
	return rp
}

func polygonLoop(pts ...Point2) loopRecord {
	segs := make([]curveSegment, len(pts))
	for i, p := range pts {
		segs[i] = lineSeg{Start: p, End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return loopRecord{Segments: segs}
}

func TestRevolveBlendAxisGate(t *testing.T) {
	t.Parallel()
	rp := blendAxisReceiver(t)

	t.Run(`the receiver's own meridian re-resolves`, func(t *testing.T) {
		ax, err := revolveBlendAxis(t.Context(), rp, rp.profile, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.Equal(t, rp.ax.dU, ax.dU, `the region stays on the receiver axis's own side`)
		require.Equal(t, rp.ax.dV, ax.dV)
	})

	t.Run(`a meridian across the axis`, func(t *testing.T) {
		crossing := profileRecord{Outer: polygonLoop(
			Point2{U: 0, V: -1}, Point2{U: 10, V: -1}, Point2{U: 10, V: 4}, Point2{U: 0, V: 4},
		)}
		_, err := revolveBlendAxis(t.Context(), rp, crossing, freeform.NewFreeformWork())
		require.ErrorIs(t, err, ErrDegenerate)
		require.ErrorContains(t, err, `passes through the region`)
	})

	t.Run(`an arc touching the axis between its ends`, func(t *testing.T) {
		// The arc about (5, 2) of radius 2 runs counter-clockwise from (3, 2)
		// under the centre to (7, 2), touching the axis at (5, 0).
		touching := profileRecord{Outer: loopRecord{Segments: []curveSegment{
			arcSeg{Center: Point2{U: 5, V: 2}, Start: Point2{U: 3, V: 2}, End: Point2{U: 7, V: 2}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 7, V: 2}, End: Point2{U: 10, V: 2}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 10, V: 2}, End: Point2{U: 10, V: 4}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 10, V: 4}, End: Point2{U: 0, V: 4}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 0, V: 4}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
			lineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 3, V: 2}, TStart: 0, TEnd: 1},
		}}}
		_, err := revolveBlendAxis(t.Context(), rp, touching, freeform.NewFreeformWork())
		require.ErrorIs(t, err, ErrDegenerate)
		require.ErrorContains(t, err, `touches the revolve axis at an interior point`)
	})
}
