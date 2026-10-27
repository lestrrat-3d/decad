package sectionhistory

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/classbgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func square(u0, v0, u1, v1 float64) momentinput.Profile {
	a, b := sectionrecord.Point2{U: u0, V: v0}, sectionrecord.Point2{U: u1, V: v0}
	c, d := sectionrecord.Point2{U: u1, V: v1}, sectionrecord.Point2{U: u0, V: v1}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}
}

// TestPermutationMapsBackToTheSamePoint checks every candidate frame spans the
// requested plane, that a candidate-local point maps to the same world point
// in the requested frame, and that a counter-clockwise loop stays
// counter-clockwise after mapping back.
func TestPermutationMapsBackToTheSamePoint(t *testing.T) {
	t.Parallel()
	plane, err := r3.NewFrame(r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	reflections := 0
	for _, perm := range permutations {
		f, ok := perm.frame(plane)
		require.True(t, ok)
		require.Equal(t, plane.Origin(), f.Origin())
		if perm.reflects() {
			reflections++
			require.Equal(t, plane.N().Scale(-1), f.N())
		} else {
			require.Equal(t, plane.N(), f.N())
		}
		q := sectionrecord.Point2{U: 3, V: -7}
		require.Equal(t, f.ToWorldUV(q.U, q.V), plane.ToWorldUV(perm.point(q).U, perm.point(q).V))

		lines, ok := wholeCurves(square(1, 2, 4, 6))
		require.True(t, ok)
		mapped := perm.mapBack(lines)
		area, err := classbgeom.LoopSignedArea(mapped.Outer)
		require.NoError(t, err)
		require.Equal(t, 12.0, area)
		for i, seg := range mapped.Outer.Segments {
			next := mapped.Outer.Segments[(i+1)%len(mapped.Outer.Segments)]
			require.Equal(t, seg.(sectionrecord.LineSeg).End, next.(sectionrecord.LineSeg).Start)
		}
	}
	require.Equal(t, 4, reflections)
}

// TestWholeCurvesRestatesOnlyWholeSegments keeps a reversed whole line, as a
// forward one, keeps a whole arc and circle, and refuses a narrowed line, a
// narrowed arc and a narrowed circle.
func TestWholeCurvesRestatesOnlyWholeSegments(t *testing.T) {
	t.Parallel()
	p := square(0, 0, 1, 1)
	first := p.Outer.Segments[0].(sectionrecord.LineSeg)
	p.Outer.Segments[0] = sectionrecord.LineSeg{Start: first.End, End: first.Start, TStart: 1, TEnd: 0}
	got, ok := wholeCurves(p)
	require.True(t, ok)
	require.Equal(t, first, got.Outer.Segments[0])

	narrowed := square(0, 0, 1, 1)
	narrowed.Outer.Segments[1] = sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{U: 1, V: 2}, TStart: 0, TEnd: 0.5,
	}
	_, ok = wholeCurves(narrowed)
	require.False(t, ok)

	circle := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1},
	}}}
	got, ok = wholeCurves(circle)
	require.True(t, ok)
	require.Equal(t, circle, got)
	circle.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0.25, TEnd: 1}
	_, ok = wholeCurves(circle)
	require.False(t, ok)
	// A whole circle is a loop on its own; beside another segment it names no
	// closed walk.
	crowded := square(0, 0, 1, 1)
	crowded.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1}
	_, ok = wholeCurves(crowded)
	require.False(t, ok)

	arc := halfDisk()
	got, ok = wholeCurves(arc)
	require.True(t, ok)
	require.Equal(t, arc, got)
	narrowedArc := halfDisk()
	a := narrowedArc.Outer.Segments[0].(sectionrecord.ArcSeg)
	a.TEnd = 0.5
	narrowedArc.Outer.Segments[0] = a
	_, ok = wholeCurves(narrowedArc)
	require.False(t, ok)
}

// halfDisk is the upper half of the unit disk: the arc from (1, 0) to (-1, 0)
// then the diameter back, counter-clockwise.
func halfDisk() momentinput.Profile {
	right, left := sectionrecord.Point2{U: 1}, sectionrecord.Point2{U: -1}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Start: right, End: left, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: left, End: right, TStart: 0, TEnd: 1},
	}}}
}

// TestPermutationMapsArcsAndCircles maps a half disk and a circle through
// every candidate. Each mapped arc stays a counter-clockwise arc whose walked
// ends meet the mapped diameter, and the loop's winding is kept.
func TestPermutationMapsArcsAndCircles(t *testing.T) {
	t.Parallel()
	plane, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	for _, perm := range permutations {
		_, ok := perm.frame(plane)
		require.True(t, ok)
		mapped := perm.mapBack(halfDisk())
		area, err := classbgeom.LoopSignedArea(mapped.Outer)
		require.NoError(t, err)
		require.InDelta(t, math.Pi/2, area, 1e-12)
		var arc sectionrecord.ArcSeg
		var line sectionrecord.LineSeg
		for _, seg := range mapped.Outer.Segments {
			switch seg := seg.(type) {
			case sectionrecord.ArcSeg:
				arc = seg
			case sectionrecord.LineSeg:
				line = seg
			}
		}
		// The arc runs counter-clockwise from Start to End, so it bulges to the
		// left of the chord Start -> End.
		walkStart, walkEnd := arc.Start, arc.End
		if arc.TStart == 1 {
			walkStart, walkEnd = walkEnd, walkStart
		}
		require.Equal(t, walkEnd, line.Start)
		require.Equal(t, line.End, walkStart)

		ring := perm.mapBack(momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 2, V: 3}, Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1},
		}}})
		circle := ring.Outer.Segments[0].(sectionrecord.CircleSeg)
		require.True(t, circle.CCW)
		require.Equal(t, perm.point(sectionrecord.Point2{U: 2, V: 3}), circle.Center)
	}
}

// TestCombineRefusesAWeldedGap arranges a square and a narrower box 2^-40 mm above its
// top edge. Sketch welds the two near-coincident lines into one shared span,
// which would join two separate regions into one; the gap charge refuses it.
func TestCombineRefusesAWeldedGap(t *testing.T) {
	t.Parallel()
	gap := math.Ldexp(1, -40)
	_, err := combine(t.Context(), []momentinput.Profile{square(0, 0, 10, 10)}, []momentinput.Profile{square(2, 10+gap, 8, 15)},
		func(a, b bool) bool { return a || b })
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, msgRegion)
}

// TestPermutationFrameRefusesMovedAxes uses a tilted requested frame. The
// identity candidate is the frame itself; a rebuilt candidate whose held axes
// r3.NewFrame moves is refused.
func TestPermutationFrameRefusesMovedAxes(t *testing.T) {
	t.Parallel()
	plane, err := r3.NewFrame(r3.Vec{X: 1, Y: -2, Z: 4}, r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 3, Z: -1})
	require.NoError(t, err)
	f, ok := permutations[0].frame(plane)
	require.True(t, ok)
	require.Equal(t, plane, f)
	refused := 0
	for _, perm := range permutations[1:] {
		if _, ok := perm.frame(plane); !ok {
			refused++
		}
	}
	require.Positive(t, refused)
}
