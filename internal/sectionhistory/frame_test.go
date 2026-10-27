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

		lines, ok := wholeLines(square(1, 2, 4, 6))
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

// TestWholeLinesRestatesOnlyWholeLines keeps a reversed whole line, as a
// forward one, and refuses a narrowed line and a circle.
func TestWholeLinesRestatesOnlyWholeLines(t *testing.T) {
	t.Parallel()
	p := square(0, 0, 1, 1)
	first := p.Outer.Segments[0].(sectionrecord.LineSeg)
	p.Outer.Segments[0] = sectionrecord.LineSeg{Start: first.End, End: first.Start, TStart: 1, TEnd: 0}
	got, ok := wholeLines(p)
	require.True(t, ok)
	require.Equal(t, first, got.outer[0])

	narrowed := square(0, 0, 1, 1)
	narrowed.Outer.Segments[1] = sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 1}, End: sectionrecord.Point2{U: 1, V: 2}, TStart: 0, TEnd: 0.5,
	}
	_, ok = wholeLines(narrowed)
	require.False(t, ok)

	circle := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1},
	}}}
	_, ok = wholeLines(circle)
	require.False(t, ok)
}

// TestCombineRefusesAWeldedGap arranges a square and a narrower box 2^-40 mm above its
// top edge. Sketch welds the two near-coincident lines into one shared span,
// which would join two separate regions into one; the gap charge refuses it.
func TestCombineRefusesAWeldedGap(t *testing.T) {
	t.Parallel()
	gap := math.Ldexp(1, -40)
	_, err := combine(t.Context(), square(0, 0, 10, 10), square(2, 10+gap, 8, 15),
		func(a, b bool) bool { return a || b })
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, msgRegion)
}
