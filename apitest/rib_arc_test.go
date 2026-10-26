package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/rib-design.md §5's public-surface tests for RibArc,
// on the pocketed block of rib_test.go: a floor at z = 5, a rib sketch plane at
// z = 4 and ribs extruded 11 mm, so each rib stands 10 mm above the floor.

// ribArcSketch returns a sketch on the plane z = 4 with the arc about (cu, cv)
// from (su, sv) counter-clockwise to (eu, ev).
func ribArcSketch(t *testing.T, cu, cv, su, sv, eu, ev float64) (*sketch.Sketch, *sketch.Arc) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 4)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	centre, start, end := s.CreatePoint(cu, cv), s.CreatePoint(su, sv), s.CreatePoint(eu, ev)
	s.Fix(centre)
	s.Fix(start)
	s.Fix(end)
	arc := s.CreateArc(centre, start, end)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, arc
}

func TestRibArcStandsOnThePocketFloor(t *testing.T) {
	t.Parallel()
	doc, part := ribPart(t)
	s, arc := ribArcSketch(t, 20, 20, 30, 20, 10, 20) // a half circle of radius 10
	operationCount := len(doc.OperationHistory().Operations())
	out, err := decad.RibArc(t.Context(), part, s, arc, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the part and the rib tool leave the document")
	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, operationCount+1)
	require.Equal(t, decad.OperationRib, operations[len(operations)-1].Kind())
	// A half circle of radius 10 thickened by 2 is π·10·2 mm² and stands 10 mm.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+math.Pi*10*2*10), decadtest.WithinRel(units.Scalar(1e-3)))
	// The arc sweeps counter-clockwise from (30, 20) through (20, 30) to (10, 20):
	// its top, at y = 30, is rib, and the opposite point on the circle is not.
	rib, err := out.LocatePoint(t.Context(), r3.NewVec(20, 30, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, rib, "on the arc's side")
	gap, err := out.LocatePoint(t.Context(), r3.NewVec(20, 10, 10), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, gap, "on the other half of the circle")
}

func TestRibArcOverrunLengthensEachEndByArcLength(t *testing.T) {
	t.Parallel()
	_, part := ribPart(t)
	// A quarter circle of radius 10 from angle 0, lengthened 5 mm each end: a
	// 5π + 10 mm run, thickened by 2 and standing 10 mm.
	s, arc := ribArcSketch(t, 20, 20, 30, 20, 20, 30)
	out, err := decad.RibArc(t.Context(), part, s, arc, units.Millimeters(2),
		decad.Distance{D: units.Millimeters(11), Dir: decad.Along}, decad.WithRibOverrun(units.Millimeters(5)))
	require.NoError(t, err)
	length := math.Pi/2*10 + 2*5
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(ribPartVolume+length*2*10), decadtest.WithinRel(units.Scalar(1e-3)))
}

func TestRibArcRefusals(t *testing.T) {
	t.Parallel()
	extent := decad.Distance{D: units.Millimeters(11), Dir: decad.Along}
	cases := []struct {
		name      string
		thickness units.Value
		overrun   *units.Value
		want      error
	}{
		{"zero thickness", units.Millimeters(0), nil, decad.ErrDegenerate},
		{"negative thickness", units.Millimeters(-1), nil, decad.ErrNegativeMagnitude},
		{"thickness as an angle", units.Degrees(2), nil, decad.ErrUnitKind},
		{"a rib thicker than the arc's radius", units.Millimeters(25), nil, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, part := ribPart(t)
			s, arc := ribArcSketch(t, 20, 20, 30, 20, 10, 20)
			_, err := decad.RibArc(t.Context(), part, s, arc, tc.thickness, extent)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{part}, doc.Bodies(), "a refused rib leaves the part live and alone")
		})
	}
	t.Run("an arc of another sketch", func(t *testing.T) {
		t.Parallel()
		doc, part := ribPart(t)
		s, _ := ribArcSketch(t, 20, 20, 30, 20, 10, 20)
		_, foreign := ribArcSketch(t, 20, 20, 30, 20, 10, 20)
		_, err := decad.RibArc(t.Context(), part, s, foreign, units.Millimeters(2), extent)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{part}, doc.Bodies())
	})
	t.Run("a nil arc", func(t *testing.T) {
		t.Parallel()
		_, part := ribPart(t)
		s, _ := ribArcSketch(t, 20, 20, 30, 20, 10, 20)
		_, err := decad.RibArc(t.Context(), part, s, nil, units.Millimeters(2), extent)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("an arc outside the body", func(t *testing.T) {
		t.Parallel()
		_, part := ribPart(t)
		s, arc := ribArcSketch(t, 80, 80, 90, 80, 70, 80)
		_, err := decad.RibArc(t.Context(), part, s, arc, units.Millimeters(2), extent)
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}
