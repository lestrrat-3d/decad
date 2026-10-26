package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/surface-design.md §6.2's tests for Table J's circular
// rows: a Circle3 or Arc3 free edge joins another when a builder proved both
// curve bounds zero and the held circles and angular ranges are bit-identical.

// circleSketchAt builds a solved origin-centred circle of radius r on the XY
// plane offset by z millimetres.
func circleSketchAt(t *testing.T, r, z float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	plane := w.XY()
	if z != 0 {
		var err error
		plane, err = w.CreateOffsetPlane(w.XY(), z)
		require.NoError(t, err)
	}
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.CreateCircle(c, r)
	s.Fix(c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// TestStitchWeldsAZeroBoundCircularRim welds a surface-extruded cylinder wall
// to a disk patch of the same circle. The wall's bottom rim and the disk's
// boundary are whole circles with a proven zero curve bound, so the rim welds
// and the wall's top rim is the one free edge left.
func TestStitchWeldsAZeroBoundCircularRim(t *testing.T) {
	t.Parallel()
	const r, h = 10.0, 15.0
	doc := decad.New()
	s, p := circleSketchAt(t, r, 0)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(h), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	disk, err := decad.PatchProfile(t.Context(), doc, s, p)
	require.NoError(t, err)

	sheet, err := decad.Stitch(t.Context(), wall, disk)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())

	free, err := decad.Edges(decad.Free()).Exactly(1).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 1)
	require.Equal(t, 15.0, free[0].Start().Position().Value.Z)
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(2*math.Pi*r*h+math.Pi*r*r))
}

// TestStitchLeavesANonzeroBoundCircularRimFree keeps a wall rim whose curve
// bound is nonzero apart from an exact disk at the same coordinates: the
// wall is built in inches, so its top rim vertices carry a bound and held
// equality proves nothing. The vertex bound is what keeps the pair apart here;
// no builder yet yields zero-bound vertices with a nonzero curve bound.
func TestStitchLeavesANonzeroBoundCircularRimFree(t *testing.T) {
	t.Parallel()
	const r = 10.0
	doc := decad.New()
	s, p := circleSketchAt(t, r, 0)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Inches(5), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	ts, tp := circleSketchAt(t, r, 127)
	top, err := decad.PatchProfile(t.Context(), doc, ts, tp)
	require.NoError(t, err)

	sheet, err := decad.Stitch(t.Context(), wall, top)
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).Exactly(3).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 3)
}

// halfDiskSketch holds the upper and lower half disks of one circle: the two
// arcs share both endpoints and the circle, and each is the other's
// complement. The diameter line is common.
func halfDiskSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	c, a, b := fixed(0, 0), fixed(10, 0), fixed(-10, 0)
	s.CreateArc(c, a, b)
	s.CreateArc(c, b, a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 2)
	return s
}

// TestStitchLeavesComplementaryArcsFree stitches the two half disks. Their
// diameters weld; their arcs share endpoints and circle but are complementary,
// so both stay free.
func TestStitchLeavesComplementaryArcsFree(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s := halfDiskSketch(t)
	up, err := decad.PatchProfile(t.Context(), doc, s, s.Profiles()[0])
	require.NoError(t, err)
	down, err := decad.PatchProfile(t.Context(), doc, s, s.Profiles()[1])
	require.NoError(t, err)

	sheet, err := decad.Stitch(t.Context(), up, down)
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 2)
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(math.Pi*100))
}
