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

// This file is agent-docs/surface-design.md §6.2's tests for Table J's circular
// rows: a Circle3 or Arc3 free edge joins another when a builder proved both
// curve bounds zero and the held circles and angular ranges are bit-identical.

const stitchCircleRadius = 10.0

// circleSketchAt builds a solved origin-centred circle of stitchCircleRadius on the XY
// plane offset by z millimetres.
func circleSketchAt(t *testing.T, z float64) (*sketch.Sketch, *sketch.Profile) {
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
	s.CreateCircle(c, stitchCircleRadius)
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
	s, p := circleSketchAt(t, 0)
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
	doc := decad.New()
	s, p := circleSketchAt(t, 0)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Inches(5), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	ts, tp := circleSketchAt(t, 127)
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

// flippedCircleSketch is circleSketchAt(t, 0) on a plane whose normal is -Z.
func flippedCircleSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.CreateCircle(c, stitchCircleRadius)
	s.Fix(c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// stitchedNormals reads the wall's and the disk's normals of a welded cylinder
// wall plus bottom disk: a point on the wall at mid height and the disk centre.
func stitchedNormals(t *testing.T, sheet *decad.Body) (wall, disk r3.Vec) {
	t.Helper()
	planar, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(sheet)
	require.NoError(t, err)
	dm, err := planar[0].NormalAt(r3.NewVec(0, 0, 0))
	require.NoError(t, err)
	disk = dm.Value
	curved, err := decad.Faces(decad.Cylindrical()).Exactly(1).SelectFaces(sheet)
	require.NoError(t, err)
	wm, err := curved[0].NormalAt(r3.NewVec(10, 0, 7.5))
	require.NoError(t, err)
	wall = wm.Value
	return wall, disk
}

// TestStitchedCylinderRimKeepsConsistentNormals welds a disk onto the wall and
// checks the sheet's normals point to one side: the wall outward and the disk
// down, away from the wall's material side.
func TestStitchedCylinderRimKeepsConsistentNormals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := circleSketchAt(t, 0)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	disk, err := decad.PatchProfile(t.Context(), doc, s, p)
	require.NoError(t, err)
	sheet, err := decad.Stitch(t.Context(), wall, disk)
	require.NoError(t, err)
	w, d := stitchedNormals(t, sheet)
	require.InDelta(t, 1, w.X, 1e-9)
	require.InDelta(t, -1, d.Z, 1e-9)
}

// TestStitchDoesNotWeldAnOppositeAxisCircle stitches a wall on the +Z circle
// with a disk patched at normal -Z. Welding two whole circles with opposite
// axes would leave their faces with inconsistent normals, so the rims stay
// unwelded; the two rims then pinch at one vertex, which Table R row R7
// refuses.
func TestStitchDoesNotWeldAnOppositeAxisCircle(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := circleSketchAt(t, 0)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	fs, fp := flippedCircleSketch(t)
	disk, err := decad.PatchProfile(t.Context(), doc, fs, fp)
	require.NoError(t, err)
	_, err = decad.Stitch(t.Context(), wall, disk)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

// halfDiskWallAndPatch extrudes the upper half disk of radius 10 as a surface
// wall 15 mm Along and patches a half disk onto its bottom rim; flipped builds
// that patch on a -Z plane. In the flipped plane, lower picks the lower local
// half, which is the same world region as the wall with the arc's axis reversed;
// the upper local half is the complementary world half.
func halfDiskWallAndPatch(t *testing.T, flipped, lower bool) (*decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	build := func(w *sketch.World, plane *sketch.Plane, lower bool) (*sketch.Sketch, *sketch.Profile) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		fixed := func(u, v float64) *sketch.Point {
			p := s.CreatePoint(u, v)
			s.Fix(p)
			return p
		}
		c, a, b := fixed(0, 0), fixed(10, 0), fixed(-10, 0)
		if lower {
			s.CreateArc(c, b, a)
		} else {
			s.CreateArc(c, a, b)
		}
		s.CreateLine(a, b)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		require.Len(t, s.Profiles(), 1)
		return s, s.Profiles()[0]
	}
	w := sketch.NewWorld()
	s, p := build(w, w.XY(), false)
	wall, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(15), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	pw := sketch.NewWorld()
	plane := pw.XY()
	if flipped {
		frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
		require.NoError(t, err)
		plane, err = pw.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
	}
	ps, pp := build(pw, plane, lower)
	patch, err := decad.PatchProfile(t.Context(), doc, ps, pp)
	require.NoError(t, err)
	return wall, patch
}

// TestStitchWeldsAZeroBoundArcRim welds a half-disk wall to its half-disk patch.
// The wall's bottom arc and the patch's arc meet start to start with equal axes
// and the diameters weld too, leaving the wall's top arc and top diameter free.
func TestStitchWeldsAZeroBoundArcRim(t *testing.T) {
	t.Parallel()
	wall, patch := halfDiskWallAndPatch(t, false, false)
	sheet, err := decad.Stitch(t.Context(), wall, patch)
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(sheet)
	require.NoError(t, err)
	for _, e := range free {
		require.Equal(t, 15.0, e.Start().Position().Value.Z)
	}
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters((math.Pi*10+20)*15+math.Pi*50))
}

// TestStitchWeldsAnOppositeAxisArcRim welds the same wall to a patch whose arc
// runs the other way with the opposite axis: the arcs meet start to end.
func TestStitchWeldsAnOppositeAxisArcRim(t *testing.T) {
	t.Parallel()
	wall, patch := halfDiskWallAndPatch(t, true, true)
	sheet, err := decad.Stitch(t.Context(), wall, patch)
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).Exactly(2).SelectEdges(sheet)
	require.NoError(t, err)
	for _, e := range free {
		require.Equal(t, 15.0, e.Start().Position().Value.Z)
	}
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters((math.Pi*10+20)*15+math.Pi*50))
}

// TestStitchLeavesAComplementaryOppositeAxisArcFree patches the lower world
// half disk on a -Z plane against the upper half-disk wall. The arcs meet start
// to start with opposite axes: the same circle and endpoints, but the
// complementary arc. Only the diameters weld, so four arcs and lines stay free.
func TestStitchLeavesAComplementaryOppositeAxisArcFree(t *testing.T) {
	t.Parallel()
	wall, patch := halfDiskWallAndPatch(t, true, false)
	sheet, err := decad.Stitch(t.Context(), wall, patch)
	require.NoError(t, err)
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 4)
}
