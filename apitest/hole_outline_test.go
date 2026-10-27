package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §13's public-surface tests for OutlineHole, on the 20 x 20 x 10
// plate of hole_test.go.

// lHoleCorners is an L-shaped bore about the hole axis: an 8 x 8 square less the quadrant x, y > 0 past 4.
// It encloses 48 mm² and its reflex corner is at (0, 0).
func lHoleCorners() []decad.Point2 {
	return []decad.Point2{{U: -4, V: -4}, {U: 4, V: -4}, {U: 4, V: 0}, {U: 0, V: 0}, {U: 0, V: 4}, {U: -4, V: 4}}
}

func outlineHoleSpec(plate *decad.Body, corners []decad.Point2, depth decad.HoleDepth, degrees float64) decad.HoleSpec {
	return decad.HoleSpec{
		Face:  capFace(true, plate),
		At:    r3.Vec{X: 10, Y: 10, Z: 10},
		Depth: depth,
		Shape: decad.OutlineHole{Corners: corners, Angle: units.Degrees(degrees)},
	}
}

func TestHoleOutlineCutsAnLShapedBore(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, outlineHoleSpec(plate, lHoleCorners(), decad.ThroughHole{}, 0))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-48*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 12})
	probe := func(p r3.Vec) decad.PointLocation {
		loc, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.05))
		require.NoError(t, err)
		return loc
	}
	// The L opens to -U and -V of its reflex corner at the axis (10, 10) and reaches +U along the lower arm.
	require.Equal(t, decad.PointOutside, probe(r3.Vec{X: 13, Y: 8, Z: 5}), "in the lower arm")
	require.Equal(t, decad.PointOutside, probe(r3.Vec{X: 8, Y: 13, Z: 5}), "in the left arm")
	require.Equal(t, decad.PointInside, probe(r3.Vec{X: 13, Y: 12, Z: 5}), "in the removed quadrant")
	require.Equal(t, decad.PointInside, probe(r3.Vec{X: 5.5, Y: 10, Z: 5}), "past the left side")
}

func TestHoleOutlineKeepsTheShapeWhenListedClockwiseAndTurnsWithTheAngle(t *testing.T) {
	t.Parallel()
	corners := lHoleCorners()
	reversed := make([]decad.Point2, len(corners))
	for i, c := range corners {
		reversed[len(corners)-1-i] = c
	}
	probe := func(corners []decad.Point2, degrees float64, p r3.Vec) decad.PointLocation {
		_, plate := drillPlate(t)
		out, err := decad.Hole(t.Context(), plate, outlineHoleSpec(plate, corners, decad.ThroughHole{}, degrees))
		require.NoError(t, err)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-48*10))
		loc, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.05))
		require.NoError(t, err)
		return loc
	}
	require.Equal(t, decad.PointOutside, probe(reversed, 0, r3.Vec{X: 13, Y: 8, Z: 5}), "the same bore listed clockwise")
	require.Equal(t, decad.PointInside, probe(reversed, 0, r3.Vec{X: 13, Y: 12, Z: 5}))
	// Turned a quarter, (u, v) goes to (-v, u) about the axis: the removed quadrant moves to u < 0, v > 0.
	require.Equal(t, decad.PointOutside, probe(corners, 90, r3.Vec{X: 12, Y: 13, Z: 5}), "the turned bore")
	require.Equal(t, decad.PointInside, probe(corners, 90, r3.Vec{X: 8, Y: 13, Z: 5}), "the turned removed quadrant")
}

func TestHoleOutlineBlindAndIrregularTriangle(t *testing.T) {
	t.Parallel()
	tri := []decad.Point2{{U: -3, V: -2}, {U: 5, V: -1}, {U: 1, V: 4}}
	area := math.Abs((tri[1].U-tri[0].U)*(tri[2].V-tri[0].V)-(tri[2].U-tri[0].U)*(tri[1].V-tri[0].V)) / 2
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, outlineHoleSpec(plate, tri, decad.BlindHole{Depth: units.Millimeters(4)}, 0))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-area*4))
}

func TestHoleOutlineRefusals(t *testing.T) {
	t.Parallel()
	square := []decad.Point2{{U: -2, V: -2}, {U: 2, V: -2}, {U: 2, V: 2}, {U: -2, V: 2}}
	cases := map[string][]decad.Point2{
		"two corners":        square[:2],
		"a bow tie":          {{U: -2, V: -2}, {U: 2, V: 2}, {U: 2, V: -2}, {U: -2, V: 2}},
		"no area":            {{U: -2, V: 0}, {U: 0, V: 0}, {U: 2, V: 0}},
		"a repeat corner":    {{U: -2, V: -2}, {U: 2, V: -2}, {U: 2, V: -2}, {U: 2, V: 2}, {U: -2, V: 2}},
		"a corner on a side": {{U: -2, V: -2}, {U: 2, V: -2}, {U: 2, V: 2}, {U: 0, V: -2}, {U: -2, V: 2}},
		"a non-finite one":   {{U: -2, V: -2}, {U: math.NaN(), V: -2}, {U: 2, V: 2}, {U: -2, V: 2}},
	}
	for name, corners := range cases {
		doc, plate := drillPlate(t)
		_, err := decad.Hole(t.Context(), plate, outlineHoleSpec(plate, corners, decad.ThroughHole{}, 0))
		require.ErrorIs(t, err, decad.ErrDegenerate, name)
		require.Len(t, doc.Bodies(), 1, "%s leaves the plate alone", name)
	}
	_, plate := drillPlate(t)
	spec := outlineHoleSpec(plate, square, decad.ThroughHole{}, 0)
	spec.Diameter = units.Millimeters(4)
	_, err := decad.Hole(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a shaped hole takes no diameter")
	spec = outlineHoleSpec(plate, square, decad.ThroughHole{}, 0)
	spec.Shape = decad.OutlineHole{Corners: square, Angle: units.Millimeters(1)}
	_, err = decad.Hole(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrUnitKind)
}
