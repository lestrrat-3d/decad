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

// This file is agent-docs/hole-design.md §10's public-surface tests for the
// polygonal Shape of Hole, on the 20 x 20 x 10 plate of hole_test.go.

func polygonHoleSpec(plate *decad.Body, sides int, depth decad.HoleDepth, degrees float64) decad.HoleSpec {
	const flats = 6.0 // the width across flats of every polygon here
	return decad.HoleSpec{
		Face:  capFace(true, plate),
		At:    r3.Vec{X: 10, Y: 10, Z: 10},
		Depth: depth,
		Shape: decad.PolygonHole{Sides: sides, AcrossFlats: units.Millimeters(flats), Angle: units.Degrees(degrees)},
	}
}

// holePolygonArea is the area of a regular polygon of n sides a across flats apart.
func holePolygonArea(n int, flats float64) float64 {
	return float64(n) * flats * flats / 4 * math.Tan(math.Pi/float64(n))
}

func TestHolePolygonHexagonThrough(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, polygonHoleSpec(plate, 6, decad.ThroughHole{}, 0))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == out)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-holePolygonArea(6, 6)*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 12})
}

func TestHolePolygonFlatsFaceUAndTurnsWithTheAngle(t *testing.T) {
	t.Parallel()
	// Across flats 6 and sides 6: the circumradius is 3.464. With no turn a flat faces +U, so the
	// hole reaches x = 13 along U and y = 13.464 along V; a turn of 30 degrees swaps them.
	probe := func(degrees float64, p r3.Vec) decad.PointLocation {
		_, plate := drillPlate(t)
		out, err := decad.Hole(t.Context(), plate, polygonHoleSpec(plate, 6, decad.ThroughHole{}, degrees))
		require.NoError(t, err)
		loc, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.05))
		require.NoError(t, err)
		return loc
	}
	require.Equal(t, decad.PointOutside, probe(0, r3.Vec{X: 12.9, Y: 10, Z: 5}), "inside the hole at the flat")
	require.Equal(t, decad.PointInside, probe(0, r3.Vec{X: 13.1, Y: 10, Z: 5}), "past the flat is material")
	require.Equal(t, decad.PointOutside, probe(0, r3.Vec{X: 10, Y: 13.4, Z: 5}), "inside the hole at a corner")
	require.Equal(t, decad.PointInside, probe(0, r3.Vec{X: 10, Y: 13.6, Z: 5}))
	require.Equal(t, decad.PointOutside, probe(30, r3.Vec{X: 13.4, Y: 10, Z: 5}), "turned, a corner faces U")
	require.Equal(t, decad.PointInside, probe(30, r3.Vec{X: 13.6, Y: 10, Z: 5}))
	require.Equal(t, decad.PointInside, probe(30, r3.Vec{X: 10, Y: 13.1, Z: 5}), "turned, a flat faces V")
}

func TestHolePolygonOtherSidesAndBlind(t *testing.T) {
	t.Parallel()
	for _, n := range []int{3, 5, 8, 12} {
		_, plate := drillPlate(t)
		out, err := decad.Hole(t.Context(), plate, polygonHoleSpec(plate, n, decad.BlindHole{Depth: units.Millimeters(4)}, 15))
		require.NoError(t, err, "%d sides", n)
		decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-holePolygonArea(n, 6)*4))
	}
}

func TestHolePolygonRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		edit func(*decad.HoleSpec)
		want error
	}{
		{"two sides", func(s *decad.HoleSpec) {
			s.Shape = decad.PolygonHole{Sides: 2, AcrossFlats: units.Millimeters(6), Angle: units.Degrees(0)}
		}, decad.ErrDegenerate},
		{"too many sides", func(s *decad.HoleSpec) {
			s.Shape = decad.PolygonHole{Sides: 65, AcrossFlats: units.Millimeters(6), Angle: units.Degrees(0)}
		}, decad.ErrDegenerate},
		{"zero width", func(s *decad.HoleSpec) {
			s.Shape = decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(0), Angle: units.Degrees(0)}
		}, decad.ErrDegenerate},
		{"angle of a length", func(s *decad.HoleSpec) {
			s.Shape = decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(6), Angle: units.Millimeters(1)}
		}, decad.ErrUnitKind},
		{"head beside a shape", func(s *decad.HoleSpec) {
			s.Head = decad.Counterbore{Diameter: units.Millimeters(8), Depth: units.Millimeters(1)}
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := drillPlate(t)
			spec := polygonHoleSpec(plate, 6, decad.BlindHole{Depth: units.Millimeters(5)}, 0)
			tc.edit(&spec)
			_, err := decad.Hole(t.Context(), plate, spec)
			require.ErrorIs(t, err, tc.want)
			require.Len(t, doc.Bodies(), 1)
			require.True(t, doc.Bodies()[0] == plate, "a refused hole leaves the plate live and alone")
		})
	}
}
