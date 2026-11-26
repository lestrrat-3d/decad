package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/hole-design.md §5's public-surface tests for the
// rectangular Shape of Hole, on the 20 x 20 x 10 plate of hole_test.go.

// plateShapedHole is a hole of the given shape and depth at the top face of the
// 20 x 20 x 10 plate of hole_test.go, centred on (10, 10).
func plateShapedHole(plate *decad.Body, shape decad.HoleShape, depth decad.HoleDepth) *decad.HoleShapedCommand {
	return decad.HoleShaped(plate, capFace(true, plate), r3.Vec{X: 10, Y: 10, Z: 10}, shape, depth)
}

func rectangularHole(plate *decad.Body, depth decad.HoleDepth, degrees float64) *decad.HoleShapedCommand {
	return plateShapedHole(plate, decad.RectangularHole{
		Width:  units.Millimeters(6),
		Height: units.Millimeters(4),
		Angle:  units.Degrees(degrees),
	}, depth)
}

func TestHoleRectangularThrough(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := rectangularHole(plate, decad.ThroughHole{}, 0).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-6*4*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 10})
}

func TestHoleRectangularBlind(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := rectangularHole(plate, decad.BlindHole{Depth: units.Millimeters(6)}, 0).Do(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-6*4*6))
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 11})
}

// A turned rectangle removes the same area, and its walls are not parallel to
// the plate's sides.
func TestHoleRectangularTurned(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := rectangularHole(plate, decad.ThroughHole{}, 30).Do(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-6*4*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
}

func TestHoleRectangularRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		shape decad.RectangularHole
		want  error
	}{
		{"zero width",
			decad.RectangularHole{Width: units.Millimeters(0), Height: units.Millimeters(4), Angle: units.Degrees(0)},
			decad.ErrDegenerate},
		{"wrong height kind",
			decad.RectangularHole{Width: units.Millimeters(6), Height: units.Degrees(4), Angle: units.Degrees(0)},
			decad.ErrUnitKind},
		{"angle of a length",
			decad.RectangularHole{Width: units.Millimeters(6), Height: units.Millimeters(4), Angle: units.Millimeters(1)},
			decad.ErrUnitKind},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := drillPlate(t)
			_, err := plateShapedHole(plate, tc.shape, decad.BlindHole{Depth: units.Millimeters(5)}).Do(t.Context())
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused hole leaves the plate live and alone")
		})
	}
}
