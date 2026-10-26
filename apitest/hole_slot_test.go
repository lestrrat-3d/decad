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

// This file is agent-docs/hole-design.md §6's public-surface tests for the
// SlotHole Shape of Hole, on the 20 x 20 x 10 plate of hole_test.go.

const (
	slotLength = 10.0
	slotWidth  = 4.0
)

// slotArea is a 10 long, 4 wide stadium: a 6 x 4 strip and one full circle of
// radius 2.
var slotArea = (slotLength-slotWidth)*slotWidth + math.Pi*(slotWidth/2)*(slotWidth/2)

func slotHoleSpec(plate *decad.Body, depth decad.HoleDepth, degrees float64) decad.HoleSpec {
	return decad.HoleSpec{
		Face:  capFace(true, plate),
		At:    r3.Vec{X: 10, Y: 10, Z: 10},
		Depth: depth,
		Shape: decad.SlotHole{
			Length: units.Millimeters(slotLength),
			Width:  units.Millimeters(slotWidth),
			Angle:  units.Degrees(degrees),
		},
	}
}

func TestHoleSlotThrough(t *testing.T) {
	t.Parallel()
	doc, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, slotHoleSpec(plate, decad.ThroughHole{}, 0))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-slotArea*10))
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: 20, Y: 20, Z: 10})
	// Two side walls and the two end half cylinders join the plate's six faces.
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 8, decad.KindCylinder: 2})
}

func TestHoleSlotBlindTurned(t *testing.T) {
	t.Parallel()
	_, plate := drillPlate(t)
	out, err := decad.Hole(t.Context(), plate, slotHoleSpec(plate, decad.BlindHole{Depth: units.Millimeters(6)}, 90))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(drillPlateVolume-slotArea*6))
	// A quarter turn lays the slot along y: its bore reaches y = 5 to 15.
	probe, err := out.LocatePoint(t.Context(), r3.Vec{X: 10, Y: 14, Z: 8}, units.Millimeters(1e-6))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, probe, "a point 4 mm along y from the axis lies in the slot")
	probe, err = out.LocatePoint(t.Context(), r3.Vec{X: 14, Y: 10, Z: 8}, units.Millimeters(1e-6))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, probe, "a point 4 mm along x from the axis lies in the plate")
}

func TestHoleSlotRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name  string
		shape decad.SlotHole
		want  error
	}{
		{"length equal to width", decad.SlotHole{Length: units.Millimeters(4), Width: units.Millimeters(4), Angle: units.Degrees(0)}, decad.ErrDegenerate},
		{"length below width", decad.SlotHole{Length: units.Millimeters(3), Width: units.Millimeters(4), Angle: units.Degrees(0)}, decad.ErrDegenerate},
		{"zero width", decad.SlotHole{Length: units.Millimeters(10), Width: units.Millimeters(0), Angle: units.Degrees(0)}, decad.ErrDegenerate},
		{"width as an angle", decad.SlotHole{Length: units.Millimeters(10), Width: units.Degrees(4), Angle: units.Degrees(0)}, decad.ErrUnitKind},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := drillPlate(t)
			spec := slotHoleSpec(plate, decad.BlindHole{Depth: units.Millimeters(5)}, 0)
			spec.Shape = tc.shape
			_, err := decad.Hole(t.Context(), plate, spec)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused hole leaves the plate live and alone")
		})
	}
}
