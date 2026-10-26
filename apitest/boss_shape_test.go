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

// This file is agent-docs/boss-design.md §7's public-surface tests for a shaped
// boss, on the plate of boss_test.go.

func shapedBossSpec(plate *decad.Body, shape decad.HoleShape, height float64) decad.BossSpec {
	return decad.BossSpec{
		Face:   capFace(true, plate),
		At:     r3.Vec{X: 0, Y: 0, Z: 5},
		Height: units.Millimeters(height),
		Shape:  shape,
	}
}

func TestBossRectangularStandsOnThePlate(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, shapedBossSpec(plate,
		decad.RectangularHole{Width: units.Millimeters(10), Height: units.Millimeters(6), Angle: units.Degrees(0)}, 5))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == out)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+10*6*5))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 10})
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 11})
}

func TestBossPolygonAndSlotVolumes(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	hex, err := decad.Boss(t.Context(), plate, shapedBossSpec(plate,
		decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(8), Angle: units.Degrees(10)}, 4))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, hex, units.CubicMillimeters(bossPlateVolume+6*16*math.Tan(math.Pi/6)*4))
	_, plate = bossPlate(t)
	slot, err := decad.Boss(t.Context(), plate, shapedBossSpec(plate,
		decad.SlotHole{Length: units.Millimeters(14), Width: units.Millimeters(6), Angle: units.Degrees(30)}, 3))
	require.NoError(t, err)
	// A stadium: a 8 x 6 rectangle and two half discs of radius 3.
	decadtest.MeasuresVolume(t, slot, units.CubicMillimeters(bossPlateVolume+(8*6+math.Pi*9)*3), decadtest.WithinRel(units.Scalar(1e-6)))
}

func TestBossRectangularStaysWhereTheShapeIsTurned(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, shapedBossSpec(plate,
		decad.RectangularHole{Width: units.Millimeters(10), Height: units.Millimeters(2), Angle: units.Degrees(90)}, 4))
	require.NoError(t, err)
	// Turned a quarter, the 10 mm side lies along Y: a point 4 mm out along Y is in the boss, along X not.
	in, err := out.LocatePoint(t.Context(), r3.Vec{Y: 4, Z: 7}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, in)
	beside, err := out.LocatePoint(t.Context(), r3.Vec{X: 4, Z: 7}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, beside)
}

func TestBossShapedDraftLeansEveryWall(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	spec := shapedBossSpec(plate, decad.RectangularHole{Width: units.Millimeters(10), Height: units.Millimeters(6), Angle: units.Degrees(0)}, 5)
	spec.Draft = units.Degrees(4)
	out, err := decad.Boss(t.Context(), plate, spec)
	require.NoError(t, err)
	// Each wall leans in by tan 4 degrees a millimetre: the section at height z is (10 − 2zt)(6 − 2zt).
	tn := math.Tan(4 * math.Pi / 180)
	want := bossPlateVolume + 10*6*5 - (10+6)*tn*25 + 4.0/3*tn*tn*125
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.WithinRel(units.Scalar(1e-3)))
}

func TestBossShapedWithAPilot(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	spec := shapedBossSpec(plate, decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(8), Angle: units.Degrees(0)}, 6)
	spec.Bore = &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(4)}
	out, err := decad.Boss(t.Context(), plate, spec)
	require.NoError(t, err)
	hexArea := 6 * 16 * math.Tan(math.Pi/6)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+hexArea*6-math.Pi*2.25*4))
	// A pilot as wide as the flats does not fit.
	_, plate = bossPlate(t)
	spec = shapedBossSpec(plate, decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(8), Angle: units.Degrees(0)}, 6)
	spec.Bore = &decad.BossBore{Diameter: units.Millimeters(8), Depth: units.Millimeters(4)}
	_, err = decad.Boss(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestBossShapedRefusals(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	rect := decad.RectangularHole{Width: units.Millimeters(10), Height: units.Millimeters(6), Angle: units.Degrees(0)}
	spec := shapedBossSpec(plate, rect, 5)
	spec.Diameter = units.Millimeters(8)
	_, err := decad.Boss(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a diameter beside a shape")
	spec = shapedBossSpec(plate, rect, 5)
	spec.RootFillet = units.Millimeters(1)
	_, err = decad.Boss(t.Context(), plate, spec)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Boss(t.Context(), plate, shapedBossSpec(plate, decad.PolygonHole{Sides: 2, AcrossFlats: units.Millimeters(5), Angle: units.Degrees(0)}, 5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Boss(t.Context(), plate, shapedBossSpec(plate, decad.SlotHole{Length: units.Millimeters(4), Width: units.Millimeters(6), Angle: units.Degrees(0)}, 5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Boss(t.Context(), plate, shapedBossSpec(plate, decad.RectangularHole{Width: units.Degrees(1), Height: units.Millimeters(6), Angle: units.Degrees(0)}, 5))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	require.Len(t, doc.Bodies(), 1)
	require.True(t, doc.Bodies()[0] == plate, "a refused boss leaves the plate live and alone")
}
