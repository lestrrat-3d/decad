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

// This file is agent-docs/boss-design.md's public-surface tests for Boss, on a
// 40 x 40 x 5 plate whose top face is at z = 5 and bottom face at z = 0.

const bossPlateVolume = 40 * 40 * 5

func bossPlate(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	return doc, boxBody(t, doc, -20, -20, 20, 20, 5)
}

// bossCommand is a boss of diameter 8 and height 8 on the top face of plate, z = 5, or on
// its bottom face, z = 0. A test adds a bore or other methods to it.
func bossCommand(plate *decad.Body, top bool) *decad.BossCommand {
	z := 0.0
	if top {
		z = 5
	}
	return decad.Boss(plate, capFace(top, plate), r3.Vec{X: 0, Y: 0, Z: z}, units.Millimeters(8), units.Millimeters(8))
}

func TestBossSolid(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	out, err := bossCommand(plate, true).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the plate and the tools leave the document")
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationBoss, operations[len(operations)-1].Kind())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+math.Pi*16*8))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
}

func TestBossWithPilotHole(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := bossCommand(plate, true).
		Bore(decad.NewBossBore(units.Millimeters(3), units.Millimeters(6))).Do(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+math.Pi*16*8-math.Pi*2.25*6))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 13})
	// The pilot is blind and 6 mm deep from z = 13, so its floor is at z = 7, inside the boss.
	inside, err := out.LocatePoint(t.Context(), r3.Vec{Z: 7.5}, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "above the pilot's floor, in the hole")
	below, err := out.LocatePoint(t.Context(), r3.Vec{Z: 6.5}, units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, below, "below the pilot's floor, in the boss")
}

func TestBossOnTheUnderside(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := bossCommand(plate, false).Do(t.Context())
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+math.Pi*16*8))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20, Z: -8}, r3.Vec{X: 20, Y: 20, Z: 5})
}

func TestBossRefusals(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	cases := []struct {
		name string
		make func(plate *decad.Body) *decad.BossCommand
		want error
	}{
		{"no selector", func(plate *decad.Body) *decad.BossCommand {
			return decad.Boss(plate, nil, r3.Vec{Z: 5}, mm(8), mm(8))
		}, decad.ErrDegenerate},
		{"zero diameter", func(plate *decad.Body) *decad.BossCommand {
			return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(0), mm(8))
		}, decad.ErrDegenerate},
		{"zero height", func(plate *decad.Body) *decad.BossCommand {
			return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), mm(0))
		}, decad.ErrDegenerate},
		{"height as an angle", func(plate *decad.Body) *decad.BossCommand {
			return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 5}, mm(8), units.Degrees(8))
		}, decad.ErrUnitKind},
		{"centre off the plane", func(plate *decad.Body) *decad.BossCommand {
			return decad.Boss(plate, capFace(true, plate), r3.Vec{Z: 4}, mm(8), mm(8))
		}, decad.ErrDegenerate},
		{"pilot as wide as the boss", func(plate *decad.Body) *decad.BossCommand {
			return bossCommand(plate, true).Bore(decad.NewBossBore(mm(8), mm(2)))
		}, decad.ErrDegenerate},
		{"pilot through the back", func(plate *decad.Body) *decad.BossCommand {
			return bossCommand(plate, true).Bore(decad.NewBossBore(mm(2), mm(14)))
		}, decad.ErrDegenerate},
		{"zero pilot depth", func(plate *decad.Body) *decad.BossCommand {
			return bossCommand(plate, true).Bore(decad.NewBossBore(mm(2), mm(0)))
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := bossPlate(t)
			_, err := tc.make(plate).Do(t.Context())
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused boss leaves the plate live and alone")
		})
	}
}

func TestBossRefusesAFaceThatIsNeitherPlanarNorCylindrical(t *testing.T) {
	t.Parallel()
	cone := coneBody(t, 10)
	doc := cone.Document()
	_, err := decad.Boss(cone, sideFace(t, cone, coneSurface), r3.Vec{X: 5, Y: 5}, units.Millimeters(2), units.Millimeters(2)).Do(t.Context())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{cone}, doc.Bodies())
}
