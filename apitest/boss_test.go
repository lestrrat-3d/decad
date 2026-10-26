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

func bossSpec(plate *decad.Body, top bool, bore *decad.BossBore) decad.BossSpec {
	z := 0.0
	if top {
		z = 5
	}
	return decad.BossSpec{
		Face:     capFace(top, plate),
		At:       r3.Vec{X: 0, Y: 0, Z: z},
		Diameter: units.Millimeters(8),
		Height:   units.Millimeters(8),
		Bore:     bore,
	}
}

func TestBossSolid(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, bossSpec(plate, true, nil))
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
	out, err := decad.Boss(t.Context(), plate, bossSpec(plate, true, &decad.BossBore{
		Diameter: units.Millimeters(3), Depth: units.Millimeters(6)}))
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
	out, err := decad.Boss(t.Context(), plate, bossSpec(plate, false, nil))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bossPlateVolume+math.Pi*16*8))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20, Z: -8}, r3.Vec{X: 20, Y: 20, Z: 5})
}

func TestBossRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		edit func(*decad.BossSpec)
		want error
	}{
		{"no selector", func(s *decad.BossSpec) { s.Face = nil }, decad.ErrDegenerate},
		{"zero diameter", func(s *decad.BossSpec) { s.Diameter = units.Millimeters(0) }, decad.ErrDegenerate},
		{"zero height", func(s *decad.BossSpec) { s.Height = units.Millimeters(0) }, decad.ErrDegenerate},
		{"height as an angle", func(s *decad.BossSpec) { s.Height = units.Degrees(8) }, decad.ErrUnitKind},
		{"centre off the plane", func(s *decad.BossSpec) { s.At = r3.Vec{Z: 4} }, decad.ErrDegenerate},
		{"pilot as wide as the boss", func(s *decad.BossSpec) {
			s.Bore = &decad.BossBore{Diameter: units.Millimeters(8), Depth: units.Millimeters(2)}
		}, decad.ErrDegenerate},
		{"pilot through the back", func(s *decad.BossSpec) {
			s.Bore = &decad.BossBore{Diameter: units.Millimeters(2), Depth: units.Millimeters(14)}
		}, decad.ErrDegenerate},
		{"zero pilot depth", func(s *decad.BossSpec) {
			s.Bore = &decad.BossBore{Diameter: units.Millimeters(2), Depth: units.Millimeters(0)}
		}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, plate := bossPlate(t)
			spec := bossSpec(plate, true, nil)
			tc.edit(&spec)
			_, err := decad.Boss(t.Context(), plate, spec)
			require.ErrorIs(t, err, tc.want)
			require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refused boss leaves the plate live and alone")
		})
	}
}

func TestBossRefusesAFaceThatIsNotPlanar(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	rod := circleBodyAtZ(t, doc, 0, 5, 0, 20)
	_, err := decad.Boss(t.Context(), rod, decad.BossSpec{
		Face: decad.Faces(decad.Cylindrical()), At: r3.Vec{X: 5},
		Diameter: units.Millimeters(2), Height: units.Millimeters(2)})
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{rod}, doc.Bodies())
}
