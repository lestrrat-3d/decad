package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §10's public-surface tests for a tilted Boss, on the
// plate of boss_test.go: a boss of diameter 8 and height 8 on its top face, z = 5.

// tiltPlateVolume and tiltPlate: a 10 mm plate, thick enough for the footing of a boss tilted 45
// degrees, whose tilted base disc reaches 2·r·sin(a) + 1 mm below the face.
const tiltPlateVolume = 40 * 40 * 10

func tiltPlate(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	return doc, boxBody(t, doc, -20, -20, 20, 20, 10)
}

func tiltedSpec(plate *decad.Body, tilt, direction units.Value) decad.BossSpec {
	return decad.BossSpec{
		Face: capFace(true, plate), At: r3.Vec{Z: 10},
		Diameter: units.Millimeters(8), Height: units.Millimeters(8),
		Tilt: tilt, TiltDirection: direction,
	}
}

func TestBossLeansAlongAnAxis(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		tilt, direction float64
		lean            r3.Vec // the unit direction in the face the boss leans toward
	}{
		"toward +X":   {30, 0, r3.NewVec(1, 0, 0)},
		"toward +Y":   {30, 90, r3.NewVec(0, 1, 0)},
		"toward -X":   {45, 180, r3.NewVec(-1, 0, 0)},
		"toward 45 d": {20, 45, r3.NewVec(math.Sqrt2/2, math.Sqrt2/2, 0)},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			doc, plate := tiltPlate(t)
			out, err := decad.Boss(t.Context(), plate, tiltedSpec(plate, units.Degrees(tc.tilt), units.Degrees(tc.direction)))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			a := tc.tilt * math.Pi / 180
			// A right cylinder cut by the face through its axis, at the middle of the cut, holds
			// pi·r²·h of its length along the axis from that point to its end disc: the oblique
			// cut is balanced about it.
			volume, err := out.Volume()
			require.NoError(t, err)
			requireCylinderHoleVolume(t, volume, tiltPlateVolume+math.Pi*16*8)
			// The boss's top disc is centred 8 along the axis from (0, 0, 10), and the part
			// reaches 4·sin(a) above the centre and 4·cos(a) to the side.
			box, err := out.Bounds()
			require.NoError(t, err)
			require.InDelta(t, 10+8*math.Cos(a)+4*math.Sin(a), box.Max.Z, 0.01, "the top disc's highest point, to its chord")
			// The axis is material 4 mm up it, and the point straight above the face's middle
			// at the same height is beside the boss, 4·sin(a) off the axis.
			up := r3.NewVec(0, 0, 1)
			axisPoint := up.Scale(10 + 4*math.Cos(a)).Add(tc.lean.Scale(4 * math.Sin(a)))
			in, err := out.LocatePoint(t.Context(), axisPoint, units.Millimeters(0.05))
			require.NoError(t, err)
			require.Equal(t, decad.PointInside, in)
			far := axisPoint.Add(tc.lean.Scale(-(4 + 4*math.Cos(a))))
			side, err := out.LocatePoint(t.Context(), far, units.Millimeters(0.05))
			require.NoError(t, err)
			require.Equal(t, decad.PointOutside, side, "the boss does not reach back across its own foot")
		})
	}
}

func TestBossTiltRefusals(t *testing.T) {
	t.Parallel()
	doc, plate := tiltPlate(t)
	boss := func(edit func(*decad.BossSpec)) error {
		spec := tiltedSpec(plate, units.Degrees(30), units.Value{})
		edit(&spec)
		_, err := decad.Boss(t.Context(), plate, spec)
		return err
	}
	for name, tc := range map[string]struct {
		edit func(*decad.BossSpec)
		want error
	}{
		"past 60 degrees":     {func(s *decad.BossSpec) { s.Tilt = units.Degrees(61) }, decad.ErrDegenerate},
		"a negative tilt":     {func(s *decad.BossSpec) { s.Tilt = units.Degrees(-5) }, decad.ErrNegativeMagnitude},
		"a length for a tilt": {func(s *decad.BossSpec) { s.Tilt = units.Millimeters(1) }, decad.ErrUnitKind},
		"a direction alone":   {func(s *decad.BossSpec) { s.Tilt = units.Value{}; s.TiltDirection = units.Degrees(10) }, decad.ErrDegenerate},
		"a length direction":  {func(s *decad.BossSpec) { s.TiltDirection = units.Millimeters(1) }, decad.ErrUnitKind},
		"with a pilot": {func(s *decad.BossSpec) {
			s.Bore = &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(5)}
		}, decad.ErrUnsupported},
		"with a draft":       {func(s *decad.BossSpec) { s.Draft = units.Degrees(2) }, decad.ErrUnsupported},
		"with a root fillet": {func(s *decad.BossSpec) { s.RootFillet = units.Millimeters(1) }, decad.ErrUnsupported},
		"with ribs": {func(s *decad.BossSpec) {
			s.Ribs = &decad.BossRibs{Count: 2, Thickness: units.Millimeters(1), Length: units.Millimeters(4), Height: units.Millimeters(3)}
		}, decad.ErrUnsupported},
	} {
		require.ErrorIs(t, boss(tc.edit), tc.want, name)
	}
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "every refusal leaves the plate live")
}
