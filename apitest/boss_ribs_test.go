package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §9's public-surface tests for BossRibs, on the
// plate of boss_test.go: a boss of diameter 8 and height 8 on its top face, z = 5.

// ribOutside is the volume of one gusset rib of the given thickness, length and height that
// lies outside a boss of the given radius and above the plate: the rib's section is flat at the
// height from the axis to the boss's wall radius, and slopes down to the face at radius + length,
// and the boss's wall at distance y across the rib is √(r² − y²), so each slice of the rib
// across it holds height·(r − √(r² − y²)) of flat top before the wall's radius and
// height·length/2 of slope after it.
func ribOutside(radius, thickness, length, height float64) float64 {
	const steps = 200000
	half := thickness / 2
	step := thickness / steps
	flat := 0.0
	for i := range steps {
		y := -half + (float64(i)+0.5)*step
		flat += (radius - math.Sqrt(radius*radius-y*y)) * step
	}
	return height * (thickness*length/2 + flat)
}

func ribbedSpec(plate *decad.Body, ribs *decad.BossRibs) decad.BossSpec {
	spec := bossSpec(plate, true, nil)
	spec.Ribs = ribs
	return spec
}

func TestBossWithGussetRibs(t *testing.T) {
	t.Parallel()
	const count, thick, length, height = 4, 2.0, 6.0, 5.0
	doc, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, ribbedSpec(plate, &decad.BossRibs{
		Count: count, Thickness: units.Millimeters(thick), Length: units.Millimeters(length),
		Height: units.Millimeters(height)}))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the plate and the tools leave the document")
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume,
		bossPlateVolume+math.Pi*16*8+count*ribOutside(4, thick, length, height))
	// The ribs reach 6 mm past the boss's wall (4 mm) along +X: a point on the face in the
	// rib is material at its foot and empty off the rib's line.
	foot, err := out.LocatePoint(t.Context(), r3.Vec{X: 7, Y: 0, Z: 5.5}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, foot)
	between, err := out.LocatePoint(t.Context(), r3.Vec{X: 7, Y: 3.5, Z: 5.5}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, between)
}

func TestBossRibsTurnWithTheirAngleAndLeavePilotHoleClear(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	spec := bossSpec(plate, true, &decad.BossBore{Diameter: units.Millimeters(3), Depth: units.Millimeters(6)})
	spec.Ribs = &decad.BossRibs{Count: 3, Thickness: units.Millimeters(1.5), Length: units.Millimeters(5),
		Height: units.Millimeters(4), Angle: units.Degrees(90)}
	out, err := decad.Boss(t.Context(), plate, spec)
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	requireCylinderHoleVolume(t, volume,
		bossPlateVolume+math.Pi*16*8-math.Pi*2.25*6+3*ribOutside(4, 1.5, 5, 4))
	// The first rib lies along +Y: its foot at y = 6.5 is material and +X, at 0 and 120 and 240 degrees
	// from +Y, has none.
	along, err := out.LocatePoint(t.Context(), r3.Vec{X: 0, Y: 6.5, Z: 5.5}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, along)
	away, err := out.LocatePoint(t.Context(), r3.Vec{X: 6.5, Y: 0, Z: 5.5}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, away)
	// The pilot is cut after the ribs, so the axis in the boss is clear down to its floor.
	pilot, err := out.LocatePoint(t.Context(), r3.Vec{X: 0, Y: 0, Z: 9}, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, pilot)
}

func TestBossRibsRefusals(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	good := decad.BossRibs{Count: 4, Thickness: units.Millimeters(2), Length: units.Millimeters(6), Height: units.Millimeters(5)}
	boss := func(edit func(*decad.BossSpec, *decad.BossRibs)) error {
		ribs := good
		spec := bossSpec(plate, true, nil)
		edit(&spec, &ribs)
		spec.Ribs = &ribs
		_, err := decad.Boss(t.Context(), plate, spec)
		return err
	}
	for name, tc := range map[string]struct {
		edit func(*decad.BossSpec, *decad.BossRibs)
		want error
	}{
		"no ribs":          {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Count = 0 }, decad.ErrDegenerate},
		"too many":         {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Count = 17 }, decad.ErrDegenerate},
		"zero thickness":   {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Thickness = units.Millimeters(0) }, decad.ErrDegenerate},
		"negative length":  {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Length = units.Millimeters(-1) }, decad.ErrNegativeMagnitude},
		"wrong unit":       {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Height = units.Degrees(5) }, decad.ErrUnitKind},
		"taller than boss": {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Height = units.Millimeters(9) }, decad.ErrDegenerate},
		"ribs collide":     {func(_ *decad.BossSpec, r *decad.BossRibs) { r.Count = 16; r.Thickness = units.Millimeters(3) }, decad.ErrDegenerate},
		"with a draft":     {func(s *decad.BossSpec, _ *decad.BossRibs) { s.Draft = units.Degrees(2) }, decad.ErrUnsupported},
		"with a fillet":    {func(s *decad.BossSpec, _ *decad.BossRibs) { s.RootFillet = units.Millimeters(1) }, decad.ErrUnsupported},
	} {
		require.ErrorIs(t, boss(tc.edit), tc.want, name)
	}
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "every refusal leaves the plate live")
}
