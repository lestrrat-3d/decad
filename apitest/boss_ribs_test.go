package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §9's public-surface tests for BossCommand.Ribs, on the
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

func ribbedCommand(plate *decad.Body, ribs decad.BossRibs) *decad.BossCommand {
	return bossCommand(plate, true).Ribs(ribs)
}

func TestBossWithGussetRibs(t *testing.T) {
	t.Parallel()
	const count, thick, length, height = 4, 2.0, 6.0, 5.0
	doc, plate := bossPlate(t)
	out, err := ribbedCommand(plate, decad.NewBossRibs(count, units.Millimeters(thick),
		units.Millimeters(length), units.Millimeters(height))).Do(t.Context())
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
	out, err := bossCommand(plate, true).
		Bore(decad.NewBossBore(units.Millimeters(3), units.Millimeters(6))).
		Ribs(decad.NewBossRibs(3, units.Millimeters(1.5), units.Millimeters(5), units.Millimeters(4)).
			Angle(units.Degrees(90))).Do(t.Context())
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
	mm := units.Millimeters
	ribs := func(count int, thickness, length, height units.Value) decad.BossRibs {
		return decad.NewBossRibs(count, thickness, length, height)
	}
	good := ribs(4, mm(2), mm(6), mm(5))
	for name, tc := range map[string]struct {
		cmd  *decad.BossCommand
		want error
	}{
		"no ribs":          {ribbedCommand(plate, ribs(0, mm(2), mm(6), mm(5))), decad.ErrDegenerate},
		"too many":         {ribbedCommand(plate, ribs(17, mm(2), mm(6), mm(5))), decad.ErrDegenerate},
		"zero thickness":   {ribbedCommand(plate, ribs(4, mm(0), mm(6), mm(5))), decad.ErrDegenerate},
		"negative length":  {ribbedCommand(plate, ribs(4, mm(2), mm(-1), mm(5))), decad.ErrNegativeMagnitude},
		"wrong unit":       {ribbedCommand(plate, ribs(4, mm(2), mm(6), units.Degrees(5))), decad.ErrUnitKind},
		"taller than boss": {ribbedCommand(plate, ribs(4, mm(2), mm(6), mm(9))), decad.ErrDegenerate},
		"ribs collide":     {ribbedCommand(plate, ribs(16, mm(3), mm(6), mm(5))), decad.ErrDegenerate},
		"with a draft":     {ribbedCommand(plate, good).Draft(units.Degrees(2)), decad.ErrUnsupported},
		"with a fillet":    {ribbedCommand(plate, good).RootFillet(mm(1)), decad.ErrUnsupported},
	} {
		_, err := tc.cmd.Do(t.Context())
		require.ErrorIs(t, err, tc.want, name)
	}
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "every refusal leaves the plate live")
}

// shapedRibbed is a boss of the given shape and height 8 on the top face of the plate of boss_test.go, braced by the ribs.
func shapedRibbed(plate *decad.Body, shape decad.HoleShape, ribs decad.BossRibs) *decad.BossShapedCommand {
	return shapedBossCommand(plate, shape, 8).Ribs(ribs)
}

func TestBossRibsStandAtTheWallOfARectangularBoss(t *testing.T) {
	t.Parallel()
	const thick, length, height = 2.0, 6.0, 5.0
	for name, tc := range map[string]struct {
		angle   units.Value
		inside  r3.Vec // a point in a rib's foot
		outside r3.Vec // a point on the face where no rib stands
	}{
		// A 20 x 10 boss: its wall is 10 mm from the axis along U and 5 mm along V, and a rib stands where its plane meets that
		// wall, so it is a triangle of length x height / 2 across its thickness outside the boss.
		"along U": {units.Degrees(0), r3.Vec{X: 12, Y: 0, Z: 5.5}, r3.Vec{X: 0, Y: 7, Z: 5.5}},
		"along V": {units.Degrees(90), r3.Vec{X: 0, Y: 7, Z: 5.5}, r3.Vec{X: 12, Y: 0, Z: 5.5}},
	} {
		t.Run(name, func(t *testing.T) {
			doc, plate := bossPlate(t)
			out, err := shapedRibbed(plate,
				decad.RectangularHole{Width: units.Millimeters(20), Height: units.Millimeters(10), Angle: units.Degrees(0)},
				decad.NewBossRibs(2, units.Millimeters(thick), units.Millimeters(length), units.Millimeters(height)).
					Angle(tc.angle)).Do(t.Context())
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			requireVolumeWithin(t, out, bossPlateVolume+20*10*8+2*thick*length*height/2)
			foot, err := out.LocatePoint(t.Context(), tc.inside, units.Millimeters(0.05))
			require.NoError(t, err)
			require.Equal(t, decad.PointInside, foot)
			bare, err := out.LocatePoint(t.Context(), tc.outside, units.Millimeters(0.05))
			require.NoError(t, err)
			require.Equal(t, decad.PointOutside, bare)
		})
	}
}

func TestBossRibsStandAtTheFlatsOfAPolygonalBoss(t *testing.T) {
	t.Parallel()
	const thick, length, height = 2.0, 6.0, 5.0
	_, plate := bossPlate(t)
	// A hexagon 10 mm across flats with a flat toward U: six ribs at 60 degrees from one another meet its six flats square.
	out, err := shapedRibbed(plate,
		decad.PolygonHole{Sides: 6, AcrossFlats: units.Millimeters(10), Angle: units.Degrees(0)},
		decad.NewBossRibs(6, units.Millimeters(thick), units.Millimeters(length), units.Millimeters(height))).
		Do(t.Context())
	require.NoError(t, err)
	hexagon := 6 * 25 * math.Tan(math.Pi/6)
	requireVolumeWithin(t, out, bossPlateVolume+hexagon*8+6*thick*length*height/2)
}

func TestBossRibsStandAtTheEndOfASlot(t *testing.T) {
	t.Parallel()
	const thick, length, height = 2.0, 6.0, 5.0
	_, plate := bossPlate(t)
	// A slot 20 long and 8 wide: a rib along its long axis meets the round end 10 mm from the axis, so it reaches the wall as
	// a rib of a boss of radius 4 does.
	out, err := shapedRibbed(plate,
		decad.SlotHole{Length: units.Millimeters(20), Width: units.Millimeters(8), Angle: units.Degrees(0)},
		decad.NewBossRibs(2, units.Millimeters(thick), units.Millimeters(length), units.Millimeters(height))).
		Do(t.Context())
	require.NoError(t, err)
	slot := 8*12 + math.Pi*16
	requireVolumeWithin(t, out, bossPlateVolume+slot*8+2*ribOutside(4, thick, length, height))
}
