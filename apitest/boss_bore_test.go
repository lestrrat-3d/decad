package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/boss-design.md §4's public-surface tests for the Head
// and Thread of a BossBore, on the plate of boss_test.go with an 8 mm boss 8 mm
// tall (a wider boss makes the pilot's cut coarser than the evaluator allows). The pilot is a 3 mm tap drill and the pitch 0.5 mm, an M3.5 thread.

const (
	bossTapRadius = 1.5
	bossTapPitch  = 0.5
)

func bossTapSpec(plate *decad.Body, bore decad.BossBore) decad.BossSpec {
	return bossSpec(plate, true, &bore)
}

func TestBossPilotWithACounterbore(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, bossTapSpec(plate, decad.BossBore{
		Diameter: units.Millimeters(2 * bossTapRadius), Depth: units.Millimeters(6),
		Head: decad.Counterbore{Diameter: units.Millimeters(6), Depth: units.Millimeters(2)}}))
	require.NoError(t, err)
	want := bossPlateVolume + math.Pi*16*8 - math.Pi*9*2 - math.Pi*bossTapRadius*bossTapRadius*4
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9*want, "volume %v ± %v, want %v", volume.Value.Base(), volume.Bound.Base(), want)
	// The boss top is at z = 13: the recess floor is at z = 11 and the pilot floor at z = 7.
	for _, tc := range []struct {
		z    float64
		x    float64
		want decad.PointLocation
	}{
		{12.5, 2.5, decad.PointOutside}, // in the counterbore, beside the bore
		{10.5, 2.5, decad.PointInside},  // below the counterbore's floor, beside the bore
		{10.5, 0, decad.PointOutside},   // in the bore
		{6.5, 0, decad.PointInside},     // under the pilot's floor at z = 7
	} {
		got, err := out.LocatePoint(t.Context(), r3.NewVec(tc.x, 0, tc.z), units.Millimeters(0.05))
		require.NoError(t, err)
		require.Equal(t, tc.want, got, "at x %v, z %v", tc.x, tc.z)
	}
}

func TestBossPilotWithACountersink(t *testing.T) {
	t.Parallel()
	_, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, bossTapSpec(plate, decad.BossBore{
		Diameter: units.Millimeters(2 * bossTapRadius), Depth: units.Millimeters(6),
		Head: decad.Countersink{Diameter: units.Millimeters(5), Angle: units.Degrees(90)}}))
	require.NoError(t, err)
	// A 90 degree sink 5 wide over a bore 3 wide is 1 mm deep: a frustum of radii 2.5 and 1.5.
	want := bossPlateVolume + math.Pi*16*8 - cylinderFrustum(2.5, 1.5, 1) - math.Pi*bossTapRadius*bossTapRadius*5
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9*want, "volume %v ± %v, want %v", volume.Value.Base(), volume.Bound.Base(), want)
}

func TestBossPilotTapped(t *testing.T) {
	t.Parallel()
	doc, plate := bossPlate(t)
	out, err := decad.Boss(t.Context(), plate, bossTapSpec(plate, decad.BossBore{
		Diameter: units.Millimeters(2 * bossTapRadius), Depth: units.Millimeters(6),
		Thread: &decad.HoleThread{Pitch: units.Millimeters(bossTapPitch), Length: units.Millimeters(4)}}))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationBoss, operations[len(operations)-1].Kind(), "one boss entry holds the boss, the pilot and the thread")
	volume, err := out.Volume()
	require.NoError(t, err)
	turns := (4 + 0.05*bossTapPitch) / bossTapPitch
	want := bossPlateVolume + math.Pi*16*8 - math.Pi*bossTapRadius*bossTapRadius*6 -
		removedPartialInternal(bossTapRadius, bossTapPitch, 4, turns)
	requireThreadVolume(t, volume, want)
}

func TestBossBoreRefusals(t *testing.T) {
	t.Parallel()
	pilot := units.Millimeters(2 * bossTapRadius)
	depth := units.Millimeters(6)
	cases := []struct {
		name string
		bore decad.BossBore
		want error
	}{
		{"a counterbore no wider than the pilot", decad.BossBore{Diameter: pilot, Depth: depth,
			Head: decad.Counterbore{Diameter: units.Millimeters(2), Depth: units.Millimeters(1)}}, decad.ErrDegenerate},
		{"a counterbore as deep as the pilot", decad.BossBore{Diameter: pilot, Depth: depth,
			Head: decad.Counterbore{Diameter: units.Millimeters(6), Depth: depth}}, decad.ErrDegenerate},
		{"a thread longer than the pilot", decad.BossBore{Diameter: pilot, Depth: depth,
			Thread: &decad.HoleThread{Pitch: units.Millimeters(bossTapPitch), Length: units.Millimeters(7)}}, decad.ErrDegenerate},
		{"a zero thread pitch", decad.BossBore{Diameter: pilot, Depth: depth,
			Thread: &decad.HoleThread{Pitch: units.Millimeters(0)}}, decad.ErrDegenerate},
	}
	for _, tc := range cases {
		doc, plate := bossPlate(t)
		_, err := decad.Boss(t.Context(), plate, bossTapSpec(plate, tc.bore))
		require.ErrorIs(t, err, tc.want, tc.name)
		require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "%s: a refused boss leaves the plate live and alone", tc.name)
	}
}
