package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §4's public-surface tests for features cut into and raised on a mesh
// solid: a hole or boss with straight sides reaches the exact grid route, and a round one the general mesh route.

func meshPlate(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	return blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 10))
}

func TestMeshSolidTakesStraightSidedPocketsAndBossesExactly(t *testing.T) {
	t.Parallel()
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	rect := decad.RectangularHole{Width: units.Millimeters(10), Height: units.Millimeters(6), Angle: units.Degrees(0)}
	plate := meshPlate(t)
	pocket, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face: top, At: r3.NewVec(20, 15, 10), Depth: decad.BlindHole{Depth: units.Millimeters(4)}, Shape: rect})
	require.NoError(t, err)
	requireVolume(t, pocket, 12000-10*6*4)
	require.Len(t, pocket.Faces(), 6+5, "the plate's six faces and the pocket's four walls and floor")
	plate = meshPlate(t)
	slot, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face: top, At: r3.NewVec(20, 15, 10), Depth: decad.ThroughHole{}, Shape: decad.OutlineHole{
			Corners: []decad.Point2{{U: -4, V: -4}, {U: 4, V: -4}, {U: 4, V: 0}, {U: 0, V: 0}, {U: 0, V: 4}, {U: -4, V: 4}}}})
	require.NoError(t, err)
	requireVolume(t, slot, 12000-48*10)
	plate = meshPlate(t)
	boss, err := decad.Boss(t.Context(), plate, decad.BossSpec{Face: top, At: r3.NewVec(20, 15, 10), Shape: rect, Height: units.Millimeters(5)})
	require.NoError(t, err)
	requireVolume(t, boss, 12000+10*6*5)
	bounds, err := boss.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 15, bounds.Max.Z, 1e-9)
}

func TestMeshSolidTakesRoundHolesWithinTheirBound(t *testing.T) {
	t.Parallel()
	plate := meshPlate(t)
	drilled, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face: decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), At: r3.NewVec(20, 15, 10),
		Diameter: units.Millimeters(6), Depth: decad.ThroughHole{}})
	require.NoError(t, err)
	volume, err := drilled.Volume()
	require.NoError(t, err)
	// A round bore is cut as chords on the general mesh route, so its volume is the bore's within the stated bound.
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(12000-math.Pi*9*10)), volume.Bound.Base()+1e-9)
	require.Less(t, volume.Bound.Base(), 1.0)
	inside, err := drilled.LocatePoint(t.Context(), r3.NewVec(20, 15, 5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inside, "the bore is empty")
}
