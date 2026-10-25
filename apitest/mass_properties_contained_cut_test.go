package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestMassPropertiesContainedCutSourceMoments(t *testing.T) {
	world := sketch.NewWorld()
	doc := decad.New()
	boxSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	box := boxSketch.CreateRectangle(0, 0, 20, 20)
	boxSketch.Fix(box.A)
	_, err = boxSketch.Solve(t.Context())
	require.NoError(t, err)
	target, err := decad.Extrude(t.Context(), doc, boxSketch, boxSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)

	drillSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := drillSketch.CreatePoint(0, 0)
	drillSketch.Fix(center)
	drillSketch.CreateCircle(center, 1)
	_, err = drillSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), doc, drillSketch, drillSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
	require.NoError(t, err)
	rotation, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(20))
	require.NoError(t, err)
	axis := rotation.ApplyDir(r3.Vec{Z: 1})
	translation, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: 10}.Sub(axis.Scale(2)))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, placement)
	require.NoError(t, err)
	result, err := decad.Cut(t.Context(), target, tool)
	require.NoError(t, err)
	history := doc.OperationHistory().Operations()
	require.Len(t, history, 4)

	unitDensity := units.KilogramsPerCubicMillimeter(1)
	doubleDensity := units.KilogramsPerCubicMillimeter(2)
	unit, err := result.MassProperties(t.Context(), unitDensity)
	require.NoError(t, err)
	doubled, err := result.MassProperties(t.Context(), doubleDensity)
	require.NoError(t, err)
	require.Equal(t, unit.Center, doubled.Center)
	for _, pair := range [][2]decad.Measurement{
		{unit.Mass, doubled.Mass},
		{unit.Inertia.XX, doubled.Inertia.XX}, {unit.Inertia.YY, doubled.Inertia.YY},
		{unit.Inertia.ZZ, doubled.Inertia.ZZ}, {unit.Inertia.XY, doubled.Inertia.XY},
		{unit.Inertia.XZ, doubled.Inertia.XZ}, {unit.Inertia.YZ, doubled.Inertia.YZ},
	} {
		require.InDelta(t, pair[0].Value.Base()*2, pair[1].Value.Base(), 1e-10)
		require.InDelta(t, pair[0].Bound.Base()*2, pair[1].Bound.Base(), 1e-10)
	}
	after := doc.OperationHistory().Operations()
	require.Len(t, after, len(history))
	for i := range history {
		require.Same(t, history[i], after[i])
	}
}
