package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutUsesCertifiedSourceVolumeAndRecordsHistory(t *testing.T) {
	world := sketch.NewWorld()
	doc := New()

	cubeSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	cube := cubeSketch.CreateRectangle(0, 0, 20, 20)
	cubeSketch.Fix(cube.A)
	_, err = cubeSketch.Solve(t.Context())
	require.NoError(t, err)
	target, err := Extrude(t.Context(), doc, cubeSketch, cubeSketch.Profiles()[0],
		Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)

	drillSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := drillSketch.CreatePoint(0, 0)
	drillSketch.Fix(center)
	drillSketch.CreateCircle(center, 1)
	_, err = drillSketch.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, drillSketch, drillSketch.Profiles()[0],
		Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)

	rotation, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(20))
	require.NoError(t, err)
	axis := rotation.ApplyDir(r3.Vec{Z: 1})
	translation, err := r3.Translation(r3.Vec{X: 10, Y: 10, Z: 10}.Sub(axis.Scale(2)))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	tool, err = Transform(t.Context(), tool, placement)
	require.NoError(t, err)

	result, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.Equal(t, "facetedPayload", result.n.Payload.(facetedPayload).SurveyClass())
	require.Len(t, result.Lumps(), 1)
	require.Len(t, result.Shells(), 2)
	require.False(t, result.Shells()[0].IsOpen())
	require.False(t, result.Shells()[1].IsOpen())
	require.NotEqual(t, result.Shells()[0].IsVoid(), result.Shells()[1].IsVoid())

	lineage := result.n.VolumeLineage
	require.NotNil(t, lineage)
	require.Positive(t, lineage.Containment.BoundaryGapLower.Base())
	require.Positive(t, lineage.Containment.InnerShellWitnesses)
	require.Positive(t, lineage.Containment.OuterShellWitnesses)
	require.Less(t, lineage.Derived.Bound.Base(), lineage.Ordinary.Bound.Base())
	require.Equal(t, lineage.Derived, result.n.Volume)
	require.Equal(t, Approximate, result.n.Volume.Exactness)

	want := 8000 - 4*math.Pi
	for _, measured := range []Measurement{lineage.Derived, lineage.Ordinary} {
		require.LessOrEqual(t, math.Abs(measured.Value.Base()-want), measured.Bound.Base()+1e-10)
	}

	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	bodyReport, err := report.ForBody(result)
	require.NoError(t, err)
	require.NotNil(t, bodyReport.Region)
	require.Equal(t, result.n.Volume, bodyReport.Region.Volume.Measurement)

	history := doc.OperationHistory()
	operations := history.Operations()
	require.Len(t, operations, 4)
	require.Equal(t, OperationExtrude, operations[0].Kind())
	require.Equal(t, OperationExtrude, operations[1].Kind())
	require.Equal(t, OperationTransform, operations[2].Kind())
	require.Equal(t, OperationCut, operations[3].Kind())
	require.Equal(t, []*Body{result}, operations[3].Outputs())
	inputs := operations[3].Inputs()
	require.Equal(t, []OperationInput{
		{Body: target, Role: operationRoleTarget, Use: InputConsumed},
		{Body: tool, Role: operationRoleTool, Use: InputConsumed},
	}, inputs)
	producer, err := history.Producer(result)
	require.NoError(t, err)
	require.Same(t, operations[3], producer)
	consumers, err := history.Consumers(target)
	require.NoError(t, err)
	require.Equal(t, []*Operation{operations[3]}, consumers)
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, result, doc.Bodies()[0])
}
