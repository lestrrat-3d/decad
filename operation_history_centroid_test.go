package decad

import (
	"math"
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutUsesCertifiedSourceCentroid(t *testing.T) {
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
	translation, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: 10}.Sub(axis.Scale(2)))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	tool, err = Transform(t.Context(), tool, placement)
	require.NoError(t, err)

	targetVolume, err := target.Volume()
	require.NoError(t, err)
	targetCentroid, err := target.Centroid()
	require.NoError(t, err)
	toolVolume, err := tool.Volume()
	require.NoError(t, err)
	toolCentroid, err := tool.Centroid()
	require.NoError(t, err)

	result, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.Equal(t, "facetedPayload", result.n.Payload.(facetedPayload).SurveyClass())
	require.Len(t, result.Lumps(), 1)
	require.Len(t, result.Shells(), 2)
	require.False(t, result.Shells()[0].IsOpen())
	require.False(t, result.Shells()[1].IsOpen())
	require.NotEqual(t, result.Shells()[0].IsVoid(), result.Shells()[1].IsVoid())

	volumeLineage := result.n.VolumeLineage
	require.NotNil(t, volumeLineage)
	require.Positive(t, volumeLineage.Containment.BoundaryGapLower.Base())
	require.Positive(t, volumeLineage.Containment.InnerShellWitnesses)
	require.Positive(t, volumeLineage.Containment.OuterShellWitnesses)
	centroidLineage := volumeLineage.Centroid
	require.NotNil(t, centroidLineage)
	require.True(t, centroidLineage.Selected)
	require.Less(t, centroidLineage.Derived.Bound.Base(), centroidLineage.Ordinary.Bound.Base())

	derivedCentroid, err := result.Centroid()
	require.NoError(t, err)
	require.Equal(t, centroidLineage.Derived, derivedCentroid)
	reference, ok := containedCutCentroidReference(targetVolume, targetCentroid, toolVolume, toolCentroid)
	require.True(t, ok, "certified source intervals must prove a positive reference volume")
	require.True(t, centroidBallContainsIntervals(centroidLineage.Derived, reference))
	require.True(t, centroidBallContainsIntervals(centroidLineage.Ordinary, reference))

	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	bodyReport, err := report.ForBody(result)
	require.NoError(t, err)
	require.NotNil(t, bodyReport.Region)
	require.Equal(t, derivedCentroid, bodyReport.Region.Centroid.VecMeasurement)

	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, 4)
	require.Equal(t, OperationExtrude, operations[0].Kind())
	require.Equal(t, OperationExtrude, operations[1].Kind())
	require.Equal(t, OperationTransform, operations[2].Kind())
	require.Equal(t, OperationCut, operations[3].Kind())
	require.Equal(t, []*Body{result}, operations[3].Outputs())
	require.Equal(t, []OperationInput{
		{Body: target, Role: operationRoleTarget, Use: InputConsumed},
		{Body: tool, Role: operationRoleTool, Use: InputConsumed},
	}, operations[3].Inputs())
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, result, doc.Bodies()[0])
}

func TestContainedCutCentroidEvidenceFailureKeepsDocumentUnchanged(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	target.n.Centroid.Value.X = math.NaN()

	bodiesBefore := doc.Bodies()
	operationsBefore := doc.OperationHistory().Operations()
	producerBefore := doc.nextProducerID()
	_, err := Cut(t.Context(), target, tool)
	require.ErrorContains(t, err, "contained-cut centroid evidence is invalid")
	var booleanErr *BooleanError
	require.ErrorAs(t, err, &booleanErr)
	require.Equal(t, BooleanEvaluatorFailure, booleanErr.Code)

	require.Equal(t, bodiesBefore, doc.Bodies())
	require.Equal(t, producerBefore, doc.nextProducerID())
	operationsAfter := doc.OperationHistory().Operations()
	require.Len(t, operationsAfter, len(operationsBefore))
	for i := range operationsBefore {
		require.Same(t, operationsBefore[i], operationsAfter[i])
	}
}

func containedCutCentroidSources(t *testing.T) (*Document, *Body, *Body) {
	t.Helper()
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
	translation, err := r3.Translation(r3.Vec{X: 12, Y: 10, Z: 10}.Sub(axis.Scale(2)))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	tool, err = Transform(t.Context(), tool, placement)
	require.NoError(t, err)
	return doc, target, tool
}

func containedCutCentroidReference(targetVolume Measurement, targetCentroid VecMeasurement,
	toolVolume Measurement, toolCentroid VecMeasurement) ([3]proofbound.RatInterval, bool) {
	targetVol, ok := intersectContainedCutIntervals(containedCutMeasurementInterval(targetVolume),
		proofbound.PointInterval(proofarith.FloatRat(8000)))
	if !ok {
		return [3]proofbound.RatInterval{}, false
	}
	toolVol, ok := intersectContainedCutIntervals(containedCutMeasurementInterval(toolVolume),
		proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), big.NewRat(4, 1)))
	if !ok {
		return [3]proofbound.RatInterval{}, false
	}
	denominator := proofbound.IntervalSub(targetVol, toolVol)
	anchors := [3]float64{targetCentroid.Value.X, targetCentroid.Value.Y, targetCentroid.Value.Z}
	targetCoordinates := [3]float64{
		targetCentroid.Value.X, targetCentroid.Value.Y, targetCentroid.Value.Z,
	}
	toolCoordinates := [3]float64{toolCentroid.Value.X, toolCentroid.Value.Y, toolCentroid.Value.Z}
	var result [3]proofbound.RatInterval
	for i, anchor := range anchors {
		anchorInterval := proofbound.PointInterval(proofarith.FloatRat(anchor))
		targetCoordinate := containedCutCoordinateInterval(targetCoordinates[i], targetCentroid.Bound.Base())
		toolCoordinate := containedCutCoordinateInterval(toolCoordinates[i], toolCentroid.Bound.Base())
		targetMoment := proofbound.IntervalMul(targetVol, proofbound.IntervalSub(targetCoordinate, anchorInterval))
		toolMoment := proofbound.IntervalMul(toolVol, proofbound.IntervalSub(toolCoordinate, anchorInterval))
		numerator := proofbound.IntervalSub(targetMoment, toolMoment)
		quotient, ok := proofbound.IntervalQuo(numerator, denominator)
		if !ok {
			return [3]proofbound.RatInterval{}, false
		}
		result[i] = proofbound.IntervalAdd(anchorInterval, quotient)
	}
	return result, true
}

func intersectContainedCutIntervals(a, b proofbound.RatInterval) (proofbound.RatInterval, bool) {
	lo, hi := a.Lo, a.Hi
	if b.Lo.Cmp(lo) > 0 {
		lo = b.Lo
	}
	if b.Hi.Cmp(hi) < 0 {
		hi = b.Hi
	}
	if lo.Cmp(hi) > 0 {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(lo, hi), true
}

func containedCutMeasurementInterval(measurement Measurement) proofbound.RatInterval {
	center := proofarith.FloatRat(measurement.Value.Base())
	bound := proofarith.FloatRat(measurement.Bound.Base())
	return proofbound.Interval(new(big.Rat).Sub(center, bound), new(big.Rat).Add(center, bound))
}

func containedCutCoordinateInterval(value, bound float64) proofbound.RatInterval {
	center := proofarith.FloatRat(value)
	radius := proofarith.FloatRat(bound)
	return proofbound.Interval(new(big.Rat).Sub(center, radius), new(big.Rat).Add(center, radius))
}

func centroidBallContainsIntervals(centroid VecMeasurement, reference [3]proofbound.RatInterval) bool {
	coordinates := [3]float64{centroid.Value.X, centroid.Value.Y, centroid.Value.Z}
	maxError := [3]*big.Rat{}
	for i, coordinate := range coordinates {
		difference := proofbound.IntervalSub(reference[i],
			proofbound.PointInterval(proofarith.FloatRat(coordinate)))
		maxError[i] = proofbound.IntervalAbsUpper(difference)
	}
	squaredDistance := new(big.Rat).Mul(maxError[0], maxError[0])
	squaredDistance.Add(squaredDistance, new(big.Rat).Mul(maxError[1], maxError[1]))
	squaredDistance.Add(squaredDistance, new(big.Rat).Mul(maxError[2], maxError[2]))
	radius := proofarith.FloatRat(centroid.Bound.Base())
	radiusSquared := new(big.Rat).Mul(radius, radius)
	return squaredDistance.Cmp(radiusSquared) <= 0
}
