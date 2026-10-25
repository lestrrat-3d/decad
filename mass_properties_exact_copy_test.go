package decad

import (
	"math/big"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExactCopyUsesContainedCutSourceMassProperties(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	source, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.NotNil(t, source.n.VolumeLineage.Moments)
	require.Len(t, source.Lumps(), 1)
	require.Len(t, source.Shells(), 2)
	require.False(t, source.Shells()[0].IsOpen())
	require.False(t, source.Shells()[1].IsOpen())

	historyBefore := doc.OperationHistory().Operations()
	center, axis := r3.NewVec(10, 10, 10), r3.NewVec(1, 2, 3)
	copyBody, err := Copy(t.Context(), source, WithCopyRotation(center, axis, units.Degrees(37)))
	require.NoError(t, err)
	require.Contains(t, doc.Bodies(), source)
	require.Contains(t, doc.Bodies(), copyBody)
	require.Len(t, doc.Bodies(), 2)
	require.Len(t, copyBody.Lumps(), 1)
	require.Len(t, copyBody.Shells(), 2)
	require.False(t, copyBody.Shells()[0].IsOpen())
	require.False(t, copyBody.Shells()[1].IsOpen())

	copyPayload := copyBody.n.Payload.(facetedPayload)
	require.NotNil(t, copyPayload.ExactCopyMoments)
	require.Nil(t, copyBody.n.VolumeLineage)
	require.NotNil(t, copyPayload.CopyLineage)
	require.True(t, copyPayload.CopyLineage.Direct)
	require.Same(t, &source.n, copyPayload.CopyLineage.Source)
	require.Equal(t, source.n.Origin.Producer, copyPayload.CopyLineage.SourceProducer)
	require.True(t, copyPayload.ExactPlaced)
	require.NotNil(t, copyPayload.CopyLineage.Exact)

	historyAfter := doc.OperationHistory().Operations()
	require.Len(t, historyAfter, len(historyBefore)+1)
	copyOperation := historyAfter[len(historyAfter)-1]
	require.Equal(t, OperationCopy, copyOperation.Kind())
	require.Equal(t, []*Body{copyBody}, copyOperation.Outputs())
	require.Len(t, copyOperation.Inputs(), 1)
	require.Same(t, source, copyOperation.Inputs()[0].Body)
	require.Equal(t, InputReferenced, copyOperation.Inputs()[0].Use)

	density := units.KilogramsPerCubicMillimeter(1)
	ordinary, err := exactCopyOrdinaryMassProperties(t, copyBody, density)
	require.NoError(t, err)
	got, err := copyBody.MassProperties(t.Context(), density)
	require.NoError(t, err)

	rotation := copyPayload.CopyLineage.Exact
	reference := rotateContainedCutMassReference(makeContainedCutMassReference(target, tool),
		rotation.Linear(), rotation.Translation())
	requireMeasurementContainsInterval(t, got.Mass, reference.mass)
	requireMeasurementContainsInterval(t, ordinary.Mass, reference.mass)
	for i, reading := range [6]Measurement{
		got.Inertia.XX, got.Inertia.YY, got.Inertia.ZZ, got.Inertia.XY, got.Inertia.XZ, got.Inertia.YZ,
	} {
		requireMeasurementContainsInterval(t, reading, reference.inertia[i])
		requireMeasurementContainsInterval(t, ordinary.Tensor[i], reference.inertia[i])
	}
	requireCenterBallContainsIntervals(t, got.Center, reference.center)
	requireCenterBallContainsIntervals(t, ordinary.Center, reference.center)

	gotBounds := [8]float64{
		got.Mass.Bound.Base(), got.Center.Bound.Base(),
		got.Inertia.XX.Bound.Base(), got.Inertia.YY.Bound.Base(), got.Inertia.ZZ.Bound.Base(),
		got.Inertia.XY.Bound.Base(), got.Inertia.XZ.Bound.Base(), got.Inertia.YZ.Bound.Base(),
	}
	ordinaryBounds := [8]float64{
		ordinary.Mass.Bound.Base(), ordinary.Center.Bound.Base(),
		ordinary.Tensor[0].Bound.Base(), ordinary.Tensor[1].Bound.Base(), ordinary.Tensor[2].Bound.Base(),
		ordinary.Tensor[3].Bound.Base(), ordinary.Tensor[4].Bound.Base(), ordinary.Tensor[5].Bound.Base(),
	}
	for i := range gotBounds {
		require.LessOrEqual(t, gotBounds[i], ordinaryBounds[i], "mass property component %d", i)
	}
	require.True(t, gotBounds[2] < ordinaryBounds[2] || gotBounds[3] < ordinaryBounds[3] ||
		gotBounds[4] < ordinaryBounds[4], "at least one diagonal inertia bound must shrink")
	require.Len(t, doc.OperationHistory().Operations(), len(historyAfter))
}

func TestExactCopyMomentHistoryAndRetiredSource(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	source, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	copyBody, err := Copy(t.Context(), source, WithCopyRotation(r3.NewVec(10, 10, 10),
		r3.NewVec(1, 2, 3), units.Degrees(37)))
	require.NoError(t, err)
	before, err := copyBody.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(1))
	require.NoError(t, err)
	historyBefore := doc.OperationHistory().Operations()
	producer, err := doc.OperationHistory().Producer(source)
	require.NoError(t, err)
	require.NoError(t, Remove(source))
	after, err := copyBody.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(1))
	require.NoError(t, err)
	require.Equal(t, before.Mass, after.Mass)
	require.Equal(t, before.Center, after.Center)
	require.Equal(t, before.Inertia, after.Inertia)
	operations := doc.OperationHistory().Operations()
	require.Len(t, operations, len(historyBefore)+1)
	require.Equal(t, OperationRemove, operations[len(operations)-1].Kind())
	retiredProducer, err := doc.OperationHistory().Producer(source)
	require.NoError(t, err)
	require.Same(t, producer, retiredProducer)
	require.Contains(t, doc.Bodies(), copyBody)
}

func TestExactCopyMomentEvidenceRollback(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	source, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	source.n.VolumeLineage.Target.Value = units.CubicMillimeters(
		source.n.VolumeLineage.Target.Value.Base() + 1)
	bodies := doc.Bodies()
	operations := doc.OperationHistory().Operations()
	nextProducer := doc.nextProducerID()
	_, err = Copy(t.Context(), source, WithCopyRotation(r3.NewVec(10, 10, 10),
		r3.NewVec(1, 2, 3), units.Degrees(37)))
	require.ErrorIs(t, err, ErrBooleanFailed)
	require.Len(t, doc.Bodies(), len(bodies))
	for i := range bodies {
		require.Same(t, bodies[i], doc.Bodies()[i])
	}
	require.Equal(t, nextProducer, doc.nextProducerID())
	current := doc.OperationHistory().Operations()
	require.Len(t, current, len(operations))
	for i := range operations {
		require.Same(t, operations[i], current[i])
	}
}

func TestExactCopyMomentScope(t *testing.T) {
	_, target, tool := containedCutCentroidSources(t)
	source, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	ordinary, err := Copy(t.Context(), source)
	require.NoError(t, err)
	require.Nil(t, ordinary.n.Payload.(facetedPayload).ExactCopyMoments)

	rotation := WithCopyRotation(r3.NewVec(10, 10, 10), r3.NewVec(1, 2, 3), units.Degrees(37))
	copyOfCopy, err := Copy(t.Context(), ordinary, rotation)
	require.NoError(t, err)
	require.Nil(t, copyOfCopy.n.Payload.(facetedPayload).ExactCopyMoments)

	_, floatRotation, err := exactrotation.New(r3.NewVec(10, 10, 10), r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	floatPlaced, err := Copy(t.Context(), source, WithCopyPlacement(floatRotation))
	require.NoError(t, err)
	require.Nil(t, floatPlaced.n.Payload.(facetedPayload).ExactCopyMoments)

	lastPlacement, err := Copy(t.Context(), source, rotation, WithCopyPlacement(floatRotation))
	require.NoError(t, err)
	require.Nil(t, lastPlacement.n.Payload.(facetedPayload).ExactCopyMoments)

	zero, err := Copy(t.Context(), source,
		WithCopyRotation(r3.Vec{}, r3.NewVec(1, 0, 0), units.Degrees(0)))
	require.NoError(t, err)
	require.NotNil(t, zero.n.Payload.(facetedPayload).ExactCopyMoments)
}

func TestExactCopyMassQueriesAreReadOnly(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	source, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	copyBody, err := Copy(t.Context(), source, WithCopyRotation(r3.NewVec(10, 10, 10),
		r3.NewVec(1, 2, 3), units.Degrees(37)))
	require.NoError(t, err)
	density1, density2 := units.KilogramsPerCubicMillimeter(1), units.KilogramsPerCubicMillimeter(2)
	base, err := copyBody.MassProperties(t.Context(), density1)
	require.NoError(t, err)
	doubled, err := copyBody.MassProperties(t.Context(), density2)
	require.NoError(t, err)
	require.Equal(t, base.Center, doubled.Center)
	require.Equal(t, 2*base.Mass.Value.Base(), doubled.Mass.Value.Base())
	require.Equal(t, 2*base.Mass.Bound.Base(), doubled.Mass.Bound.Base())
	baseTensor := [6]Measurement{base.Inertia.XX, base.Inertia.YY, base.Inertia.ZZ,
		base.Inertia.XY, base.Inertia.XZ, base.Inertia.YZ}
	doubledTensor := [6]Measurement{doubled.Inertia.XX, doubled.Inertia.YY, doubled.Inertia.ZZ,
		doubled.Inertia.XY, doubled.Inertia.XZ, doubled.Inertia.YZ}
	for i := range baseTensor {
		require.Equal(t, 2*baseTensor[i].Value.Base(), doubledTensor[i].Value.Base())
		require.Equal(t, 2*baseTensor[i].Bound.Base(), doubledTensor[i].Bound.Base())
	}
	bodies := doc.Bodies()
	history := doc.OperationHistory().Operations()
	type queryResult struct {
		readings MassProperties
		err      error
	}
	results := make(chan queryResult, 12)
	var wait sync.WaitGroup
	for i := range 8 {
		wait.Go(func() {
			readings, queryErr := copyBody.MassProperties(t.Context(), density1)
			results <- queryResult{readings: readings, err: queryErr}
		})
		if i%2 == 0 {
			wait.Go(func() {
				readings, queryErr := copyBody.MassProperties(t.Context(), density2)
				results <- queryResult{readings: readings, err: queryErr}
			})
		}
	}
	wait.Wait()
	close(results)
	for result := range results {
		require.NoError(t, result.err)
		if result.readings.Mass.Value.Base() == base.Mass.Value.Base() {
			require.Equal(t, base, result.readings)
		} else {
			require.Equal(t, doubled, result.readings)
		}
	}
	require.Equal(t, bodies, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), len(history))
	for i := range history {
		require.Same(t, history[i], doc.OperationHistory().Operations()[i])
	}
}

func exactCopyOrdinaryMassProperties(t *testing.T, body *Body, density units.Value) (massmoment.MassReadings, error) {
	t.Helper()
	payload := body.n.Payload.(facetedPayload)
	original := payload
	defer func() { body.n.Payload = original }()
	payload.ExactCopyMoments = nil
	body.n.Payload = payload
	return massdispatch.Properties(t.Context(), verifyAllMesher{}, &body.n, density)
}

func rotateContainedCutMassReference(source containedCutMassReference, linear [3][3]*big.Rat,
	translation [3]*big.Rat) containedCutMassReference {
	result := containedCutMassReference{mass: source.mass}
	for i := range 3 {
		result.center[i] = proofbound.PointInterval(translation[i])
		for j := range 3 {
			result.center[i] = proofbound.IntervalAdd(result.center[i],
				proofbound.IntervalScale(source.center[j], linear[i][j]))
		}
	}
	var inertia [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			inertia[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	inertia[0][0], inertia[1][1], inertia[2][2] =
		source.inertia[0], source.inertia[1], source.inertia[2]
	inertia[0][1], inertia[1][0] = source.inertia[3], source.inertia[3]
	inertia[0][2], inertia[2][0] = source.inertia[4], source.inertia[4]
	inertia[1][2], inertia[2][1] = source.inertia[5], source.inertia[5]
	var rotated [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			entry := proofbound.PointInterval(new(big.Rat))
			for k := range 3 {
				for l := range 3 {
					term := proofbound.IntervalScale(inertia[k][l],
						new(big.Rat).Mul(linear[i][k], linear[j][l]))
					entry = proofbound.IntervalAdd(entry, term)
				}
			}
			rotated[i][j], rotated[j][i] = entry, entry
		}
	}
	result.inertia = [6]proofbound.RatInterval{
		rotated[0][0], rotated[1][1], rotated[2][2], rotated[0][1], rotated[0][2], rotated[1][2],
	}
	return result
}
