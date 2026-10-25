package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutSourceMassProperties(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	result, err := Cut(t.Context(), target, tool)
	require.NoError(t, err)
	require.NotNil(t, result.n.VolumeLineage)
	require.NotNil(t, result.n.VolumeLineage.Moments)
	require.Len(t, result.Lumps(), 1)
	require.Len(t, result.Shells(), 2)
	require.False(t, result.Shells()[0].IsOpen())
	require.False(t, result.Shells()[1].IsOpen())

	density := units.KilogramsPerCubicMillimeter(1)
	lineage := result.n.VolumeLineage
	result.n.VolumeLineage = nil
	ordinary, err := massdispatch.Properties(t.Context(), verifyAllMesher{}, &result.n, density)
	result.n.VolumeLineage = lineage
	require.NoError(t, err)

	got, err := result.MassProperties(t.Context(), density)
	require.NoError(t, err)
	derivedBounds := [8]float64{
		got.Mass.Bound.Base(), got.Center.Bound.Base(),
		got.Inertia.XX.Bound.Base(), got.Inertia.YY.Bound.Base(), got.Inertia.ZZ.Bound.Base(),
		got.Inertia.XY.Bound.Base(), got.Inertia.XZ.Bound.Base(), got.Inertia.YZ.Bound.Base(),
	}
	ordinaryBounds := [8]float64{
		ordinary.Mass.Bound.Base(), ordinary.Center.Bound.Base(),
		ordinary.Tensor[0].Bound.Base(), ordinary.Tensor[1].Bound.Base(), ordinary.Tensor[2].Bound.Base(),
		ordinary.Tensor[3].Bound.Base(), ordinary.Tensor[4].Bound.Base(), ordinary.Tensor[5].Bound.Base(),
	}
	strict := false
	for i := range derivedBounds {
		require.LessOrEqual(t, derivedBounds[i], ordinaryBounds[i], "mass property component %d", i)
		strict = strict || derivedBounds[i] < ordinaryBounds[i]
	}
	require.True(t, strict)
	comparison := [3]bool{
		got.Inertia.XX.Bound.Base() < ordinary.Tensor[0].Bound.Base(),
		got.Inertia.YY.Bound.Base() < ordinary.Tensor[1].Bound.Base(),
		got.Inertia.ZZ.Bound.Base() < ordinary.Tensor[2].Bound.Base(),
	}
	require.True(t, comparison[0] || comparison[1] || comparison[2], "at least one diagonal inertia bound must shrink")

	reference := makeContainedCutMassReference(target, tool)
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
	require.Len(t, doc.OperationHistory().Operations(), 4)
}

func TestContainedCutMomentEvidenceRollback(t *testing.T) {
	doc, target, tool := containedCutCentroidSources(t)
	target.n.Volume.Value = units.CubicMillimeters(target.n.Volume.Value.Base() + 1e-6)
	bodiesBefore := doc.Bodies()
	operationsBefore := doc.OperationHistory().Operations()
	producerBefore := doc.nextProducerID()
	_, err := Cut(t.Context(), target, tool)
	require.ErrorContains(t, err, "contained-cut moment evidence is invalid")
	require.ErrorContains(t, err, "target source volume")
	require.ErrorIs(t, err, decaderr.ErrBooleanFailed)

	require.Len(t, doc.Bodies(), len(bodiesBefore))
	for i := range bodiesBefore {
		require.Same(t, bodiesBefore[i], doc.Bodies()[i])
	}
	require.Equal(t, producerBefore, doc.nextProducerID())
	after := doc.OperationHistory().Operations()
	require.Len(t, after, len(operationsBefore))
	for i := range operationsBefore {
		require.Same(t, operationsBefore[i], after[i])
	}
}

type containedCutMassReference struct {
	mass    proofbound.RatInterval
	center  [3]proofbound.RatInterval
	inertia [6]proofbound.RatInterval
}

func makeContainedCutMassReference(target, tool *Body) containedCutMassReference {
	targetRecord := target.n.Payload.(prismrecord.Payload)
	toolRecord := tool.n.Payload.(prismrecord.Payload)
	targetMap, targetTranslation := containedCutReferenceMap(targetRecord)
	toolMap, toolTranslation := containedCutReferenceMap(toolRecord)
	targetDet := new(big.Rat).Abs(containedCutReferenceDeterminant(targetMap))
	toolDet := new(big.Rat).Abs(containedCutReferenceDeterminant(toolMap))
	targetMinU, targetMaxU, targetMinV, targetMaxV := containedCutRectangleBounds(targetRecord)
	targetWidth := new(big.Rat).Sub(proofarith.FloatRat(targetMaxU), proofarith.FloatRat(targetMinU))
	targetDepth := new(big.Rat).Sub(proofarith.FloatRat(targetMaxV), proofarith.FloatRat(targetMinV))
	targetHeight := new(big.Rat).Sub(proofarith.FloatRat(targetRecord.Z1), proofarith.FloatRat(targetRecord.Z0))
	targetLocalCenter := [3]*big.Rat{
		new(big.Rat).Quo(new(big.Rat).Add(proofarith.FloatRat(targetMinU), proofarith.FloatRat(targetMaxU)), big.NewRat(2, 1)),
		new(big.Rat).Quo(new(big.Rat).Add(proofarith.FloatRat(targetMinV), proofarith.FloatRat(targetMaxV)), big.NewRat(2, 1)),
		new(big.Rat).Quo(new(big.Rat).Add(proofarith.FloatRat(targetRecord.Z0), proofarith.FloatRat(targetRecord.Z1)), big.NewRat(2, 1)),
	}
	targetVolume := new(big.Rat).Mul(new(big.Rat).Mul(targetWidth, targetDepth), targetHeight)
	targetCentral := [3][3]proofbound.RatInterval{}
	for i := range 3 {
		for j := range 3 {
			targetCentral[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	targetCentral[0][0] = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Mul(targetVolume, new(big.Rat).Mul(targetWidth, targetWidth)), big.NewRat(12, 1)))
	targetCentral[1][1] = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Mul(targetVolume, new(big.Rat).Mul(targetDepth, targetDepth)), big.NewRat(12, 1)))
	targetCentral[2][2] = proofbound.PointInterval(new(big.Rat).Quo(new(big.Rat).Mul(targetVolume, new(big.Rat).Mul(targetHeight, targetHeight)), big.NewRat(12, 1)))
	targetMoments := buildContainedCutReferenceMoments(targetMap, targetTranslation, targetDet,
		proofbound.PointInterval(targetVolume), targetCentral, targetLocalCenter, target.n.Centroid.Value)

	circle := toolRecord.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	toolVolume := proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper),
		new(big.Rat).Mul(big.NewRat(4, 1), toolDet))
	radius := proofarith.FloatRat(circle.Radius.Base())
	toolHeight := new(big.Rat).Sub(proofarith.FloatRat(toolRecord.Z1), proofarith.FloatRat(toolRecord.Z0))
	toolLocalCenter := [3]*big.Rat{
		proofarith.FloatRat(circle.Center.U), proofarith.FloatRat(circle.Center.V),
		new(big.Rat).Quo(new(big.Rat).Add(proofarith.FloatRat(toolRecord.Z0), proofarith.FloatRat(toolRecord.Z1)), big.NewRat(2, 1)),
	}
	toolCentral := [3][3]proofbound.RatInterval{}
	for i := range 3 {
		for j := range 3 {
			toolCentral[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	toolCentral[0][0] = proofbound.IntervalScale(toolVolume, new(big.Rat).Quo(new(big.Rat).Mul(radius, radius), big.NewRat(4, 1)))
	toolCentral[1][1] = toolCentral[0][0]
	toolCentral[2][2] = proofbound.IntervalScale(toolVolume, new(big.Rat).Quo(new(big.Rat).Mul(toolHeight, toolHeight), big.NewRat(12, 1)))
	toolMoments := buildContainedCutReferenceMoments(toolMap, toolTranslation, toolDet,
		toolVolume, toolCentral, toolLocalCenter, target.n.Centroid.Value)

	volume := proofbound.IntervalSub(targetMoments.volume, toolMoments.volume)
	var first [3]proofbound.RatInterval
	var second [3][3]proofbound.RatInterval
	for i := range 3 {
		first[i] = proofbound.IntervalSub(targetMoments.first[i], toolMoments.first[i])
		for j := range 3 {
			second[i][j] = proofbound.IntervalSub(targetMoments.second[i][j], toolMoments.second[i][j])
		}
	}
	ref := containedCutMassReference{mass: volume}
	for i := range 3 {
		ref.center[i] = proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(component(target.n.Centroid.Value, i))),
			mustContainedCutQuotient(first[i], volume))
	}
	var central [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			product := proofbound.IntervalMul(first[i], first[j])
			central[i][j] = proofbound.IntervalSub(second[i][j], mustContainedCutQuotient(product, volume))
		}
	}
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(central[0][0], central[1][1]), central[2][2])
	for i, ij := range [6][2]int{{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}} {
		row, col := ij[0], ij[1]
		if row == col {
			ref.inertia[i] = proofbound.IntervalSub(trace, central[row][col])
		} else {
			ref.inertia[i] = proofbound.IntervalNeg(central[row][col])
		}
	}
	return ref
}

type containedCutMomentTuple struct {
	volume proofbound.RatInterval
	first  [3]proofbound.RatInterval
	second [3][3]proofbound.RatInterval
}

func buildContainedCutReferenceMoments(linear [3][3]*big.Rat, translation [3]*big.Rat, determinant *big.Rat,
	volume proofbound.RatInterval, central [3][3]proofbound.RatInterval, localCenter [3]*big.Rat,
	anchor r3.Vec) containedCutMomentTuple {
	center := [3]*big.Rat{}
	for i := range 3 {
		center[i] = new(big.Rat).Set(translation[i])
		for j := range 3 {
			center[i].Add(center[i], new(big.Rat).Mul(linear[i][j], localCenter[j]))
		}
	}
	var worldCentral [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			entry := proofbound.PointInterval(new(big.Rat))
			for k := range 3 {
				for l := range 3 {
					coefficient := new(big.Rat).Mul(linear[i][k], linear[j][l])
					coefficient.Mul(coefficient, determinant)
					entry = proofbound.IntervalAdd(entry, proofbound.IntervalScale(central[k][l], coefficient))
				}
			}
			worldCentral[i][j], worldCentral[j][i] = entry, entry
		}
	}
	delta := [3]*big.Rat{}
	for i := range 3 {
		delta[i] = new(big.Rat).Sub(center[i], proofarith.FloatRat(component(anchor, i)))
	}
	out := containedCutMomentTuple{volume: proofbound.IntervalScale(volume, determinant)}
	for i := range 3 {
		out.first[i] = proofbound.IntervalScale(out.volume, delta[i])
		for j := range 3 {
			out.second[i][j] = proofbound.IntervalAdd(worldCentral[i][j],
				proofbound.IntervalScale(out.volume, new(big.Rat).Mul(delta[i], delta[j])))
		}
	}
	return out
}

func containedCutReferenceMap(record prismrecord.Payload) ([3][3]*big.Rat, [3]*big.Rat) {
	basis := record.Xform.Basis()
	placement := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	frame := [3]r3.Vec{record.Frame.U(), record.Frame.V(), record.Frame.N()}
	var linear [3][3]*big.Rat
	for i := range 3 {
		for k := range 3 {
			linear[i][k] = new(big.Rat)
			for j := range 3 {
				linear[i][k].Add(linear[i][k], new(big.Rat).Mul(
					proofarith.FloatRat(component(placement[j], i)), proofarith.FloatRat(component(frame[k], j))))
			}
		}
	}
	translation := [3]*big.Rat{}
	for i := range 3 {
		translation[i] = proofarith.FloatRat(component(record.Xform.Translation(), i))
		for j := range 3 {
			translation[i].Add(translation[i], new(big.Rat).Mul(linear[i][j], proofarith.FloatRat(component(record.Frame.Origin(), j))))
		}
	}
	return linear, translation
}

func containedCutReferenceDeterminant(m [3][3]*big.Rat) *big.Rat {
	first := new(big.Rat).Mul(m[0][0], new(big.Rat).Sub(
		new(big.Rat).Mul(m[1][1], m[2][2]), new(big.Rat).Mul(m[1][2], m[2][1])))
	second := new(big.Rat).Mul(m[0][1], new(big.Rat).Sub(
		new(big.Rat).Mul(m[1][0], m[2][2]), new(big.Rat).Mul(m[1][2], m[2][0])))
	third := new(big.Rat).Mul(m[0][2], new(big.Rat).Sub(
		new(big.Rat).Mul(m[1][0], m[2][1]), new(big.Rat).Mul(m[1][1], m[2][0])))
	return new(big.Rat).Add(new(big.Rat).Sub(first, second), third)
}

func containedCutRectangleBounds(record prismrecord.Payload) (minU, maxU, minV, maxV float64) {
	first := true
	for _, curve := range record.Profile.Outer.Segments {
		line := curve.(sectionrecord.LineSeg)
		for _, point := range []sectionrecord.Point2{line.Start, line.End} {
			if first {
				minU, maxU, minV, maxV = point.U, point.U, point.V, point.V
				first = false
				continue
			}
			minU, maxU = math.Min(minU, point.U), math.Max(maxU, point.U)
			minV, maxV = math.Min(minV, point.V), math.Max(maxV, point.V)
		}
	}
	return minU, maxU, minV, maxV
}

func mustContainedCutQuotient(numerator, denominator proofbound.RatInterval) proofbound.RatInterval {
	quotient, ok := proofbound.IntervalQuo(numerator, denominator)
	if !ok {
		panic("independent contained-Cut reference volume must be positive")
	}
	return quotient
}

func component(value r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return value.X
	case 1:
		return value.Y
	default:
		return value.Z
	}
}

func requireMeasurementContainsInterval(t *testing.T, reading Measurement, reference proofbound.RatInterval) {
	t.Helper()
	center := proofarith.FloatRat(reading.Value.Base())
	bound := proofarith.FloatRat(reading.Bound.Base())
	lo, hi := new(big.Rat).Sub(center, bound), new(big.Rat).Add(center, bound)
	require.LessOrEqual(t, lo.Cmp(reference.Lo), 0, "reading lower endpoint must enclose reference lower endpoint")
	require.GreaterOrEqual(t, hi.Cmp(reference.Hi), 0, "reading upper endpoint must enclose reference upper endpoint")
}

func requireCenterBallContainsIntervals(t *testing.T, reading VecMeasurement, reference [3]proofbound.RatInterval) {
	t.Helper()
	coords := [3]float64{reading.Value.X, reading.Value.Y, reading.Value.Z}
	maxError := [3]*big.Rat{}
	for i, coordinate := range coords {
		difference := proofbound.IntervalSub(reference[i], proofbound.PointInterval(proofarith.FloatRat(coordinate)))
		maxError[i] = proofbound.IntervalAbsUpper(difference)
	}
	squared := new(big.Rat).Mul(maxError[0], maxError[0])
	squared.Add(squared, new(big.Rat).Mul(maxError[1], maxError[1]))
	squared.Add(squared, new(big.Rat).Mul(maxError[2], maxError[2]))
	radius := proofarith.FloatRat(reading.Bound.Base())
	require.LessOrEqual(t, squared.Cmp(new(big.Rat).Mul(radius, radius)), 0,
		"reference center box must fit inside the published 3D ball")
}
