package prooflineage

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutCentroidUsesBoundedSourceFirstMoments(t *testing.T) {
	targetVolume := exactVolume(100)
	toolVolume := exactVolume(20)
	lineage := centroidVolumeLineage(t, targetVolume, toolVolume, boundedVolume(80, 0.2))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 5}, 0)
	ordinary := boundedCentroid(r3.Vec{X: -1.25}, 0.2)

	centroidLineage, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.NoError(t, err)
	require.True(t, selected)
	require.NotNil(t, centroidLineage)
	require.Equal(t, r3.Vec{X: -1.25}, centroidLineage.Derived.Value)
	require.Zero(t, centroidLineage.Derived.Bound.Base())
	require.Equal(t, measurement.Exact, centroidLineage.Derived.Exactness)
	require.Equal(t, ordinary, centroidLineage.Ordinary)
	require.True(t, centroidLineage.Selected)
}

func TestContainedCutCentroidCarriesSourceAndArithmeticBounds(t *testing.T) {
	targetVolume := boundedVolume(100, 0.01)
	toolVolume := boundedVolume(20, 0.02)
	lineage := centroidVolumeLineage(t, targetVolume, toolVolume, boundedVolume(80, 0.1))
	target := sourceCentroid(4, r3.Vec{X: 10, Y: 10, Z: 10}, 0.001)
	tool := sourceCentroid(5, r3.Vec{X: 12, Y: 10, Z: 10}, 0.002)
	ordinary := boundedCentroid(r3.Vec{X: 9.5, Y: 10, Z: 10}, 1)

	centroidLineage, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.NoError(t, err)
	require.True(t, selected)
	require.Greater(t, centroidLineage.Derived.Bound.Base(), 0.001)
	require.Less(t, centroidLineage.Derived.Bound.Base(), ordinary.Bound.Base())
	require.InDelta(t, 9.5, centroidLineage.Derived.Value.X, centroidLineage.Derived.Bound.Base())
	require.InDelta(t, 10, centroidLineage.Derived.Value.Y, centroidLineage.Derived.Bound.Base())
	require.InDelta(t, 10, centroidLineage.Derived.Value.Z, centroidLineage.Derived.Bound.Base())
}

func TestContainedCutCentroidFallsBackWhenVolumeDifferenceStraddlesZero(t *testing.T) {
	lineage := centroidVolumeLineage(t,
		boundedVolume(10, 4), boundedVolume(5, 1), boundedVolume(5, 5))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 1}, 0)
	ordinary := boundedCentroid(r3.Vec{X: -1}, 1)

	got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.NoError(t, err)
	require.False(t, selected)
	require.Nil(t, got)
}

func TestContainedCutCentroidRejectsNonpositiveVolumeDifference(t *testing.T) {
	lineage := centroidVolumeLineage(t, exactVolume(4), exactVolume(5), boundedVolume(-1, 2))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 1}, 0)
	ordinary := boundedCentroid(r3.Vec{}, 1)

	got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.ErrorIs(t, err, ErrInvalidCentroidEvidence)
	require.False(t, selected)
	require.Nil(t, got)
}

func TestContainedCutCentroidFallsBackWhenDerivedArithmeticOverflows(t *testing.T) {
	lineage := centroidVolumeLineage(t, exactVolume(100), exactVolume(20), boundedVolume(80, 1))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: math.MaxFloat64}, 0)
	ordinary := boundedCentroid(r3.Vec{}, math.MaxFloat64)

	got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.NoError(t, err)
	require.False(t, selected)
	require.Nil(t, got)
}

func TestContainedCutCentroidKeepsOrdinaryOnEqualOrWorseBound(t *testing.T) {
	t.Run("equal", func(t *testing.T) {
		lineage := centroidVolumeLineage(t, exactVolume(100), exactVolume(20), exactVolume(80))
		target := sourceCentroid(4, r3.Vec{}, 0)
		tool := sourceCentroid(5, r3.Vec{X: 5}, 0)
		ordinary := boundedCentroid(r3.Vec{X: -1.25}, 0)

		got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
		require.NoError(t, err)
		require.False(t, selected)
		require.NotNil(t, got)
		require.False(t, got.Selected)
	})

	t.Run("worse", func(t *testing.T) {
		lineage := centroidVolumeLineage(t,
			boundedVolume(100, 0.01), boundedVolume(20, 0.02), exactVolume(80))
		target := sourceCentroid(4, r3.Vec{}, 0.1)
		tool := sourceCentroid(5, r3.Vec{X: 5}, 0.1)
		ordinary := boundedCentroid(r3.Vec{X: -1.25}, 0)

		got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
		require.NoError(t, err)
		require.False(t, selected)
		require.NotNil(t, got)
		require.False(t, got.Selected)
	})
}

func TestContainedCutCentroidComparesBallsByExactCenterDistance(t *testing.T) {
	lineage := centroidVolumeLineage(t, exactVolume(100), exactVolume(20), boundedVolume(80, 0.2))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 5}, 0)
	ordinary := boundedCentroid(r3.Vec{X: -1.25, Y: 10}, 0)

	got, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.ErrorIs(t, err, ErrDisjointCentroidBounds)
	require.False(t, selected)
	require.Nil(t, got)
}

func TestContainedCutCentroidSelectionDoesNotDependOnVolumeSelection(t *testing.T) {
	targetVolume := boundedVolume(100, 0.01)
	toolVolume := boundedVolume(20, 0.02)
	lineage := centroidVolumeLineage(t, targetVolume, toolVolume, exactVolume(80))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 5}, 0)
	ordinary := boundedCentroid(r3.Vec{X: -1.25}, 1)

	centroidLineage, selected, err := ContainedCutCentroid(lineage, target, tool, ordinary)
	require.NoError(t, err)
	require.False(t, lineage.Derived.Bound.Base() < lineage.Ordinary.Bound.Base())
	require.True(t, selected)
	require.True(t, centroidLineage.Selected)
}

func TestContainedCutCentroidRejectsCorruptEvidence(t *testing.T) {
	baseLineage := centroidVolumeLineage(t, exactVolume(100), exactVolume(20), boundedVolume(80, 0.2))
	target := sourceCentroid(4, r3.Vec{}, 0)
	tool := sourceCentroid(5, r3.Vec{X: 5}, 0)
	ordinary := boundedCentroid(r3.Vec{X: -1.25}, 0.2)

	tests := []struct {
		name   string
		mutate func(*VolumeLineage, *SourceCentroid, *SourceCentroid, *measurement.VecMeasurement)
	}{
		{name: "target producer mismatch", mutate: func(_ *VolumeLineage, target *SourceCentroid,
			_ *SourceCentroid, _ *measurement.VecMeasurement) {
			target.Producer++
		}},
		{name: "stored derived volume mismatch", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Derived.Value = units.CubicMillimeters(81)
		}},
		{name: "volume unit mismatch", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Target.Value = units.Millimeters(100)
		}},
		{name: "nonfinite source volume", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Target.Value = units.CubicMillimeters(math.NaN())
		}},
		{name: "infinite source volume bound", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Target.Bound = units.CubicMillimeters(math.Inf(1))
		}},
		{name: "negative source bound", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Target.Bound = units.CubicMillimeters(-1)
		}},
		{name: "wrong containment length unit", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Containment.BoundaryGapLower = units.CubicMillimeters(1)
		}},
		{name: "nonfinite centroid", mutate: func(_ *VolumeLineage,
			target *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			target.Reading.Value.X = math.Inf(1)
		}},
		{name: "nan centroid", mutate: func(_ *VolumeLineage,
			target *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			target.Reading.Value.Y = math.NaN()
		}},
		{name: "negative centroid bound", mutate: func(_ *VolumeLineage,
			target *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			target.Reading.Bound = units.Millimeters(-1)
		}},
		{name: "displaced exactness", mutate: func(_ *VolumeLineage,
			target *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			target.Reading.Exactness = measurement.Approximate
		}},
		{name: "derived volume exactness mismatch", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Derived.Exactness = measurement.Approximate
		}},
		{name: "missing containment witness", mutate: func(lineage *VolumeLineage,
			_ *SourceCentroid, _ *SourceCentroid, _ *measurement.VecMeasurement) {
			lineage.Containment.InnerShellWitnesses = 0
		}},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			lineage := *baseLineage
			target, tool, ordinary := target, tool, ordinary
			test.mutate(&lineage, &target, &tool, &ordinary)
			got, selected, err := ContainedCutCentroid(&lineage, target, tool, ordinary)
			require.ErrorIs(t, err, ErrInvalidCentroidEvidence)
			require.False(t, selected)
			require.Nil(t, got)
		})
	}
}

func centroidVolumeLineage(t *testing.T, target, tool, ordinary measurement.Measurement) *VolumeLineage {
	t.Helper()
	lineage, _, err := ContainedCutVolume(4, 5, target, tool, ordinary, Containment{
		BoundaryGapLower:    units.Millimeters(1),
		InnerShellWitnesses: 2,
		OuterShellWitnesses: 1,
	})
	require.NoError(t, err)
	return lineage
}

func exactVolume(value float64) measurement.Measurement {
	return measurement.Measurement{
		Value: units.CubicMillimeters(value), Exactness: measurement.Exact,
		Bound: units.CubicMillimeters(0),
	}
}

func boundedVolume(value, bound float64) measurement.Measurement {
	return measurement.Measurement{
		Value: units.CubicMillimeters(value), Exactness: measurement.ExactnessOf(bound),
		Bound: units.CubicMillimeters(bound),
	}
}

func sourceCentroid(producer int, value r3.Vec, bound float64) SourceCentroid {
	return SourceCentroid{Producer: producer, Reading: boundedCentroid(value, bound)}
}

func boundedCentroid(value r3.Vec, bound float64) measurement.VecMeasurement {
	return measurement.VecMeasurement{
		Value: value, Exactness: measurement.ExactnessOf(bound), Bound: units.Millimeters(bound),
	}
}
