package prooflineage

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutVolumeSelectsOnlyAConsistentTighterInterval(t *testing.T) {
	target := measurement.Measurement{Value: units.CubicMillimeters(100)}
	tool := measurement.Measurement{Value: units.CubicMillimeters(20)}
	ordinary := measurement.Measurement{
		Value:     units.CubicMillimeters(80.1),
		Exactness: measurement.Approximate,
		Bound:     units.CubicMillimeters(0.2),
	}
	containment := Containment{
		BoundaryGapLower:    units.Millimeters(1),
		InnerShellWitnesses: 2,
		OuterShellWitnesses: 1,
	}
	lineage, selected, err := ContainedCutVolume(4, 5, target, tool, ordinary, containment)
	require.NoError(t, err)
	require.True(t, selected)
	require.Equal(t, 4, lineage.TargetProducer)
	require.Equal(t, 5, lineage.ToolProducer)
	require.Equal(t, 80.0, lineage.Derived.Value.Base())
	require.Zero(t, lineage.Derived.Bound.Base())
	require.Equal(t, 80.1, lineage.Ordinary.Value.Base())

	t.Run("bounded sources charge subtraction", func(t *testing.T) {
		boundedTarget := target
		boundedTarget.Bound = units.CubicMillimeters(0.01)
		boundedTool := tool
		boundedTool.Bound = units.CubicMillimeters(0.02)
		lineage, selected, err := ContainedCutVolume(4, 5, boundedTarget, boundedTool, ordinary, containment)
		require.NoError(t, err)
		require.True(t, selected)
		require.Greater(t, lineage.Derived.Bound.Base(), 0.03)
		require.Equal(t, measurement.Approximate, lineage.Derived.Exactness)
	})

	t.Run("equal bound keeps ordinary result", func(t *testing.T) {
		tied := ordinary
		tied.Value = units.CubicMillimeters(80)
		tied.Bound = units.CubicMillimeters(0)
		_, selected, err := ContainedCutVolume(4, 5, target, tool, tied, containment)
		require.NoError(t, err)
		require.False(t, selected)
	})

	t.Run("disjoint intervals reject the certificate", func(t *testing.T) {
		separate := ordinary
		separate.Value = units.CubicMillimeters(100)
		_, selected, err := ContainedCutVolume(4, 5, target, tool, separate, containment)
		require.ErrorIs(t, err, ErrDisjointIntervals)
		require.False(t, selected)
	})
}
