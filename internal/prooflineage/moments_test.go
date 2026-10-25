package prooflineage

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestContainedCutMomentsFreezesAndRevalidatesSources(t *testing.T) {
	parent, target, tool := momentFixture()
	got, err := NewContainedCutMoments(9, parent, target, tool)
	require.NoError(t, err)

	// Mutating constructor inputs cannot alter the retained rational endpoints.
	target.Snapshot.Volume.Lo.SetInt64(1000)
	readTarget, readTool, difference, err := got.Read(9, parent)
	require.NoError(t, err)
	require.Equal(t, int64(100), readTarget.Volume.Lo.Num().Int64())
	require.Equal(t, int64(20), readTool.Volume.Lo.Num().Int64())
	require.Equal(t, int64(80), difference.Volume.Lo.Num().Int64())

	// Returned snapshots are independent copies too.
	readTarget.Volume.Lo.SetInt64(-1)
	readTarget, _, _, err = got.Read(9, parent)
	require.NoError(t, err)
	require.Equal(t, int64(100), readTarget.Volume.Lo.Num().Int64())

	require.ErrorIs(t, func() error { _, _, _, err := got.Read(10, parent); return err }(), ErrInvalidContainedCutMoments)
	changed := *parent
	changed.ToolProducer++
	require.ErrorIs(t, func() error { _, _, _, err := got.Read(9, &changed); return err }(), ErrInvalidContainedCutMoments)
}

func TestContainedCutMomentsRejectsInvalidAndUnprovedVolume(t *testing.T) {
	t.Run("asymmetric matrix", func(t *testing.T) {
		parent, target, tool := momentFixture()
		tool.Snapshot.Second[0][1] = proofbound.PointInterval(big.NewRat(1, 1))
		_, err := NewContainedCutMoments(9, parent, target, tool)
		require.ErrorIs(t, err, ErrInvalidContainedCutMoments)
	})
	t.Run("difference straddles zero", func(t *testing.T) {
		parent, target, tool := momentFixture()
		tool.Snapshot.Volume = proofbound.Interval(big.NewRat(99, 1), big.NewRat(101, 1))
		_, err := NewContainedCutMoments(9, parent, target, tool)
		require.ErrorIs(t, err, ErrUnprovedContainedCutVolume)
	})
	t.Run("nonpositive difference", func(t *testing.T) {
		parent, target, tool := momentFixture()
		tool.Snapshot.Volume = proofbound.PointInterval(big.NewRat(101, 1))
		_, err := NewContainedCutMoments(9, parent, target, tool)
		require.ErrorIs(t, err, ErrInvalidContainedCutMoments)
	})
}

func momentFixture() (*VolumeLineage, SourceMoments, SourceMoments) {
	target := MomentSnapshot{Anchor: r3.Vec{}, Volume: proofbound.PointInterval(big.NewRat(100, 1))}
	tool := MomentSnapshot{Anchor: r3.Vec{}, Volume: proofbound.PointInterval(big.NewRat(20, 1))}
	for i := range 3 {
		target.First[i] = proofbound.PointInterval(new(big.Rat))
		tool.First[i] = proofbound.PointInterval(new(big.Rat))
		for j := range 3 {
			target.Second[i][j] = proofbound.PointInterval(new(big.Rat))
			tool.Second[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	parent := &VolumeLineage{
		TargetProducer: 4, ToolProducer: 5,
		Target:      measurement.Measurement{Value: units.CubicMillimeters(100), Bound: units.CubicMillimeters(0)},
		Tool:        measurement.Measurement{Value: units.CubicMillimeters(20), Bound: units.CubicMillimeters(0)},
		Ordinary:    measurement.Measurement{Value: units.CubicMillimeters(80), Bound: units.CubicMillimeters(0)},
		Derived:     measurement.Measurement{Value: units.CubicMillimeters(80), Bound: units.CubicMillimeters(0)},
		Containment: Containment{BoundaryGapLower: units.Millimeters(1), InnerShellWitnesses: 2, OuterShellWitnesses: 1},
	}
	return parent,
		SourceMoments{Producer: 4, Snapshot: target},
		SourceMoments{Producer: 5, Snapshot: tool}
}
