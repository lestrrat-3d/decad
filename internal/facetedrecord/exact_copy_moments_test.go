package facetedrecord

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExactCopyMomentRecordFreezesSources(t *testing.T) {
	source, result, parent, rotation, placement := exactCopyMomentFixture(t)
	record, err := NewExactCopyMomentRecord(source, result, r3.Identity(), placement, parent, rotation)
	require.NoError(t, err)
	parent.Target.Value = units.CubicMillimeters(99)
	parent.Containment.BoundaryGapLower = units.Millimeters(99)

	first, linear, translation, err := record.Read(result, placement)
	require.NoError(t, err)
	require.Equal(t, big.NewRat(8, 1), first.Volume.Lo)
	linear[0][0].SetInt64(99)
	translation[0].SetInt64(99)
	first.Volume.Lo.SetInt64(99)
	second, linearAgain, translationAgain, err := record.Read(result, placement)
	require.NoError(t, err)
	require.Equal(t, big.NewRat(8, 1), second.Volume.Lo)
	require.NotEqual(t, int64(99), linearAgain[0][0].Num().Int64())
	require.NotEqual(t, int64(99), translationAgain[0].Num().Int64())
}

func TestExactCopyMomentRecordRejectsIdentityAndPlacement(t *testing.T) {
	source, result, parent, rotation, placement := exactCopyMomentFixture(t)
	record, err := NewExactCopyMomentRecord(source, result, r3.Identity(), placement, parent, rotation)
	require.NoError(t, err)
	badProducerSnapshot, badProducerMatrix, badProducerTranslation, err := record.Read(result+1, placement)
	require.Empty(t, badProducerSnapshot.Anchor)
	require.Nil(t, badProducerMatrix[0][0])
	require.Nil(t, badProducerTranslation[0])
	require.ErrorIs(t, err, ErrInvalidExactCopyMoments)
	badPlacementSnapshot, badPlacementMatrix, badPlacementTranslation, err := record.Read(result, r3.Identity())
	require.Empty(t, badPlacementSnapshot.Anchor)
	require.Nil(t, badPlacementMatrix[0][0])
	require.Nil(t, badPlacementTranslation[0])
	require.ErrorIs(t, err, ErrInvalidExactCopyMoments)

	record.parent.Moments = nil
	missingSnapshot, missingMatrix, missingTranslation, err := record.Read(result, placement)
	require.Empty(t, missingSnapshot.Anchor)
	require.Nil(t, missingMatrix[0][0])
	require.Nil(t, missingTranslation[0])
	require.ErrorIs(t, err, ErrInvalidExactCopyMoments)
}

func TestExactCopyMomentRecordValidatesExactRotation(t *testing.T) {
	source, result, parent, _, placement := exactCopyMomentFixture(t)
	_, err := NewExactCopyMomentRecord(source, result, r3.Identity(), placement, parent, nil)
	require.ErrorIs(t, err, ErrInvalidExactCopyMoments)
	_, err = NewExactCopyMomentRecord(source, source, r3.Identity(), placement, parent, func() *exactrotation.Rotation {
		rotation, _, rotationErr := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(10))
		require.NoError(t, rotationErr)
		return rotation
	}())
	require.ErrorIs(t, err, ErrInvalidExactCopyMoments)
}

func exactCopyMomentFixture(t *testing.T) (provenance.Producer, provenance.Producer,
	*prooflineage.VolumeLineage, *exactrotation.Rotation, r3.Transform) {
	t.Helper()
	source, result := provenance.Producer(1), provenance.Producer(3)
	targetProducer, toolProducer := 1, 2
	parent := &prooflineage.VolumeLineage{
		TargetProducer: targetProducer, ToolProducer: toolProducer,
		Target: exactCopyVolumeReading(10), Tool: exactCopyVolumeReading(2),
		Ordinary: exactCopyVolumeReading(8), Derived: exactCopyVolumeReading(8),
		Containment: prooflineage.Containment{BoundaryGapLower: units.Millimeters(1),
			InnerShellWitnesses: 1, OuterShellWitnesses: 1},
	}
	target, tool := exactCopySnapshot(10), exactCopySnapshot(2)
	var err error
	parent.Moments, err = prooflineage.NewContainedCutMoments(int(source), parent,
		prooflineage.SourceMoments{Producer: targetProducer, Snapshot: target},
		prooflineage.SourceMoments{Producer: toolProducer, Snapshot: tool})
	require.NoError(t, err)
	rotation, placement, err := exactrotation.New(r3.Vec{X: 8, Y: -3}, r3.Vec{X: 1, Y: 2, Z: 1}, units.Degrees(33))
	require.NoError(t, err)
	return source, result, parent, rotation, placement
}

func exactCopyVolumeReading(volume float64) measurement.Measurement {
	return measurement.Measurement{Value: units.CubicMillimeters(volume), Bound: units.CubicMillimeters(0),
		Exactness: measurement.Exact}
}

func exactCopySnapshot(volume int64) prooflineage.MomentSnapshot {
	snapshot := prooflineage.MomentSnapshot{Anchor: r3.Vec{}, Volume: proofbound.PointInterval(big.NewRat(volume, 1))}
	for i := range 3 {
		snapshot.First[i] = proofbound.PointInterval(new(big.Rat))
		for j := range 3 {
			snapshot.Second[i][j] = proofbound.PointInterval(new(big.Rat))
		}
	}
	return snapshot
}
