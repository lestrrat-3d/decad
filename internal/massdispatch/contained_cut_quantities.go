package massdispatch

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prooflineage"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// ContainedCutEvidenceError marks a source-quantity derivation failure for the
// public Boolean adapter. Pair-query and final-context errors pass through
// without this wrapper; Unwrap returns the original derivation error.
type ContainedCutEvidenceError struct{ cause error }

func (e *ContainedCutEvidenceError) Error() string { return e.cause.Error() }

func (e *ContainedCutEvidenceError) Unwrap() error { return e.cause }

// CaptureContainedCutQuantities attaches certified source-volume, centroid and
// optional moment evidence to a completed faceted Cut. It publishes selected
// readings only after every derivation and the final context check succeeds.
// Unsupported containment or result classes keep their existing readings.
// ctx, result, target and tool must not be nil.
func CaptureContainedCutQuantities(ctx context.Context, result, target, tool *topo.Body) error {
	if _, ok := result.Payload.(facetedrecord.Payload); !ok {
		return nil
	}
	proof, err := pairkernel.Pair(ctx, target, tool, false, nil)
	if err != nil {
		return err
	}
	certificate := proof.Containment
	if certificate == nil || certificate.Inner != tool || certificate.Outer != target {
		return nil
	}
	volumeLineage, selectVolume, err := prooflineage.ContainedCutVolume(
		int(target.Origin.Producer), int(tool.Origin.Producer),
		target.Volume, tool.Volume, result.Volume,
		prooflineage.Containment{
			BoundaryGapLower:    units.Millimeters(certificate.BoundaryGapLower),
			InnerShellWitnesses: certificate.InnerShellWitnesses,
			OuterShellWitnesses: certificate.OuterShellWitnesses,
		},
	)
	if err != nil {
		return &ContainedCutEvidenceError{cause: err}
	}
	centroidLineage, selectCentroid, err := prooflineage.ContainedCutCentroid(
		volumeLineage,
		prooflineage.SourceCentroid{Producer: int(target.Origin.Producer), Reading: target.Centroid},
		prooflineage.SourceCentroid{Producer: int(tool.Origin.Producer), Reading: tool.Centroid},
		result.Centroid,
	)
	if err != nil {
		return &ContainedCutEvidenceError{cause: err}
	}
	momentLineage, err := CaptureContainedCutMoments(ctx, result, target, tool, volumeLineage)
	if err != nil {
		return &ContainedCutEvidenceError{cause: err}
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	volumeLineage.Centroid = centroidLineage
	volumeLineage.Moments = momentLineage
	result.VolumeLineage = volumeLineage
	if selectVolume {
		result.Volume = volumeLineage.Derived
	}
	if selectCentroid {
		result.Centroid = centroidLineage.Derived
	}
	return nil
}
