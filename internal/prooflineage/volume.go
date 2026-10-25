// Package prooflineage freezes compact evaluator evidence for quantity readers.
package prooflineage

import (
	"errors"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

var ErrDisjointIntervals = errors.New("contained-cut volume intervals do not overlap")

// Containment records the shell and clearance facts used by a strict
// containment certificate without retaining topology nodes.
type Containment struct {
	BoundaryGapLower    units.Value
	InnerShellWitnesses int
	OuterShellWitnesses int
}

// VolumeLineage retains the bounded source readings and both result intervals.
// It contains no body, face, document, or cache pointers.
type VolumeLineage struct {
	TargetProducer int
	ToolProducer   int
	Target         measurement.Measurement
	Tool           measurement.Measurement
	Ordinary       measurement.Measurement
	Derived        measurement.Measurement
	Containment    Containment
	Centroid       *CentroidLineage
	Moments        *ContainedCutMoments
}

// ContainedCutVolume subtracts the tool's source volume from the target's
// source volume and selects it only when its proven interval agrees with, and
// is strictly tighter than, the ordinary mesh result.
func ContainedCutVolume(targetProducer, toolProducer int, target, tool, ordinary measurement.Measurement,
	containment Containment) (*VolumeLineage, bool, error) {
	targetBound := proofbound.MeasuredScalar(target.Value.Base(), target.Bound.Base())
	toolBound := proofbound.MeasuredScalar(tool.Value.Base(), tool.Bound.Base())
	derivedBound := proofbound.BoundedSub(targetBound, toolBound)
	derived := measurement.Measurement{
		Value:     units.CubicMillimeters(derivedBound.Value),
		Exactness: measurement.ExactnessOf(derivedBound.Bound),
		Bound:     units.CubicMillimeters(derivedBound.Bound),
	}
	derivedLo, derivedHi := proofbound.BoundedEnds(derivedBound)
	ordinaryBound := proofbound.MeasuredScalar(ordinary.Value.Base(), ordinary.Bound.Base())
	ordinaryLo, ordinaryHi := proofbound.BoundedEnds(ordinaryBound)
	if derivedLo > ordinaryHi || ordinaryLo > derivedHi {
		return nil, false, ErrDisjointIntervals
	}
	lineage := &VolumeLineage{
		TargetProducer: targetProducer,
		ToolProducer:   toolProducer,
		Target:         target,
		Tool:           tool,
		Ordinary:       ordinary,
		Derived:        derived,
		Containment:    containment,
	}
	if derived.Bound.Base() >= ordinary.Bound.Base() {
		return lineage, false, nil
	}
	return lineage, true, nil
}
