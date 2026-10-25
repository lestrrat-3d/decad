package prooflineage

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	ErrInvalidCentroidEvidence = errors.New("contained-cut centroid evidence is invalid")
	ErrDisjointCentroidBounds  = errors.New("contained-cut centroid bounds do not overlap")
)

// SourceCentroid identifies one source body's bounded centroid reading.
type SourceCentroid struct {
	Producer int
	Reading  measurement.VecMeasurement
}

// CentroidLineage retains the source readings and both candidate result
// centroids without retaining body or topology nodes.
type CentroidLineage struct {
	Target   SourceCentroid
	Tool     SourceCentroid
	Ordinary measurement.VecMeasurement
	Derived  measurement.VecMeasurement
	Selected bool
}

// ContainedCutCentroid derives a bounded centroid from a contained Cut's
// source volumes and centroids. A valid but unhelpful derivation returns a nil
// lineage, false, nil when its volume interval reaches zero or its arithmetic
// exceeds the finite range. Invalid evidence and contradictory certified
// readings return an error.
func ContainedCutCentroid(lineage *VolumeLineage, target, tool SourceCentroid,
	ordinary measurement.VecMeasurement) (*CentroidLineage, bool, error) {
	if lineage == nil {
		return nil, false, invalidCentroidEvidence("missing volume lineage")
	}
	if err := validateContainment(lineage.Containment); err != nil {
		return nil, false, err
	}
	if target.Producer != lineage.TargetProducer || tool.Producer != lineage.ToolProducer {
		return nil, false, invalidCentroidEvidence("source producer does not match volume lineage")
	}
	if err := validateVolumeReading("target", lineage.Target); err != nil {
		return nil, false, err
	}
	if err := validateVolumeReading("tool", lineage.Tool); err != nil {
		return nil, false, err
	}
	if err := validateVolumeReading("ordinary result", lineage.Ordinary); err != nil {
		return nil, false, err
	}
	if err := validateVolumeReading("derived result", lineage.Derived); err != nil {
		return nil, false, err
	}
	if err := validateCentroidReading("target", target.Reading); err != nil {
		return nil, false, err
	}
	if err := validateCentroidReading("tool", tool.Reading); err != nil {
		return nil, false, err
	}
	if err := validateCentroidReading("ordinary result", ordinary); err != nil {
		return nil, false, err
	}

	targetVolume := proofbound.MeasuredScalar(lineage.Target.Value.Base(), lineage.Target.Bound.Base())
	toolVolume := proofbound.MeasuredScalar(lineage.Tool.Value.Base(), lineage.Tool.Bound.Base())
	denominator := proofbound.BoundedSub(targetVolume, toolVolume)
	if !boundedScalarFinite(denominator) ||
		lineage.Derived.Value.Base() != denominator.Value ||
		lineage.Derived.Bound.Base() != denominator.Bound ||
		lineage.Derived.Exactness != measurement.ExactnessOf(denominator.Bound) {
		return nil, false, invalidCentroidEvidence("derived volume does not match source subtraction")
	}
	derivedLo, derivedHi := proofbound.BoundedEnds(denominator)
	ordinaryVolume := proofbound.MeasuredScalar(lineage.Ordinary.Value.Base(), lineage.Ordinary.Bound.Base())
	ordinaryLo, ordinaryHi := proofbound.BoundedEnds(ordinaryVolume)
	if derivedLo > ordinaryHi || ordinaryLo > derivedHi {
		return nil, false, invalidCentroidEvidence("derived and ordinary volume intervals are disjoint")
	}

	switch proofbound.AdmitAbove(denominator, 0) {
	case proofbound.SurvAdmit:
	case proofbound.SurvStraddle:
		return nil, false, nil
	case proofbound.SurvReject:
		return nil, false, invalidCentroidEvidence("positive containment has a nonpositive volume difference")
	default:
		return nil, false, invalidCentroidEvidence("unknown volume admission state")
	}

	centroid, finite := deriveCentroid(targetVolume, toolVolume, denominator,
		target.Reading, tool.Reading)
	if !finite {
		return nil, false, nil
	}

	radiusSum := proofbound.AbsSumUpper(ordinary.Bound.Base(), centroid.Bound.Base())
	if math.IsNaN(radiusSum) {
		return nil, false, invalidCentroidEvidence("centroid radius sum is not a number")
	}
	if !math.IsInf(radiusSum, 1) &&
		!proofbound.DvLenAtLeast(radiusSum, proofbound.HeldDelta(ordinary.Value, centroid.Value)) {
		return nil, false, ErrDisjointCentroidBounds
	}

	result := &CentroidLineage{
		Target: target, Tool: tool, Ordinary: ordinary, Derived: centroid,
	}
	selected := centroid.Bound.Base() < ordinary.Bound.Base()
	result.Selected = selected
	return result, selected, nil
}

func validateContainment(containment Containment) error {
	gap := containment.BoundaryGapLower
	if gap.Unit() != units.Millimeter || !measurement.FiniteValues(gap.Base()) || gap.Base() <= 0 ||
		containment.InnerShellWitnesses <= 0 || containment.OuterShellWitnesses <= 0 {
		return invalidCentroidEvidence("containment record is incomplete")
	}
	return nil
}

func validateVolumeReading(name string, reading measurement.Measurement) error {
	if reading.Value.Unit() != units.CubicMillimeter || reading.Bound.Unit() != units.CubicMillimeter ||
		!measurement.FiniteValues(reading.Value.Base(), reading.Bound.Base()) || reading.Bound.Base() < 0 ||
		reading.Exactness != measurement.ExactnessOf(reading.Bound.Base()) {
		return invalidCentroidEvidence(name + " volume reading is malformed")
	}
	return nil
}

func validateCentroidReading(name string, reading measurement.VecMeasurement) error {
	if reading.Bound.Unit() != units.Millimeter || !proofbound.FiniteVec(reading.Value) ||
		!measurement.FiniteValues(reading.Bound.Base()) || reading.Bound.Base() < 0 ||
		reading.Exactness != measurement.ExactnessOf(reading.Bound.Base()) {
		return invalidCentroidEvidence(name + " centroid reading is malformed")
	}
	return nil
}

func deriveCentroid(targetVolume, toolVolume, denominator proofbound.BoundedScalar,
	target, tool measurement.VecMeasurement) (measurement.VecMeasurement, bool) {
	targetCoordinates := [3]float64{target.Value.X, target.Value.Y, target.Value.Z}
	toolCoordinates := [3]float64{tool.Value.X, tool.Value.Y, tool.Value.Z}
	var value [3]float64
	var coordinateBound [3]float64
	for i, anchor := range targetCoordinates {
		targetCoordinate := proofbound.MeasuredScalar(anchor, target.Bound.Base())
		toolCoordinate := proofbound.MeasuredScalar(toolCoordinates[i], tool.Bound.Base())
		exactAnchor := proofbound.ExactScalar(anchor)
		targetOffset := proofbound.BoundedSub(targetCoordinate, exactAnchor)
		toolOffset := proofbound.BoundedSub(toolCoordinate, exactAnchor)
		targetMoment := proofbound.BoundedMul(targetVolume, targetOffset)
		toolMoment := proofbound.BoundedMul(toolVolume, toolOffset)
		momentDifference := proofbound.BoundedSub(targetMoment, toolMoment)
		quotient := proofbound.BoundedDiv(momentDifference, denominator)
		coordinate := proofbound.BoundedAdd(exactAnchor, quotient)
		if !boundedScalarFinite(coordinate) || coordinate.Bound < 0 {
			return measurement.VecMeasurement{}, false
		}
		value[i], coordinateBound[i] = coordinate.Value, coordinate.Bound
	}
	radius := proofbound.Radius3D(max(coordinateBound[0], coordinateBound[1], coordinateBound[2]))
	if !measurement.FiniteValues(radius) || radius < 0 {
		return measurement.VecMeasurement{}, false
	}
	reading := measurement.VecMeasurement{
		Value:     r3.Vec{X: value[0], Y: value[1], Z: value[2]},
		Exactness: measurement.ExactnessOf(radius),
		Bound:     units.Millimeters(radius),
	}
	return reading, true
}

func boundedScalarFinite(value proofbound.BoundedScalar) bool {
	return measurement.FiniteValues(value.Value, value.Bound) && value.Bound >= 0
}

func invalidCentroidEvidence(reason string) error {
	return fmt.Errorf("%w: %s", ErrInvalidCentroidEvidence, reason)
}
