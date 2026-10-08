package tolerance

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

// RequiredThreshold reports rel*ref in the bounded reading's own unit.
func RequiredThreshold(relRef float64, sample units.Value) *units.Value {
	v := units.FromBase(relRef, sample.Unit())
	return &v
}

// Judge converts a gate comparison into the reading's tolerance verdict.
// A zero bound passes without loading a reference or publishing a limit.
func Judge(pass, haveRef bool, rel, ref float64, sample units.Value) reportvocab.ToleranceResult {
	if !haveRef {
		if pass {
			return reportvocab.ToleranceResult{State: reportvocab.ToleranceSatisfied}
		}
		return reportvocab.ToleranceResult{State: reportvocab.ToleranceUndecided}
	}
	limit := RequiredThreshold(rel*ref, sample)
	if pass {
		return reportvocab.ToleranceResult{State: reportvocab.ToleranceSatisfied, Limit: limit}
	}
	return reportvocab.ToleranceResult{State: reportvocab.ToleranceExceeded, Limit: limit}
}

// DiagnosticCode distinguishes a rejected bound from a missing reference.
func DiagnosticCode(tr reportvocab.ToleranceResult) reportvocab.DiagnosticCode {
	if tr.State == reportvocab.ToleranceUndecided {
		return reportvocab.DiagToleranceReferenceUnavailable
	}
	return reportvocab.DiagMeasurementBeyondTolerance
}

// ScalarVerdict judges one scalar reading and returns its diagnostic if it fails.
func ScalarVerdict[BodyT comparable, CellT any](reading reportvocab.ReadingKind,
	survey reportvocab.SurveyKind, body BodyT, m measurement.Measurement, rel float64,
	reference func(float64) (float64, bool),
) (reportvocab.ToleranceResult, *reportvocab.Diagnostic[BodyT, CellT]) {
	pass, ref, haveRef := Scalar(m.Value, m.Bound, rel, reference)
	tr := Judge(pass, haveRef, rel, ref, m.Value)
	if pass {
		return tr, nil
	}
	obs := m
	code := DiagnosticCode(tr)
	message := fmt.Sprintf("the %s reading's bound %s is beyond the relative tolerance", reading, m.Bound)
	if code == reportvocab.DiagToleranceReferenceUnavailable {
		message = fmt.Sprintf("the %s reading has no usable tolerance reference", reading)
	}
	return tr, &reportvocab.Diagnostic[BodyT, CellT]{
		Code: code, Status: reportvocab.Suspect, Body: body,
		Survey: survey, Reading: reading, Observed: &obs,
		Required: tr.Limit, Message: message,
	}
}

// BoundsVerdict judges a body's box against its diameter reference.
func BoundsVerdict[BodyT comparable, CellT any](body BodyT, box measurement.Box,
	rel float64, reference func() (float64, bool),
) (reportvocab.ToleranceResult, *reportvocab.Diagnostic[BodyT, CellT]) {
	pass, ref, haveRef := Bounded(box.Bound.Base(), rel, reference)
	tr := Judge(pass, haveRef, rel, ref, box.Bound)
	if pass {
		return tr, nil
	}
	observed := box
	code := DiagnosticCode(tr)
	message := fmt.Sprintf("the bounds reading's bound %s is beyond the relative tolerance", box.Bound)
	if code == reportvocab.DiagToleranceReferenceUnavailable {
		message = "the bounds reading has no usable tolerance reference"
	}
	return tr, &reportvocab.Diagnostic[BodyT, CellT]{
		Code: code, Status: reportvocab.Suspect, Body: body,
		Reading: reportvocab.ReadingBounds, ObservedBox: &observed,
		Required: tr.Limit, Message: message,
	}
}

// CentroidVerdict judges a body's centroid against its diameter reference.
func CentroidVerdict[BodyT comparable, CellT any](body BodyT, cen measurement.VecMeasurement,
	rel float64, reference func() (float64, bool),
) (reportvocab.ToleranceResult, *reportvocab.Diagnostic[BodyT, CellT]) {
	pass, ref, haveRef := Bounded(cen.Bound.Base(), rel, reference)
	tr := Judge(pass, haveRef, rel, ref, cen.Bound)
	if pass {
		return tr, nil
	}
	observed := cen
	code := DiagnosticCode(tr)
	message := fmt.Sprintf("the centroid reading's bound %s is beyond the relative tolerance", cen.Bound)
	if code == reportvocab.DiagToleranceReferenceUnavailable {
		message = "the centroid reading has no usable tolerance reference"
	}
	return tr, &reportvocab.Diagnostic[BodyT, CellT]{
		Code: code, Status: reportvocab.Suspect, Body: body,
		Reading: reportvocab.ReadingCentroid, ObservedVec: &observed,
		Required: tr.Limit, Message: message,
	}
}

// BodyReadings carries each present bounded body reading.
type BodyReadings struct {
	Area     measurement.Measurement
	Bounds   measurement.Box
	Volume   *measurement.Measurement
	Centroid *measurement.VecMeasurement
	Wall     *measurement.Measurement
	Radius   *measurement.Measurement
}

// BodyVerdicts holds the result for each present reading.
type BodyVerdicts struct {
	Area, Bounds, Volume, Centroid, Wall, Radius reportvocab.ToleranceResult
}

// BodyDiagSet keeps core diagnostics apart from optional survey findings.
type BodyDiagSet[BodyT comparable, CellT any] struct {
	Core   []reportvocab.Diagnostic[BodyT, CellT]
	Wall   *reportvocab.Diagnostic[BodyT, CellT]
	Radius *reportvocab.Diagnostic[BodyT, CellT]
}

// BodyReferences loads only the reference a nonzero bound needs.
type BodyReferences struct {
	Area, Volume, Length func(float64) (float64, bool)
	Diameter             func() (float64, bool)
}

// BodyDiagnostics judges every present reading and emits one finding for each
// failure, preserving core and survey order. Reference callbacks stay lazy.
func BodyDiagnostics[BodyT comparable, CellT any](body BodyT, r BodyReadings,
	rel float64, refs BodyReferences,
) (BodyVerdicts, BodyDiagSet[BodyT, CellT]) {
	var out BodyDiagSet[BodyT, CellT]
	var verdicts BodyVerdicts
	scalar := func(reading reportvocab.ReadingKind, survey reportvocab.SurveyKind,
		m measurement.Measurement, reference func(float64) (float64, bool),
	) (reportvocab.ToleranceResult, *reportvocab.Diagnostic[BodyT, CellT]) {
		return ScalarVerdict[BodyT, CellT](reading, survey, body, m, rel, reference)
	}

	var diag *reportvocab.Diagnostic[BodyT, CellT]
	verdicts.Area, diag = scalar(reportvocab.ReadingArea, reportvocab.SurveyNone, r.Area, refs.Area)
	if diag != nil {
		out.Core = append(out.Core, *diag)
	}
	verdicts.Bounds, diag = BoundsVerdict[BodyT, CellT](body, r.Bounds, rel, refs.Diameter)
	if diag != nil {
		out.Core = append(out.Core, *diag)
	}
	if r.Volume != nil {
		verdicts.Volume, diag = scalar(reportvocab.ReadingVolume, reportvocab.SurveyNone, *r.Volume, refs.Volume)
		if diag != nil {
			out.Core = append(out.Core, *diag)
		}
	}
	if r.Centroid != nil {
		verdicts.Centroid, diag = CentroidVerdict[BodyT, CellT](body, *r.Centroid, rel, refs.Diameter)
		if diag != nil {
			out.Core = append(out.Core, *diag)
		}
	}
	if r.Wall != nil {
		verdicts.Wall, out.Wall = scalar(reportvocab.ReadingWall, reportvocab.SurveyWall, *r.Wall, refs.Length)
	}
	if r.Radius != nil {
		verdicts.Radius, out.Radius = scalar(reportvocab.ReadingMinRadius,
			reportvocab.SurveyConcaveRadius, *r.Radius, refs.Length)
	}
	return verdicts, out
}
