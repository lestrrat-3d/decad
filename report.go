package decad

import (
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

// This file owns the public verification diagnostic and pair rows.
// The verdict rules are in docs/verification-design.md §1-§3.

// Status is a verification verdict. See docs/verification-design.md §6.
type Status = reportvocab.Status

// ReadingKind names the bounded quantity carried by a diagnostic.
type ReadingKind = reportvocab.ReadingKind

// SurveyKind names the optional body survey a diagnostic concerns.
type SurveyKind = reportvocab.SurveyKind

// DiagnosticCode is a diagnostic's stable reason code.
type DiagnosticCode = reportvocab.DiagnosticCode

const (
	Unverified                        = reportvocab.Unverified
	Sound                             = reportvocab.Sound
	Suspect                           = reportvocab.Suspect
	Violating                         = reportvocab.Violating
	Interfering                       = reportvocab.Interfering
	Unsound                           = reportvocab.Unsound
	ReadingNone                       = reportvocab.ReadingNone
	ReadingArea                       = reportvocab.ReadingArea
	ReadingBounds                     = reportvocab.ReadingBounds
	ReadingVolume                     = reportvocab.ReadingVolume
	ReadingCentroid                   = reportvocab.ReadingCentroid
	ReadingWall                       = reportvocab.ReadingWall
	ReadingMinRadius                  = reportvocab.ReadingMinRadius
	ReadingOverlapVolume              = reportvocab.ReadingOverlapVolume
	ReadingGap                        = reportvocab.ReadingGap
	SurveyNone                        = reportvocab.SurveyNone
	SurveyWall                        = reportvocab.SurveyWall
	SurveyUndercut                    = reportvocab.SurveyUndercut
	SurveyConcaveRadius               = reportvocab.SurveyConcaveRadius
	DiagMeasurementBeyondTolerance    = reportvocab.DiagMeasurementBeyondTolerance
	DiagUndecidedValidity             = reportvocab.DiagUndecidedValidity
	DiagInvalidBody                   = reportvocab.DiagInvalidBody
	DiagWallTooThin                   = reportvocab.DiagWallTooThin
	DiagUndercut                      = reportvocab.DiagUndercut
	DiagUndecidedWall                 = reportvocab.DiagUndecidedWall
	DiagUndecidedUndercut             = reportvocab.DiagUndecidedUndercut
	DiagUndecidedMinRadius            = reportvocab.DiagUndecidedMinRadius
	DiagInterference                  = reportvocab.DiagInterference
	DiagUndecidedPair                 = reportvocab.DiagUndecidedPair
	DiagUndecidedClearance            = reportvocab.DiagUndecidedClearance
	DiagUndecidedInterference         = reportvocab.DiagUndecidedInterference
	DiagUnsupportedPairPayload        = reportvocab.DiagUnsupportedPairPayload
	DiagUnsupportedPairContact        = reportvocab.DiagUnsupportedPairContact
	DiagUnsupportedPairPipeline       = reportvocab.DiagUnsupportedPairPipeline
	DiagUnsupportedPairSheet          = reportvocab.DiagUnsupportedPairSheet
	DiagSheetSolidCrossing            = reportvocab.DiagSheetSolidCrossing
	DiagUnsupportedSurveyPayload      = reportvocab.DiagUnsupportedSurveyPayload
	DiagSurveyPrerequisite            = reportvocab.DiagSurveyPrerequisite
	DiagToleranceReferenceUnavailable = reportvocab.DiagToleranceReferenceUnavailable
	DiagMotionCollision               = reportvocab.DiagMotionCollision
	DiagMotionClearanceViolated       = reportvocab.DiagMotionClearanceViolated
	DiagMotionUndecidedInterval       = reportvocab.DiagMotionUndecidedInterval
	DiagMotionUndecidedClearance      = reportvocab.DiagMotionUndecidedClearance
	DiagJointBoxBudgetExhausted       = reportvocab.DiagJointBoxBudgetExhausted
)

// DiagnosticPair names two bodies in a pair finding.
type DiagnosticPair struct{ A, B *Body }

// Diagnostic is a structured reason for a body or pair verdict.
type Diagnostic struct {
	Code        DiagnosticCode
	Status      Status
	Body        *Body
	Pair        *DiagnosticPair
	Survey      SurveyKind
	Reading     ReadingKind
	Observed    *Measurement
	ObservedVec *VecMeasurement
	ObservedBox *Box
	Required    *units.Value
	At          *units.Value
	Cell        *JointCell
	Message     string
}

// Interference is a proven overlap between two bodies.
type Interference struct {
	A, B   *Body
	Volume Measurement
}

// Clearance is a measured gap between two bodies.
type Clearance struct {
	A, B *Body
	Gap  Measurement
}

func diagnosticFromInternal(in reportvocab.Diagnostic[*Body, JointCell]) Diagnostic {
	out := Diagnostic{
		Code: in.Code, Status: in.Status, Body: in.Body,
		Survey: in.Survey, Reading: in.Reading,
		Observed:    measurementPtrFromInternal(in.Observed),
		ObservedVec: vecMeasurementPtrFromInternal(in.ObservedVec),
		ObservedBox: boxPtrFromInternal(in.ObservedBox),
		Required:    in.Required, At: in.At, Cell: in.Cell, Message: in.Message,
	}
	if in.Pair != nil {
		out.Pair = &DiagnosticPair{A: in.Pair.A, B: in.Pair.B}
	}
	return out
}
func diagnosticToInternal(in Diagnostic) reportvocab.Diagnostic[*Body, JointCell] {
	out := reportvocab.Diagnostic[*Body, JointCell]{
		Code: in.Code, Status: in.Status, Body: in.Body,
		Survey: in.Survey, Reading: in.Reading,
		Observed:    measurementPtrToInternal(in.Observed),
		ObservedVec: vecMeasurementPtrToInternal(in.ObservedVec),
		ObservedBox: boxPtrToInternal(in.ObservedBox),
		Required:    in.Required, At: in.At, Cell: in.Cell, Message: in.Message,
	}
	if in.Pair != nil {
		out.Pair = &reportvocab.DiagnosticPair[*Body]{A: in.Pair.A, B: in.Pair.B}
	}
	return out
}

func diagnosticPtrToInternal(in *Diagnostic) *reportvocab.Diagnostic[*Body, JointCell] {
	if in == nil {
		return nil
	}
	out := diagnosticToInternal(*in)
	return &out
}

func diagnosticPtrFromInternal(in *reportvocab.Diagnostic[*Body, JointCell]) *Diagnostic {
	if in == nil {
		return nil
	}
	out := diagnosticFromInternal(*in)
	return &out
}

func diagnosticsFromInternal(in []reportvocab.Diagnostic[*Body, JointCell]) []Diagnostic {
	if in == nil {
		return nil
	}
	out := make([]Diagnostic, len(in))
	for i := range in {
		out[i] = diagnosticFromInternal(in[i])
	}
	return out
}

func diagnosticsToInternal(in []Diagnostic) []reportvocab.Diagnostic[*Body, JointCell] {
	if in == nil {
		return nil
	}
	out := make([]reportvocab.Diagnostic[*Body, JointCell], len(in))
	for i := range in {
		out[i] = diagnosticToInternal(in[i])
	}
	return out
}
