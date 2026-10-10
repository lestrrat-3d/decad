package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

// This file owns the public verification diagnostic and pair rows.
// The verdict rules are in docs/verification-design.md §1-§3.

// Status is a verification verdict. See docs/verification-design.md §6.
type Status int

// String renders the verdict name.
func (s Status) String() string {
	switch s {
	case Unverified:
		return "Unverified"
	case Sound:
		return "Sound"
	case Suspect:
		return "Suspect"
	case Violating:
		return "Violating"
	case Interfering:
		return "Interfering"
	case Unsound:
		return "Unsound"
	default:
		return fmt.Sprintf("Status(%d)", int(s))
	}
}

// ReadingKind names the bounded quantity carried by a diagnostic.
type ReadingKind int

// String renders the stable reading token.
func (k ReadingKind) String() string { return reportvocab.ReadingKind(k).String() }

// SurveyKind names the optional body survey a diagnostic concerns.
type SurveyKind int

// String renders the stable survey token.
func (k SurveyKind) String() string { return reportvocab.SurveyKind(k).String() }

// DiagnosticCode is a diagnostic's stable reason code.
type DiagnosticCode int

// String renders the stable diagnostic token.
func (c DiagnosticCode) String() string { return reportvocab.DiagnosticCode(c).String() }

const (
	Unverified Status = iota
	Sound
	Suspect
	Violating
	Interfering
	Unsound
)

const (
	ReadingNone ReadingKind = iota
	ReadingArea
	ReadingBounds
	ReadingVolume
	ReadingCentroid
	ReadingWall
	ReadingMinRadius
	ReadingOverlapVolume
	ReadingGap
)

const (
	SurveyNone SurveyKind = iota
	SurveyWall
	SurveyUndercut
	SurveyConcaveRadius
)

const (
	DiagMeasurementBeyondTolerance DiagnosticCode = iota
	DiagUndecidedValidity
	DiagInvalidBody
	DiagWallTooThin
	DiagUndercut
	DiagUndecidedWall
	DiagUndecidedUndercut
	DiagUndecidedMinRadius
	DiagInterference
	DiagUndecidedPair
	DiagUndecidedClearance
	DiagUndecidedInterference
	DiagUnsupportedPairPayload
	DiagUnsupportedPairContact
	DiagUnsupportedPairPipeline
	DiagUnsupportedPairSheet
	DiagSheetSolidCrossing
	DiagUnsupportedSurveyPayload
	DiagSurveyPrerequisite
	DiagToleranceReferenceUnavailable
	DiagMotionCollision
	DiagMotionClearanceViolated
	DiagMotionUndecidedInterval
	DiagMotionUndecidedClearance
	DiagJointBoxBudgetExhausted
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
		Code: DiagnosticCode(in.Code), Status: Status(in.Status), Body: in.Body,
		Survey: SurveyKind(in.Survey), Reading: ReadingKind(in.Reading),
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
		Code: reportvocab.DiagnosticCode(in.Code), Status: reportvocab.Status(in.Status), Body: in.Body,
		Survey: reportvocab.SurveyKind(in.Survey), Reading: reportvocab.ReadingKind(in.Reading),
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
