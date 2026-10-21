package decad

import (
	"github.com/lestrrat-3d/decad/report"
)

// This file owns the public verification diagnostic and pair rows.
// The verdict rules are in agent-docs/verification-design.md §1-§3.

// Status is a verification verdict. See agent-docs/verification-design.md §6.
type Status = report.Status

// ReadingKind names the bounded quantity carried by a diagnostic.
type ReadingKind = report.ReadingKind

// SurveyKind names the optional body survey a diagnostic concerns.
type SurveyKind = report.SurveyKind

// DiagnosticCode is a diagnostic's stable reason code.
type DiagnosticCode = report.DiagnosticCode

const (
	Unverified  = report.Unverified
	Sound       = report.Sound
	Suspect     = report.Suspect
	Violating   = report.Violating
	Interfering = report.Interfering
	Unsound     = report.Unsound
)

const (
	ReadingNone          = report.ReadingNone
	ReadingArea          = report.ReadingArea
	ReadingBounds        = report.ReadingBounds
	ReadingVolume        = report.ReadingVolume
	ReadingCentroid      = report.ReadingCentroid
	ReadingWall          = report.ReadingWall
	ReadingMinRadius     = report.ReadingMinRadius
	ReadingOverlapVolume = report.ReadingOverlapVolume
	ReadingGap           = report.ReadingGap
)

const (
	SurveyNone          = report.SurveyNone
	SurveyWall          = report.SurveyWall
	SurveyUndercut      = report.SurveyUndercut
	SurveyConcaveRadius = report.SurveyConcaveRadius
)

const (
	DiagMeasurementBeyondTolerance    = report.DiagMeasurementBeyondTolerance
	DiagUndecidedValidity             = report.DiagUndecidedValidity
	DiagInvalidBody                   = report.DiagInvalidBody
	DiagWallTooThin                   = report.DiagWallTooThin
	DiagUndercut                      = report.DiagUndercut
	DiagUndecidedWall                 = report.DiagUndecidedWall
	DiagUndecidedUndercut             = report.DiagUndecidedUndercut
	DiagUndecidedMinRadius            = report.DiagUndecidedMinRadius
	DiagInterference                  = report.DiagInterference
	DiagUndecidedPair                 = report.DiagUndecidedPair
	DiagUndecidedClearance            = report.DiagUndecidedClearance
	DiagUndecidedInterference         = report.DiagUndecidedInterference
	DiagUnsupportedPairPayload        = report.DiagUnsupportedPairPayload
	DiagUnsupportedPairContact        = report.DiagUnsupportedPairContact
	DiagUnsupportedPairPipeline       = report.DiagUnsupportedPairPipeline
	DiagUnsupportedPairSheet          = report.DiagUnsupportedPairSheet
	DiagSheetSolidCrossing            = report.DiagSheetSolidCrossing
	DiagUnsupportedSurveyPayload      = report.DiagUnsupportedSurveyPayload
	DiagSurveyPrerequisite            = report.DiagSurveyPrerequisite
	DiagToleranceReferenceUnavailable = report.DiagToleranceReferenceUnavailable
	DiagMotionCollision               = report.DiagMotionCollision
	DiagMotionClearanceViolated       = report.DiagMotionClearanceViolated
	DiagMotionUndecidedInterval       = report.DiagMotionUndecidedInterval
	DiagMotionUndecidedClearance      = report.DiagMotionUndecidedClearance
	DiagJointBoxBudgetExhausted       = report.DiagJointBoxBudgetExhausted
)

// DiagnosticPair names two bodies in a pair finding.
type DiagnosticPair = report.DiagnosticPairOf[*Body]

// Diagnostic is a structured reason for a body or pair verdict.
type Diagnostic = report.DiagnosticOf[*Body, JointCell]

// Interference is a proven overlap between two bodies.
type Interference = report.InterferenceOf[*Body]

// Clearance is a measured gap between two bodies.
type Clearance = report.ClearanceOf[*Body]
