package decad

import (
	"github.com/lestrrat-3d/decad/internal/reportvocab"
)

// This file exposes the verification vocabulary and report rows from
// internal/reportvocab. The verdict rules are in docs/verification-design.md
// §1-§3.

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
	DiagUnsupportedPair               = reportvocab.DiagUnsupportedPair
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
type DiagnosticPair = reportvocab.DiagnosticPair[*Body]

// Diagnostic is a structured reason for a body or pair verdict.
type Diagnostic = reportvocab.Diagnostic[*Body, JointCell]

// Interference is a proven overlap between two bodies.
type Interference = reportvocab.Interference[*Body]

// Clearance is a measured gap between two bodies.
type Clearance = reportvocab.Clearance[*Body]
