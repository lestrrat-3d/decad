package decad

import (
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

// This file exposes the verification vocabulary from internal/reportvocab
// and defines report rows that refer to Body, JointCell, and Measurement.
// Report and BodyReport live in verify_result.go. The verdict rules are in
// docs/verification-design.md §1-§3.

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

// DiagnosticPair names the two bodies of a pair diagnostic, in the report's own
// stable pair order (interference design §2).
type DiagnosticPair struct{ A, B *Body }

// Diagnostic is one structured, branchable reason a body or a pair is not Sound
// (verification §1.1). It never decides the verdict — Status is still §6's
// worst-wins aggregate — it explains it. Exactly one of Observed / ObservedVec
// / ObservedBox is non-nil, keyed by Reading (all three nil when
// Reading == ReadingNone). Survey identifies which optional body survey the
// reason concerns — SurveyNone for every core reading and every pair
// diagnostic — set even when Reading is ReadingNone, so an unsupported wall,
// undercut, or concave-radius refusal is distinguished without inspecting
// Message text. At is the motion parameter a VerifyMotion finding concerns
// (docs/motion-check-design.md §4.1); it is nil on every diagnostic Verify
// emits and on every motion finding about the whole path. Cell is the joint
// cell a VerifyJointBox finding concerns (docs/linkage-check-design.md §14.2);
// it is nil on every diagnostic Verify, VerifyMotion and VerifyLinkage emit.
type Diagnostic struct {
	Code        DiagnosticCode  // the stable branch key
	Status      Status          // the rung this reason contributes
	Body        *Body           // the body it concerns; nil for a pair diagnostic
	Pair        *DiagnosticPair // the pair it concerns; nil for a body diagnostic
	Survey      SurveyKind      // which optional body survey this concerns; SurveyNone for core/pair reasons
	Reading     ReadingKind     // which quantity the Observed* form carries; ReadingNone names none
	Observed    *Measurement    // a scalar reading; nil unless Reading names a scalar quantity
	ObservedVec *VecMeasurement // a vector reading (a Centroid); nil unless Reading == ReadingCentroid
	ObservedBox *Box            // a box reading (a Bounds); nil unless Reading == ReadingBounds
	Required    *units.Value    // the threshold the reading was judged against; nil when the reason states none
	At          *units.Value    // the motion parameter a VerifyMotion finding concerns; nil outside motion reports
	Cell        *JointCell      // the joint cell a VerifyJointBox finding concerns; nil outside joint-box reports
	Message     string          // human-readable; NEVER the branch key
}

// Interference is a proven pairwise overlap, carrying its bounded overlap
// volume (verification §1, interference design §6). Verification computes it
// without consuming either body or changing the document.
type Interference struct {
	A, B   *Body
	Volume Measurement
}

// Clearance is the minimum gap between a pair of proven-disjoint bodies
// (verification §1). A row exists only for a pair proven disjoint with a
// measured gap: the clearance kernel (docs/clearance-design.md) proves the
// gap as an interval, and Gap reports its midpoint with the interval's half
// width as the proven Bound — Exact exactly when the interval is a point. A
// touching pair's zero is a measured Exact zero carried by a certified
// contact; a pair whose gap the kernel cannot prove yields no row and reads
// Suspect under WithClearances.
//
// For a pair holding a sheet operand, the row means something narrower
// (docs/surface-design.md §9.3): a sheet has no volume to be disjoint FROM,
// so Gap states the proven distance between the sheet and the solid's
// boundary, whether the sheet lies wholly inside or wholly outside it, WITHOUT
// asserting which side. A solid-solid row's meaning is unchanged.
type Clearance struct {
	A, B *Body
	Gap  Measurement
}
