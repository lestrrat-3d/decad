package decad

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tokenNotEvaluated and tokenUndecided are the two stable lower-snake tokens
// every outcome enum below shares at its own zero-value or undecided branch;
// naming each once keeps every enum's String() spelling it identically.
const (
	tokenNotEvaluated = "not_evaluated"
	tokenUndecided    = "undecided"
)

// This file holds the result vocabulary Verify's report is written in: the
// outcome enums, the effective-request records, the bounded reading
// wrappers, the per-survey result records, and Report/BodyReport themselves
// (docs/verification-design.md §1-§9). It holds TYPES and the two pure
// predicates over them (Passed, ForBody) alone; verify_publish.go builds a
// Report and its BodyReport entries from real survey outcomes and certified
// readings, and verify.go and survey.go feed it.

// ScalarOutcome is a whole-body scalar survey's primary outcome (Wall or
// ConcaveRadius): whether the question was asked, and if so, whether the
// solid prerequisite or payload allows it, the survey could decide it, no
// feature exists, or a minimum was proven. Every successful Verify call
// fills this nonzero on every returned BodyReport's Wall and ConcaveRadius
// results — ScalarNotRequested for a survey the call did not ask.
type ScalarOutcome int

const (
	// ScalarNotEvaluated is the reserved zero value: only a zero or
	// caller-created record carries it, never a value Verify returns.
	ScalarNotEvaluated ScalarOutcome = iota
	// ScalarNotRequested — this call did not ask this question.
	ScalarNotRequested
	// ScalarUnavailable — the solid prerequisite failed, or this payload has
	// no implemented survey.
	ScalarUnavailable
	// ScalarUndecided — the survey could not certify the whole minimum or
	// its absence.
	ScalarUndecided
	// ScalarAbsent — no feature in the survey's defined class exists.
	ScalarAbsent
	// ScalarMeasured — the reading encloses the actual whole-body minimum.
	ScalarMeasured
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "scalar_outcome(<n>)", never a panic.
func (o ScalarOutcome) String() string {
	switch o {
	case ScalarNotEvaluated:
		return tokenNotEvaluated
	case ScalarNotRequested:
		return "not_requested"
	case ScalarUnavailable:
		return "unavailable"
	case ScalarUndecided:
		return tokenUndecided
	case ScalarAbsent:
		return "absent"
	case ScalarMeasured:
		return "measured"
	default:
		return fmt.Sprintf("scalar_outcome(%d)", int(o))
	}
}

// Coverage is the undercut survey's primary outcome: how completely the
// producer decided face membership against the requested pull direction.
// Every successful Verify call fills this nonzero on every returned
// BodyReport's Undercut result — CoverageNotRequested for a call that did
// not ask.
type Coverage int

const (
	// CoverageNotEvaluated is the reserved zero value: only a zero or
	// caller-created record carries it, never a value Verify returns.
	CoverageNotEvaluated Coverage = iota
	// CoverageNotRequested — the pull direction was not requested.
	CoverageNotRequested
	// CoverageUnavailable — a prerequisite or payload capability prevents
	// the survey.
	CoverageUnavailable
	// CoverageUndecided — the producer certifies neither a complete list
	// nor any opposing face.
	CoverageUndecided
	// CoveragePartial — confirmed opposing faces exist and other
	// membership remains undecided.
	CoveragePartial
	// CoverageComplete — every face was decided; Faces lists every
	// opposing face, empty when none opposes.
	CoverageComplete
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "coverage(<n>)", never a panic.
func (c Coverage) String() string {
	switch c {
	case CoverageNotEvaluated:
		return tokenNotEvaluated
	case CoverageNotRequested:
		return "not_requested"
	case CoverageUnavailable:
		return "unavailable"
	case CoverageUndecided:
		return tokenUndecided
	case CoveragePartial:
		return "partial"
	case CoverageComplete:
		return "complete"
	default:
		return fmt.Sprintf("coverage(%d)", int(c))
	}
}

// Assessment is a stated spec's verdict against a ScalarOutcome or Coverage
// result: met, violated, undecided, or not evaluated because nothing was
// asked.
type Assessment int

const (
	// AssessmentNotEvaluated — no requirement was assessed: the survey was
	// not requested, or it carries no assessment (ConcaveRadius).
	AssessmentNotEvaluated Assessment = iota
	// AssessmentMet — the requirement holds.
	AssessmentMet
	// AssessmentViolated — the requirement is proven to fail.
	AssessmentViolated
	// AssessmentUndecided — no comparison can be proved.
	AssessmentUndecided
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "assessment(<n>)", never a panic.
func (a Assessment) String() string {
	switch a {
	case AssessmentNotEvaluated:
		return tokenNotEvaluated
	case AssessmentMet:
		return "met"
	case AssessmentViolated:
		return "violated"
	case AssessmentUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("assessment(%d)", int(a))
	}
}

// ToleranceState is one reading's verdict against the caller's relative
// tolerance gate (verification §2).
type ToleranceState int

const (
	// ToleranceNotEvaluated — no precision decision was made: the reading
	// does not exist, as for an invalid body's boundary data.
	ToleranceNotEvaluated ToleranceState = iota
	// ToleranceSatisfied — the gate accepted the bound.
	ToleranceSatisfied
	// ToleranceExceeded — the gate compared against a usable reference and
	// rejected the bound.
	ToleranceExceeded
	// ToleranceUndecided — a nonzero bound had no usable reference.
	ToleranceUndecided
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "tolerance_state(<n>)", never a panic.
func (t ToleranceState) String() string {
	switch t {
	case ToleranceNotEvaluated:
		return tokenNotEvaluated
	case ToleranceSatisfied:
		return "satisfied"
	case ToleranceExceeded:
		return "exceeded"
	case ToleranceUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("tolerance_state(%d)", int(t))
	}
}

// ValidityOutcome is one body's held-boundary validity verdict (proposal §9).
// ValidityValid entails the watertightness, manifoldness, and lack of
// self-intersection the proof gives for that solid; ValidityInvalid and
// ValidityUndecided establish no independent per-property verdict.
type ValidityOutcome int

const (
	// ValidityNotEvaluated is the reserved zero value: only a zero or
	// caller-created record carries it, never a value Verify returns.
	ValidityNotEvaluated ValidityOutcome = iota
	// ValidityValid — the evaluator's construction and the boundary audit
	// prove the body a valid solid.
	ValidityValid
	// ValidityInvalid — a concrete invalid-solid proof exists.
	ValidityInvalid
	// ValidityUndecided — the current evidence cannot decide validity.
	ValidityUndecided
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "validity_outcome(<n>)", never a panic.
func (v ValidityOutcome) String() string {
	switch v {
	case ValidityNotEvaluated:
		return tokenNotEvaluated
	case ValidityValid:
		return "valid"
	case ValidityInvalid:
		return "invalid"
	case ValidityUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("validity_outcome(%d)", int(v))
	}
}

// WallRequest is the effective WithMinWallThickness spec (proposal §5):
// canonicalized to millimetres and radians.
type WallRequest struct {
	Minimum        units.Value
	DraftAllowance units.Value
}

// UndercutRequest is the effective WithPullDirection spec (proposal §5): the
// exact accepted pull vector, unnormalized.
type UndercutRequest struct {
	PullDirection r3.Vec
}

// VerifyRequest is one Verify call's effective settings (proposal §5),
// recorded on Report.Request even for an empty document. Wall is non-nil
// exactly when WithMinWallThickness was requested, Undercut exactly when
// WithPullDirection was; WallResult.Request and UndercutResult.Request point
// at these same records.
type VerifyRequest struct {
	RelativeTolerance units.Value
	Wall              *WallRequest
	Undercut          *UndercutRequest
	ConcaveRadius     bool
	Clearances        bool
}

// ToleranceResult is one reading's tolerance verdict (proposal §8): Limit is
// nil for the zero-bound short circuit and for an undecided reference,
// otherwise the computed relative-tolerance-times-reference threshold, in
// the reading's own unit.
type ToleranceResult struct {
	State ToleranceState
	Limit *units.Value
}

// ScalarReading is a scalar measurement plus its tolerance verdict
// (proposal §8). Embedding Measurement gives callers typed value access
// (.Value, .Exactness, .Bound) with Tolerance as a separate decision.
type ScalarReading struct {
	Measurement
	Tolerance ToleranceResult
}

// VectorReading is a vector measurement (a centroid) plus its tolerance
// verdict.
type VectorReading struct {
	VecMeasurement
	Tolerance ToleranceResult
}

// BoundsReading is a bounding box plus its tolerance verdict.
type BoundsReading struct {
	Box
	Tolerance ToleranceResult
}

// WallResult is one body's wall survey result (proposal §6). Request is
// non-nil exactly for a requested survey and shares the pointer
// VerifyRequest.Wall carries. Diagnostics is this result's own local
// findings: the survey's own explanation (or the DiagSurveyPrerequisite a
// non-valid body's validity substitutes for it), plus the reading's own
// precision finding when it fired.
type WallResult struct {
	Request     *WallRequest
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Assessment  Assessment
	Diagnostics []Diagnostic
}

// UndercutResult is one body's undercut survey result (proposal §7): every
// face in Faces is a CONFIRMED opposing face against the requested pull; no
// uncertain face appears.
type UndercutResult struct {
	Request     *UndercutRequest
	Coverage    Coverage
	Faces       []*Face
	Assessment  Assessment
	Diagnostics []Diagnostic
}

// ConcaveRadiusResult is one body's concave-radius survey result (proposal
// §6). It has no Assessment field: Verify accepts no radius requirement.
type ConcaveRadiusResult struct {
	Outcome     ScalarOutcome
	Minimum     *ScalarReading
	Diagnostics []Diagnostic
}

// ValidityResult is one body's validity verdict and the diagnostic that
// explains it (proposal §9): at most one entry, since a body carries exactly
// one underlying validity finding.
type ValidityResult struct {
	Outcome     ValidityOutcome
	Diagnostics []Diagnostic
}

// HeldTopology is one body's lump and void counts (proposal §9), using the
// current count definitions; more than one lump is descriptive, not a
// failed requirement.
type HeldTopology struct {
	Lumps int
	Voids int
}

// RegionReadings is one proven-solid body's region quantities (proposal §9):
// present on BodyReport.Region exactly when Validity.Outcome is
// ValidityValid.
type RegionReadings struct {
	Volume   ScalarReading
	Centroid VectorReading
}

// BodyReport is one live body's verdict and readings (verification §1,
// proposal §3). Area and Bounds are unconditional boundary readings on
// every returned body; Region carries the two region quantities and is
// non-nil exactly when Validity.Outcome is ValidityValid (proposal §9).
// Every successful Verify call fills Validity.Outcome, Wall.Outcome,
// Undercut.Coverage, and ConcaveRadius.Outcome nonzero — NotRequested for an
// omitted survey — though this guarantee does not extend to Assessment or
// Tolerance states (proposal §4). Diagnostics is the deterministic
// flattening of Validity's, the core readings', Wall's, Undercut's, and
// ConcaveRadius's own local diagnostics, in that order, each finding
// present exactly once even though it is also reachable through its own
// result's local Diagnostics slice (proposal §10); it is empty exactly when
// Status is Sound.
type BodyReport struct {
	Body          *Body
	Status        Status
	Validity      ValidityResult
	Topology      HeldTopology
	Area          ScalarReading
	Bounds        BoundsReading
	Region        *RegionReadings
	Wall          WallResult
	Undercut      UndercutResult
	ConcaveRadius ConcaveRadiusResult
	Diagnostics   []Diagnostic
}

// Report is what Verify returns: the 3D counterpart of
// sketch.VerificationReport (core §10, verification §1). Request records the
// validated effective settings this call used, including defaults, even for
// an empty document (proposal §5). Bodies follows Document.Bodies() order at
// the call. Diagnostics is the canonical inventory for counting and logging
// the whole call: body diagnostics in body order, then pair diagnostics in
// pair order, each finding present exactly once (proposal §10, §11); it is
// empty EXACTLY when Status == Sound, and Status is the worst
// Diagnostic.Status in it. The zero Report is Unverified and carries no
// verdict.
type Report struct {
	Request       VerifyRequest
	Bodies        []*BodyReport
	Interferences []Interference
	Clearances    []Clearance
	Diagnostics   []Diagnostic
	Status        Status
}

// Passed reports whether the whole report is Sound for the effective request,
// including the default interference check (proposal §1, §4). It does not
// imply that every optional survey ran — Wall, Undercut, and ConcaveRadius
// can each read NotRequested on a Sound report. It returns false for a nil
// Report and for any Status other than Sound, including an unrecognized one:
// it is a convenience predicate over Status, not a validator of a
// caller-built or decoded Report, so an unrecognized nested result enum
// never overrides a caller-assigned Sound Status.
func (r *Report) Passed() bool {
	return r != nil && r.Status == Sound
}

// ForBody looks up body's BodyReport by exact pointer identity (proposal
// §11). A nil receiver or a nil body returns ErrDegenerate. Any absent
// non-nil identity, including a foreign body, returns
// ErrBodyReportNotFound. Successful lookup returns the report's own
// existing pointer and performs no verification: it consults only this
// Report's recorded bodies, never current document membership or liveness,
// so it still resolves a body later retired from its document.
func (r *Report) ForBody(body *Body) (*BodyReport, error) {
	if r == nil || body == nil {
		return nil, fmt.Errorf(`%w: a nil report or body names no lookup`, ErrDegenerate)
	}
	for _, br := range r.Bodies {
		if br.Body == body {
			return br, nil
		}
	}
	return nil, ErrBodyReportNotFound
}
