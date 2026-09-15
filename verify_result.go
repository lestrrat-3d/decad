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
// wrappers, and the per-survey / per-body / per-report result records
// (proposal §3 in .tmp/document-verify-api-proposal.md, under the private
// names .tmp/document-verify-api-tasks.md §2 assigns them). It holds TYPES
// alone; verify_publish.go builds them from real survey outcomes and
// certified readings, and verify.go and survey.go feed it. PR 5 renames
// every declaration here to its public counterpart with no shape change.

// scalarOutcome is a whole-body scalar survey's primary outcome (wall or
// concave radius): whether the question was asked, and if so, whether the
// prerequisite or payload allows it, the survey could decide it, no feature
// exists, or a minimum was proven.
type scalarOutcome int

const (
	scalarNotEvaluated scalarOutcome = iota
	scalarNotRequested
	scalarUnavailable
	scalarUndecided
	scalarAbsent
	scalarMeasured
)

func (o scalarOutcome) String() string {
	switch o {
	case scalarNotEvaluated:
		return tokenNotEvaluated
	case scalarNotRequested:
		return "not_requested"
	case scalarUnavailable:
		return "unavailable"
	case scalarUndecided:
		return tokenUndecided
	case scalarAbsent:
		return "absent"
	case scalarMeasured:
		return "measured"
	default:
		return fmt.Sprintf("scalar_outcome(%d)", int(o))
	}
}

// coverageState is the undercut survey's primary outcome: how completely the
// producer decided face membership against the pull.
type coverageState int

const (
	coverageNotEvaluated coverageState = iota
	coverageNotRequested
	coverageUnavailable
	coverageUndecided
	coveragePartial
	coverageComplete
)

func (c coverageState) String() string {
	switch c {
	case coverageNotEvaluated:
		return tokenNotEvaluated
	case coverageNotRequested:
		return "not_requested"
	case coverageUnavailable:
		return "unavailable"
	case coverageUndecided:
		return tokenUndecided
	case coveragePartial:
		return "partial"
	case coverageComplete:
		return "complete"
	default:
		return fmt.Sprintf("coverage(%d)", int(c))
	}
}

// assessmentState is a stated spec's verdict against a scalar or coverage
// outcome: met, violated, undecided, or not evaluated because nothing was
// asked.
type assessmentState int

const (
	assessmentNotEvaluated assessmentState = iota
	assessmentMet
	assessmentViolated
	assessmentUndecided
)

func (a assessmentState) String() string {
	switch a {
	case assessmentNotEvaluated:
		return tokenNotEvaluated
	case assessmentMet:
		return "met"
	case assessmentViolated:
		return "violated"
	case assessmentUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("assessment(%d)", int(a))
	}
}

// toleranceState is one reading's verdict against the caller's relative
// tolerance gate.
type toleranceState int

const (
	toleranceNotEvaluated toleranceState = iota
	toleranceSatisfied
	toleranceExceeded
	toleranceUndecided
)

func (t toleranceState) String() string {
	switch t {
	case toleranceNotEvaluated:
		return tokenNotEvaluated
	case toleranceSatisfied:
		return "satisfied"
	case toleranceExceeded:
		return "exceeded"
	case toleranceUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("tolerance_state(%d)", int(t))
	}
}

// validityOutcome is one body's held-boundary validity verdict.
type validityOutcome int

const (
	validityNotEvaluated validityOutcome = iota
	validityValid
	validityInvalid
	validityUndecided
)

func (v validityOutcome) String() string {
	switch v {
	case validityNotEvaluated:
		return tokenNotEvaluated
	case validityValid:
		return "valid"
	case validityInvalid:
		return "invalid"
	case validityUndecided:
		return tokenUndecided
	default:
		return fmt.Sprintf("validity_outcome(%d)", int(v))
	}
}

// wallRequest is the effective WithMinWallThickness spec (proposal §5):
// canonicalized to millimetres and radians.
type wallRequest struct {
	Minimum        units.Value
	DraftAllowance units.Value
}

// undercutRequest is the effective WithPullDirection spec (proposal §5): the
// exact accepted pull vector, unnormalized.
type undercutRequest struct {
	PullDirection r3.Vec
}

// verifyRequest is one Verify call's effective settings (proposal §5),
// recorded even for an empty document.
type verifyRequest struct {
	RelativeTolerance units.Value
	Wall              *wallRequest
	Undercut          *undercutRequest
	ConcaveRadius     bool
	Clearances        bool
}

// toleranceResult is one reading's tolerance verdict (proposal §8): Limit is
// nil for the zero-bound short circuit and for an undecided reference,
// otherwise the computed rel*ref threshold.
type toleranceResult struct {
	State toleranceState
	Limit *units.Value
}

// scalarReading is a scalar measurement plus its tolerance verdict
// (proposal §8). Embedding Measurement gives callers typed value access
// (.Value, .Exactness, .Bound) with Tolerance as a separate decision.
type scalarReading struct {
	Measurement
	Tolerance toleranceResult
}

// vectorReading is a vector measurement (a centroid) plus its tolerance
// verdict.
type vectorReading struct {
	VecMeasurement
	Tolerance toleranceResult
}

// boundsReading is a bounding box plus its tolerance verdict.
type boundsReading struct {
	Box
	Tolerance toleranceResult
}

// wallResult is one body's wall survey result (proposal §6). Request is
// non-nil exactly for a requested survey and shares the pointer
// verifyRequest.Wall carries.
type wallResult struct {
	Request     *wallRequest
	Outcome     scalarOutcome
	Minimum     *scalarReading
	Assessment  assessmentState
	Diagnostics []Diagnostic
}

// undercutResult is one body's undercut survey result (proposal §7): every
// face in Faces is a CONFIRMED opposing face; no uncertain face appears.
type undercutResult struct {
	Request     *undercutRequest
	Coverage    coverageState
	Faces       []*Face
	Assessment  assessmentState
	Diagnostics []Diagnostic
}

// concaveRadiusResult is one body's concave-radius survey result (proposal
// §6). It has no assessment field: Verify accepts no radius requirement.
type concaveRadiusResult struct {
	Outcome     scalarOutcome
	Minimum     *scalarReading
	Diagnostics []Diagnostic
}

// validityResult is one body's validity verdict and the diagnostics that
// explain it (proposal §9).
type validityResult struct {
	Outcome     validityOutcome
	Diagnostics []Diagnostic
}

// heldTopology is one body's lump and void counts.
type heldTopology struct {
	Lumps int
	Voids int
}

// regionReadings is one proven-solid body's region quantities (proposal §9):
// present only when Validity.Outcome is validityValid.
type regionReadings struct {
	Volume   scalarReading
	Centroid vectorReading
}

// bodyResult is one live body's verdict and readings — the private shape
// verify_publish.go assembles and projectLegacyBodyReport bridges onto the
// still-exported BodyReport. Undercut, Validity, Topology and Region stay
// their zero value until PRs 2 and 4 extend the assembler that fills them.
type bodyResult struct {
	Body          *Body
	Status        Status
	Validity      validityResult
	Topology      heldTopology
	Area          scalarReading
	Bounds        boundsReading
	Region        *regionReadings
	Wall          wallResult
	Undercut      undercutResult
	ConcaveRadius concaveRadiusResult
	Diagnostics   []Diagnostic
}

// verifyResult is one Verify call's complete result — the private shape
// PR 5 renames to the exported Report with no change of layout.
type verifyResult struct {
	Request       verifyRequest
	Bodies        []*bodyResult
	Interferences []Interference
	Clearances    []Clearance
	Diagnostics   []Diagnostic
	Status        Status
}
