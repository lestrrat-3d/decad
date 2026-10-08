package reportvocab

import "fmt"

// tokenNotEvaluated and tokenUndecided are the two stable lower-snake tokens
// every outcome enum below shares at its own zero-value or undecided branch;
// naming each once keeps every enum's String() spelling it identically.
const (
	tokenNotEvaluated = "not_evaluated"
	tokenUndecided    = "undecided"
)

// This package holds ContactRelation and SweepOutcome. It also holds Verify's
// report vocabulary: outcome enums, effective-request records, bounded
// readings, survey result records, and Report/BodyReport
// (docs/verification-design.md §1-§9). Verify publication builds a Report
// from survey outcomes and certified readings in verify_publish.go.

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
	// ToleranceNotEvaluated — no precision decision was made. Two cases
	// share this state: the reading does not exist at all, and the reading
	// exists but its precision was deliberately not judged, as for an
	// invalid body's area and bounds, which are published as boundary data
	// with the tolerance check skipped.
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

// Status is a verdict, used at both the body and the report level
// (verification §6). Unverified is the reserved zero value and is never
// returned by a successful Verify. Severity among verification results is by
// precedence, worst wins: Unsound > Interfering > Violating > Suspect > Sound.
type Status int

const (
	// Unverified: no verification produced this value. It is reserved so a
	// zero or partially decoded Report fails Passed.
	Unverified Status = iota
	// Sound: every body a proven solid, every stated spec met, every asked
	// absence proven, nothing approximate beyond tolerance.
	Sound
	// Suspect: nothing proven wrong and something not proven right — an
	// answer beyond the caller's tolerance, an asked question undecided, an
	// undecided pair.
	Suspect
	// Violating: a spec a VerifyOption stated is proven to fail.
	Violating
	// Interfering: bodies provenly overlap. A report verdict only — overlap
	// is a property of a pair, so no single body is ever Interfering.
	Interfering
	// Unsound: some body is proven not a valid solid.
	Unsound
)

// String renders the status for diagnostics.
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

// ReadingKind names which measured quantity a diagnostic's Observed* form
// carries (verification §1.1) — a named-text enum with a stable String() like
// every other closed set decad owns. Exactly one of Diagnostic.Observed /
// ObservedVec / ObservedBox is non-nil, and this says which: a scalar rides
// Observed, a Centroid ObservedVec, a Bounds box ObservedBox. ReadingNone means
// the reason names no bounded reading and all three are nil.
type ReadingKind int

const (
	// ReadingNone — the reason names no bounded reading; every Observed* is nil.
	ReadingNone ReadingKind = iota
	// ReadingArea — a body's Area (Observed).
	ReadingArea
	// ReadingBounds — a body's Bounds box (ObservedBox).
	ReadingBounds
	// ReadingVolume — a body's Volume (Observed).
	ReadingVolume
	// ReadingCentroid — a body's Centroid (ObservedVec).
	ReadingCentroid
	// ReadingWall — a body's MinWallThickness (Observed).
	ReadingWall
	// ReadingMinRadius — a body's MinRadius (Observed).
	ReadingMinRadius
	// ReadingOverlapVolume — a pair's proven overlap volume (Observed).
	ReadingOverlapVolume
	// ReadingGap — a pair's proven clearance gap (Observed).
	ReadingGap
)

// String renders the pinned lower-snake token — the identity a caller branches
// on and a log prints, never the iota value. An out-of-range value renders
// "reading(<n>)", never a panic (verification §1.1).
func (k ReadingKind) String() string {
	switch k {
	case ReadingNone:
		return "none"
	case ReadingArea:
		return "area"
	case ReadingBounds:
		return "bounds"
	case ReadingVolume:
		return "volume"
	case ReadingCentroid:
		return "centroid"
	case ReadingWall:
		return "wall"
	case ReadingMinRadius:
		return "min_radius"
	case ReadingOverlapVolume:
		return "overlap_volume"
	case ReadingGap:
		return "gap"
	default:
		return fmt.Sprintf("reading(%d)", int(k))
	}
}

// SurveyKind identifies which optional body survey a Diagnostic concerns
// (verification §1.1). SurveyNone means no optional body survey applies — as
// for a core reading or a pair diagnostic — and is NOT an unevaluated marker:
// it is the correct, permanent value for every diagnostic outside the three
// opt-in surveys.
type SurveyKind int

const (
	// SurveyNone — no optional body survey applies; every core or pair
	// diagnostic carries this.
	SurveyNone SurveyKind = iota
	// SurveyWall — the wall-thickness survey (WithMinWallThickness).
	SurveyWall
	// SurveyUndercut — the pull-direction survey (WithPullDirection).
	SurveyUndercut
	// SurveyConcaveRadius — the concave-radius survey (WithConcaveRadius).
	SurveyConcaveRadius
)

// String renders the pinned lower-snake token. An out-of-range value renders
// "survey_kind(<n>)", never a panic (verification §1.1).
func (k SurveyKind) String() string {
	switch k {
	case SurveyNone:
		return "none"
	case SurveyWall:
		return "wall"
	case SurveyUndercut:
		return "undercut"
	case SurveyConcaveRadius:
		return "concave_radius"
	default:
		return fmt.Sprintf("survey_kind(%d)", int(k))
	}
}

// DiagnosticCode is the stable, branchable reason code (verification §1.1) — a
// named-text enum whose stable String() token, never the iota value, is the
// identity a caller and a log share.
type DiagnosticCode int

const (
	// DiagMeasurementBeyondTolerance — a bounded reading's Bound exceeds rel*Ref
	// (§2). Reading names the quantity; its Observed* form carries it, Required
	// is rel*Ref (the largest Bound that would have passed). Body set on a body
	// reading, Pair on a pair reading. Contributes Suspect.
	DiagMeasurementBeyondTolerance DiagnosticCode = iota
	// DiagUndecidedValidity — the held boundary is not decisive beyond its own
	// proven bound (§6). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedValidity
	// DiagInvalidBody — the held boundary is proven not a valid solid (§6).
	// Reading ReadingNone. Contributes Unsound.
	DiagInvalidBody
	// DiagWallTooThin — MinWallThickness's proven interval is below the tool
	// (§6). Reading ReadingWall, Observed the wall reading, Required the tool.
	// Contributes Violating.
	DiagWallTooThin
	// DiagUndercut — a face is a proven undercut against the pull (§6). Reading
	// ReadingNone, every Observed* and Required nil. Contributes Violating.
	DiagUndercut
	// DiagUndecidedWall — the wall survey is undecided: neither answered nor
	// proven absent, OR its proven interval STRADDLES the tool (§6). In the
	// straddle case Reading is ReadingWall with Observed the wall reading and
	// Required the tool; when the survey could not answer at all Reading is
	// ReadingNone and both are nil. Contributes Suspect.
	DiagUndecidedWall
	// DiagUndecidedUndercut — the pull survey could neither prove nor exclude an
	// undercut (§6). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedUndercut
	// DiagUndecidedMinRadius — the concave-radius survey could neither measure
	// nor exclude a concave feature (§6). Reading ReadingNone. Contributes
	// Suspect.
	DiagUndecidedMinRadius
	// DiagInterference — a pair proven to overlap (§1). Reading
	// ReadingOverlapVolume, Observed the overlap volume, Required nil.
	// Contributes Interfering.
	DiagInterference
	// DiagUndecidedPair — a pair the disjoint/overlap PARTITION proof resolved
	// neither way (§1). Reading ReadingNone. Contributes Suspect.
	DiagUndecidedPair
	// DiagUnsupportedPair is the broad compatibility code for a staged pair.
	// Verify no longer emits it; every unsupported pair gets one of the
	// cause-specific codes below instead. Reading ReadingNone. Contributes
	// Suspect.
	//
	// Deprecated: branch on DiagUnsupportedPairPayload,
	// DiagUnsupportedPairContact, or DiagUnsupportedPairPipeline.
	DiagUnsupportedPair
	// DiagUndecidedClearance — a pair PROVEN disjoint (by box or kernel) whose
	// requested WithClearances gap the kernel could not prove: no Clearance row
	// is emitted and the report reads Suspect. Distinct from DiagUndecidedPair
	// (partition unresolved) and the cause-specific unsupported-pair codes
	// (payload, contact, or pipeline) — here the pair is decidedly apart and
	// only the gap is unmeasured. Reading ReadingNone. Contributes Suspect.
	DiagUndecidedClearance
	// DiagUndecidedInterference — a pair PROVEN to overlap whose overlap VOLUME
	// the evaluator cannot bound (§1): the overlap-side mirror of
	// DiagUndecidedClearance. No Interference row is emitted and the report reads
	// Suspect. Reading ReadingNone, Observed and Required nil, Pair set.
	// Contributes Suspect.
	DiagUndecidedInterference
	// DiagUnsupportedPairPayload — one named operand could not enter the read-only
	// intersection: either its mesh carries no occupied-volume proof, or at the
	// chord tolerance the check derives from the pair its own tessellation
	// refused or its held facets where the pair meets are coarser than that
	// tolerance. The message names which. Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedPairPayload
	// DiagUnsupportedPairContact — the pair reaches a contact or near-contact the
	// exact boolean policy cannot classify. Reading ReadingNone. Contributes Suspect.
	// Message names a shared face plane between the operands when they have one,
	// since deepening the overlap cannot resolve that cause.
	DiagUnsupportedPairContact
	// DiagUnsupportedPairPipeline — both operands tessellate, but later boolean
	// geometry exceeds the pipeline's supported reach. Reading ReadingNone.
	// Contributes Suspect.
	DiagUnsupportedPairPipeline
	// DiagUnsupportedPairSheet — a pair holding a sheet operand that the
	// sheet decision procedure could not settle (docs/surface-design.md
	// §9.3): a sheet-sheet pair, since neither operand offers a closed
	// boundary to cast the other against, or a sheet-against-solid pair
	// whose body model is missing, whose candidate enumeration could not
	// decide the boundary distance, or whose witness cast failed. It fires
	// only when the pair's bounds-inflated boxes MEET: a sheet parked away
	// from every solid is decidedly apart and emits nothing, so a model that
	// merely holds a sheet still reads Sound. Reading ReadingNone, Observed*
	// and Required nil, Pair set. Contributes Suspect.
	DiagUnsupportedPairSheet
	// DiagSheetSolidCrossing — a sheet operand proven to cross a solid
	// operand's boundary, by an admitted transversal crossing between a
	// sheet face and a solid face (docs/surface-design.md §9.3). A sheet
	// encloses no region, so no Interference row is emitted for it — that
	// absence is NOT a proven non-overlap here, only the fact that a sheet
	// has no overlap volume to report. Reading ReadingNone, every Observed*,
	// Required and Body nil, Pair set. Contributes Interfering.
	DiagSheetSolidCrossing
	// DiagUnsupportedSurveyPayload — an asked body survey cannot run because
	// its payload class is staged. Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedSurveyPayload
	// DiagSurveyPrerequisite — a requested survey needs a proven solid, and
	// this body does not supply one: its validity is invalid or undecided, OR
	// it is a sheet body asking a wall or concave-radius question, neither of
	// which has any material on a sheet to be about
	// (docs/surface-design.md §9.1). An undercut question is not blocked this
	// way on a proven-valid surface-extruded prism sheet, which answers it
	// over the sheet's own positive side; every other sheet family still
	// reads DiagUnsupportedSurveyPayload for it instead. Survey names the
	// blocked question, Reading ReadingNone. Contributes Suspect.
	DiagSurveyPrerequisite
	// DiagToleranceReferenceUnavailable — a nonzero-bound reading has no
	// usable tolerance reference, so the gate could not judge it. Reading
	// names the quantity, Required nil. Contributes Suspect.
	DiagToleranceReferenceUnavailable
	// DiagMotionCollision — a motion pose proves a (mover, static) pair
	// overlaps (docs/motion-check-design.md §4.1). Pair and At set; Reading
	// ReadingOverlapVolume with Observed the volume when it was bounded, else
	// ReadingNone. Contributes Interfering.
	DiagMotionCollision
	// DiagMotionClearanceViolated — a motion pose proves a pair's gap below
	// the WithMinClearance minimum. Pair and At set, Reading ReadingGap,
	// Observed the gap, Required the minimum. Contributes Violating.
	DiagMotionClearanceViolated
	// DiagMotionUndecidedInterval — a motion interval neither certified clear
	// nor bounded by a proven collision. Pair nil, At the interval's From,
	// Reading ReadingNone; Message names both ends. Contributes Suspect.
	DiagMotionUndecidedInterval
	// DiagMotionUndecidedClearance — a motion interval certified clear whose
	// lower bound does not reach the WithMinClearance minimum while no pose
	// falsifies it. At the interval's From, Reading ReadingGap, Observed the
	// interval's lower bound, Required the minimum. Contributes Suspect.
	DiagMotionUndecidedClearance
	// DiagJointBoxBudgetExhausted — WithCellBudget stopped a joint-box split
	// the resolution would have allowed (docs/linkage-check-design.md §14.2).
	// Raised at most once per report; Pair, Body and Cell nil, Reading
	// ReadingNone; Message names the budget and the cells it held.
	// Contributes Suspect.
	DiagJointBoxBudgetExhausted
)

// String renders the pinned lower-snake token — the identity a caller branches
// on and a log prints, never the iota value. An out-of-range value renders
// "diagnostic(<n>)", never a panic (verification §1.1).
func (c DiagnosticCode) String() string {
	switch c {
	case DiagMeasurementBeyondTolerance:
		return "measurement_beyond_tolerance"
	case DiagUndecidedValidity:
		return "undecided_validity"
	case DiagInvalidBody:
		return "invalid_body"
	case DiagWallTooThin:
		return "wall_too_thin"
	case DiagUndercut:
		return "undercut"
	case DiagUndecidedWall:
		return "undecided_wall"
	case DiagUndecidedUndercut:
		return "undecided_undercut"
	case DiagUndecidedMinRadius:
		return "undecided_min_radius"
	case DiagInterference:
		return "interference"
	case DiagUndecidedPair:
		return "undecided_pair"
	case DiagUnsupportedPair:
		return "unsupported_pair"
	case DiagUndecidedClearance:
		return "undecided_clearance"
	case DiagUndecidedInterference:
		return "undecided_interference"
	case DiagUnsupportedPairPayload:
		return "unsupported_pair_payload"
	case DiagUnsupportedPairContact:
		return "unsupported_pair_contact"
	case DiagUnsupportedPairPipeline:
		return "unsupported_pair_pipeline"
	case DiagUnsupportedPairSheet:
		return "unsupported_pair_sheet"
	case DiagSheetSolidCrossing:
		return "sheet_solid_crossing"
	case DiagUnsupportedSurveyPayload:
		return "unsupported_survey_payload"
	case DiagSurveyPrerequisite:
		return "survey_prerequisite"
	case DiagToleranceReferenceUnavailable:
		return "tolerance_reference_unavailable"
	case DiagMotionCollision:
		return "motion_collision"
	case DiagMotionClearanceViolated:
		return "motion_clearance_violated"
	case DiagMotionUndecidedInterval:
		return "motion_undecided_interval"
	case DiagMotionUndecidedClearance:
		return "motion_undecided_clearance"
	case DiagJointBoxBudgetExhausted:
		return "joint_box_budget_exhausted"
	default:
		return fmt.Sprintf("diagnostic(%d)", int(c))
	}
}
