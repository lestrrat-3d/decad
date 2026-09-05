package decad

import (
	"fmt"

	"github.com/lestrrat-3d/units"
)

// This file is the vocabulary Document.Verify's report is written in:
// the three enumerations a reading is judged by (Status, ReadingKind,
// DiagnosticCode), the Diagnostic a failed judgement records, and the
// Report/BodyReport shapes the whole verification returns. It holds the
// TYPES alone; verify.go builds them, and every statement about how a
// judgement is reached lives there and in docs/verification-design.md §1-§3.
//
// Each enumeration's String method spells its own constants, so a constant
// added without its rendering is caught by the switch's default arm rather
// than by a reader noticing a bare integer in a report.

// Status is a verdict, used at both the body and the report level
// (verification §6). Unverified is the reserved zero value and is never
// returned by a successful Verify. Severity among verification results is by
// precedence, worst wins: Unsound > Interfering > Violating > Suspect > Sound.
type Status int

const (
	// Unverified: no verification produced this value. It is reserved so a
	// zero or partially decoded Report fails Trustworthy.
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
	// Verify emits this alongside one of the cause-specific codes below.
	// Reading ReadingNone. Contributes Suspect.
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
	// intersection: either its mesh carries no occupied-volume proof, or its own
	// tessellation refused at the chord tolerance the check derives from the pair.
	// The message names which. Reading ReadingNone. Contributes Suspect.
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
	// DiagUnsupportedSurveyPayload — an asked body survey cannot run because
	// its payload class is staged. Reading ReadingNone. Contributes Suspect.
	DiagUnsupportedSurveyPayload
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
	case DiagUnsupportedSurveyPayload:
		return "unsupported_survey_payload"
	default:
		return fmt.Sprintf("diagnostic(%d)", int(c))
	}
}

// DiagnosticPair names the two bodies of a pair diagnostic, in the report's own
// stable pair order (interference design §2).
type DiagnosticPair struct{ A, B *Body }

// Diagnostic is one structured, branchable reason a body or a pair is not Sound
// (verification §1.1). It never decides the verdict — Status is still §6's
// worst-wins aggregate — it explains it. Exactly one of Observed / ObservedVec
// / ObservedBox is non-nil, keyed by Reading (all three nil when
// Reading == ReadingNone).
type Diagnostic struct {
	Code        DiagnosticCode  // the stable branch key
	Status      Status          // the rung this reason contributes
	Body        *Body           // the body it concerns; nil for a pair diagnostic
	Pair        *DiagnosticPair // the pair it concerns; nil for a body diagnostic
	Reading     ReadingKind     // which quantity the Observed* form carries; ReadingNone names none
	Observed    *Measurement    // a scalar reading; nil unless Reading names a scalar quantity
	ObservedVec *VecMeasurement // a vector reading (a Centroid); nil unless Reading == ReadingCentroid
	ObservedBox *Box            // a box reading (a Bounds); nil unless Reading == ReadingBounds
	Required    *units.Value    // the threshold the reading was judged against; nil when the reason states none
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
type Clearance struct {
	A, B *Body
	Gap  Measurement
}

// Report is what Verify returns: the 3D counterpart of
// sketch.VerificationReport (core §10, verification §1).
type Report struct {
	Bodies        []*BodyReport
	Interferences []Interference
	Clearances    []Clearance
	// Diagnostics contains structured, branchable entries for every reason a
	// report returned by Verify is not Sound (verification §1.1). Staged pair
	// causes also carry the deprecated broad compatibility entry. On such a
	// report it is empty EXACTLY when Status == Sound, and Status is the worst
	// Diagnostic.Status in it — the §6 aggregate, itemized. The zero Report is
	// Unverified and carries no verdict.
	Diagnostics []Diagnostic
	Status      Status
}

// Trustworthy is the single bit to gate on: true only when the whole report
// is Sound (verification §6). A zero Report is Unverified and returns false.
func (r *Report) Trustworthy() bool { return r.Status == Sound }

// BodyReport is one live body's verdict and readings (verification §1).
//
// The validity predicates report the boundary the evaluator holds — exact as
// data; what they prove about the PART is Status's to say. A quantity a body
// does not have is absent — nil, never zero: Area and Bounds are boundary
// properties every body has, so they are unconditional; Volume and Centroid
// are region properties only a proven solid has, so both are non-nil exactly
// when the body is one. The opt-in fields are nil unless their option asks
// AND the body is a proven solid AND (for the two feature measures) the
// feature exists: on this evaluator's analytic bodies the surveys decide
// them outright (survey.go), so a nil MinWallThickness or MinRadius on a
// proven solid inside a Sound report is the PROVEN determination that no
// wall / no concave feature exists, and an empty Undercuts the proven
// all-clear (verification §1/§6).
type BodyReport struct {
	Body   *Body
	Status Status

	Solid            bool
	Watertight       bool
	Manifold         bool
	SelfIntersecting bool
	Lumps            int
	Voids            int

	Area   Measurement
	Bounds Box

	Volume   *Measurement
	Centroid *VecMeasurement

	Exactness Exactness

	MinWallThickness *Measurement
	Undercuts        []*Face
	MinRadius        *Measurement
}
