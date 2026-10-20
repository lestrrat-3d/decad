package boundaryaudit

import "github.com/lestrrat-3d/decad/internal/meshbool"

// PairVerdict is the kernel's partition answer for one pair.
type PairVerdict int

const (
	// PairUndecided: the kernel can prove neither shared interior nor
	// disjoint interiors. It joins neither report list and reads Suspect.
	PairUndecided PairVerdict = iota
	// PairDisjoint: boundary clearance proven positive and nesting excluded.
	PairDisjoint
	// PairTouching: every zero-distance contact certified (the coplanar
	// plane-pair certificate or a ruling certificate) — the gap is a
	// measured Exact zero (§6).
	PairTouching
	// PairOverlapping: shared interior is proven by nesting or another
	// admitted certificate. A row still requires a complete bounded overlap
	// volume from containment or the read-only boolean evaluator.
	PairOverlapping
)

// SheetSolidVerdict is docs/surface-design.md §9.3's pair decision procedure's
// answer for a pair holding exactly one sheet operand against a proven solid.
// A sheet encloses no region, so §1's four interior relations do not apply to
// this pair shape at all; these four answers replace them.
type SheetSolidVerdict int

const (
	// SheetSolidUndecided — a body model was missing, the candidate
	// enumeration could not decide the boundary distance, or the witness
	// cast failed. Neither Interfering, Sound, nor a Clearance row follows.
	SheetSolidUndecided SheetSolidVerdict = iota
	// SheetSolidCrossing — an admitted transversal crossing between a sheet
	// face and a solid face proves the sheet crosses the solid's boundary.
	SheetSolidCrossing
	// SheetSolidContained — the boundary distance is proven positive, and the
	// witness vertex is proven inside the solid.
	SheetSolidContained
	// SheetSolidOutside — the same proof, with the witness proven outside.
	SheetSolidOutside
)

// SheetSolidResult is one sheet-against-solid pair's decision. Lo, Hi, Exact
// and Diam are populated only for SheetSolidContained and SheetSolidOutside,
// and they carry the SAME boundary-distance interval regardless of which side
// the witness landed on (docs/api-design.md §6.2): the interval is proven
// before the witness cast ever runs, so containment and separation share one
// Clearance row that states the distance to the solid's boundary without
// asserting which side the sheet is on.
type SheetSolidResult struct {
	Verdict SheetSolidVerdict
	Lo, Hi  float64
	Exact   bool
	Diam    float64
}

// InterferenceOutcome distinguishes why the overlap volume could not be
// measured, so Verify can pick the matching diagnostic (verification §1.1).
// Payload staging retains the first operand that failed; contact policy and
// in-pipeline reach remain separate from it and from an undecided proof.
type InterferenceOutcome int

const (
	// InterferenceMeasured — a positive bounded overlap volume was proven.
	InterferenceMeasured InterferenceOutcome = iota
	// InterferenceUndecided — the read-only proof resolved neither way.
	InterferenceUndecided
	// InterferenceUnsupportedPayloadFirst — the first operand cannot enter the
	// read-only intersection pipeline.
	InterferenceUnsupportedPayloadFirst
	// InterferenceUnsupportedPayloadSecond — the second operand cannot enter
	// the read-only intersection pipeline.
	InterferenceUnsupportedPayloadSecond
	// InterferenceUnsupportedContact — the exact boolean contact policy refused
	// the pair.
	InterferenceUnsupportedContact
	// InterferenceUnsupportedPipeline — both operands entered the pipeline, but
	// later geometry exceeded its supported reach.
	InterferenceUnsupportedPipeline
	// InterferenceUnsupportedVolumeProofFirst — the first operand meshes, but its
	// mesh carries no occupied-volume proof, so the read-only intersection cannot
	// compose it.
	InterferenceUnsupportedVolumeProofFirst
	// InterferenceUnsupportedVolumeProofSecond — the same for the second operand.
	InterferenceUnsupportedVolumeProofSecond
)

// InterferenceOutcomeForExpected preserves the read-only boolean's private
// reason taxonomy for Verify's cause-specific diagnostics.
func InterferenceOutcomeForExpected(expected *meshbool.BooleanExpectedError) InterferenceOutcome {
	switch expected.Kind {
	case meshbool.BooleanExpectedContact:
		return InterferenceUnsupportedContact
	case meshbool.BooleanExpectedUnsupported:
		return InterferenceUnsupportedPipeline
	case meshbool.BooleanExpectedStaging:
		if expected.Operand == 1 {
			return InterferenceUnsupportedPayloadSecond
		}
		return InterferenceUnsupportedPayloadFirst
	case meshbool.BooleanExpectedVolumeProof:
		if expected.Operand == 1 {
			return InterferenceUnsupportedVolumeProofSecond
		}
		return InterferenceUnsupportedVolumeProofFirst
	default:
		return InterferenceUndecided
	}
}

// SheetAuditOutcome is AuditSheetBoundary's three-way verdict.
type SheetAuditOutcome int

const (
	// SheetAuditProven — every structural leg holds and non-self-intersection
	// is admitted by construction.
	SheetAuditProven SheetAuditOutcome = iota
	// SheetAuditViolated — a structural leg is proven to fail.
	SheetAuditViolated
	// SheetAuditUndecided — every structural leg holds, but this payload does
	// not admit non-self-intersection by construction.
	SheetAuditUndecided
)
