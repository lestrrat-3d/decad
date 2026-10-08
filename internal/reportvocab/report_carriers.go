package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// DiagnosticPair names the two bodies of a pair diagnostic, in the report's own
// stable pair order (interference design §2).
type DiagnosticPair[BodyT comparable] struct{ A, B BodyT }

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
type Diagnostic[BodyT comparable, CellT any] struct {
	Code        DiagnosticCode              // the stable branch key
	Status      Status                      // the rung this reason contributes
	Body        BodyT                       // the body it concerns; nil for a pair diagnostic
	Pair        *DiagnosticPair[BodyT]      // the pair it concerns; nil for a body diagnostic
	Survey      SurveyKind                  // which optional body survey this concerns; SurveyNone for core/pair reasons
	Reading     ReadingKind                 // which quantity the Observed* form carries; ReadingNone names none
	Observed    *measurement.Measurement    // a scalar reading; nil unless Reading names a scalar quantity
	ObservedVec *measurement.VecMeasurement // a vector reading (a Centroid); nil unless Reading == ReadingCentroid
	ObservedBox *measurement.Box            // a box reading (a Bounds); nil unless Reading == ReadingBounds
	Required    *units.Value                // the threshold the reading was judged against; nil when the reason states none
	At          *units.Value                // the motion parameter a VerifyMotion finding concerns; nil outside motion reports
	Cell        *CellT                      // the joint cell a VerifyJointBox finding concerns; nil outside joint-box reports
	Message     string                      // human-readable; NEVER the branch key
}

// Interference is a proven pairwise overlap, carrying its bounded overlap
// volume (verification §1, interference design §6). Verification computes it
// without consuming either body or changing the document.
type Interference[BodyT comparable] struct {
	A, B   BodyT
	Volume measurement.Measurement
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
type Clearance[BodyT comparable] struct {
	A, B BodyT
	Gap  measurement.Measurement
}
