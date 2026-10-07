package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// This file is the seam conversion of docs/sketch-seam-design.md: the one
// place a live sketch profile becomes the structural records the evaluator
// carries. sketch answers every 2D question and decad consumes the answers —
// nothing here re-derives a trim, projects a point, or fits a curve.

// RecordProfile converts a sketch profile into the structural records a
// the evaluator carries: the region as a [ProfileRecord] — the entity's own
// defining data per boundary edge, plus the recorded range — and the sketch
// plane, read through s.Plane().Frame(), as the [PlaneRecord] that lifts the
// plane-local region into world space. The feature calls run exactly this
// conversion; it is exported so a consumer can record — and therefore vet — a
// profile without a Document.
//
// Admission consumes only a fresh snapshot authenticated against sketch's own
// answers (docs/api-design.md §7): p must name s (Profile.Sketch, else
// [ErrForeignProfile]), be current (Profile.IsStale, else [ErrStaleProfile]),
// and exactly match one fresh result from s.Profiles. Every boundary entity
// must be non-nil and owned by s. A caller-altered snapshot is
// [ErrInvalidProfile], and a foreign boundary entity is [ErrForeignProfile].
// A profile whose Valid is false is also never silently swept
// ([ErrInvalidProfile]). The one further rejection is not a validity judgement:
// an authenticated valid profile whose boundary cannot be recorded exactly —
// a Partial fragment sketch could not certify
// (BoundaryEdge.TExact == false), a certified range the seam's reject-only
// falsifier disproves, or a loop whose source-aware junction check finds a
// contradiction — is [ErrUnrecordableProfile]. decad never repairs, projects
// or fits a point sketch handed over, and it never solves for one.
//
// That last rejection is the one a caller meets by drawing rather than by
// hitting an upstream limit. sketch admits a region on its own proximity
// threshold, so entities whose ends miss by a fraction of a nanometre still
// arrange into one valid profile. At a whole-to-whole join, decad records each
// entity's own points verbatim, so a loop whose recorded coordinates do not
// meet bounds no region. A certified mixed join at a genuinely cut bound
// instead compares the record endpoint with sketch's evaluated node at the
// range falsifier's relative tolerance. An uncut Partial bound uses the
// record's own endpoint, so it remains an exact check. Ends merely driven
// together by a coincidence constraint remain a whole-to-whole exact check:
// the solver converges to within its residual, not to the same coordinate.
func RecordProfile(s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, PlaneRecord, error) {
	profile, plane, _, err := recordProfile(s, p)
	return profile, plane, err
}

// recordProfile returns the authenticated profile's area with its structural
// record so feature callers never read a caller-mutable field after admission.
func recordProfile(s *sketch.Sketch, p *sketch.Profile) (ProfileRecord, PlaneRecord, float64, error) {
	trusted, err := sketchrecord.AdmitProfile(s, p)
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, err
	}

	frame, err := s.Plane().Frame()
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}

	record, err := recordArrangedProfile(trusted)
	if err != nil {
		return ProfileRecord{}, PlaneRecord{}, 0, err
	}
	return record, plane, trusted.Area, nil
}

// recordArrangedProfile records a profile from a fresh sketch arrangement.
// Callers that skip sketchrecord.AuthenticateProfile must own the sketch and
// pass a profile returned by that same arrangement. The record walk checks
// each fragment's TExact claim, range, and loop closure.
func recordArrangedProfile(p *sketch.Profile) (ProfileRecord, error) {
	outer, holes, err := sketchrecord.RecordProfileLoops(p)
	if err != nil {
		return ProfileRecord{}, err
	}
	return ProfileRecord{Outer: outer, Holes: holes}, nil
}

// RecordChain converts a sketch chain — Profile's open counterpart — into the
// structural records the evaluator carries: the open walk as a [ChainRecord]
// and the sketch plane as a [PlaneRecord], read through s.Plane().Frame().
// ExtrudeChain and RevolveChain run exactly this conversion; it is exported so
// a consumer can record — and therefore vet — a chain without a Document.
//
// Every gate RecordProfile runs, RecordChain runs unchanged
// (docs/sketch-seam-design.md §2.2, docs/surface-design.md §13.3): ch must
// name s (Chain.Sketch, else [ErrForeignProfile]), be current (Chain.IsStale,
// else [ErrStaleProfile]), and its exported fields — Entities, Edges, Length,
// Valid, SelfIntersecting — must exactly match one member of a fresh
// s.Chains() result, matched by CONTENT over the whole slice rather than at
// one index: that slice's order consults entity and point names, which
// Sketch.Revision hashes none of, so renaming an entity re-ranks Chains()
// while every held chain stays fresh (a mismatch is [ErrInvalidProfile]). A
// chain whose Valid is false is also never silently swept
// ([ErrInvalidProfile]). The one further rejection is not a validity
// judgement: an authenticated valid chain whose walk decad cannot record
// exactly — a Partial fragment sketch could not certify, a certified range
// the seam's falsifier disproves, or an INTERIOR junction whose two
// coordinates contradict — is [ErrUnrecordableProfile]. A chain's two free
// ends state no junction to check, exactly as a whole closed curve states
// none. Chain.Length enters only the snapshot equality comparison above and
// is never recorded or read as a measurement: it is exact only for a *Line,
// *Arc or *Circle fragment and a sampling-convergent underestimate otherwise,
// with no bound stated for the gap (docs/surface-design.md §13.3).
func RecordChain(s *sketch.Sketch, ch *sketch.Chain) (ChainRecord, PlaneRecord, error) {
	return recordChain(s, ch)
}

func recordChain(s *sketch.Sketch, ch *sketch.Chain) (ChainRecord, PlaneRecord, error) {
	return sketchrecord.RecordChain(s, ch)
}

// authenticateChain keeps root tests on the snapshot check before the validity gate.
func authenticateChain(s *sketch.Sketch, ch *sketch.Chain) (*sketch.Chain, error) {
	return sketchrecord.AuthenticateChain(s, ch)
}

func sameChainSnapshot(a, b *sketch.Chain) bool {
	return sketchrecord.SameChainSnapshot(a, b)
}

type loopJoin = sketchrecord.LoopJoin

func recordEdge(edge sketch.BoundaryEdge) (CurveSegment, error) { return sketchrecord.RecordEdge(edge) }

func edgeJoin(edge sketch.BoundaryEdge, segment CurveSegment) (loopJoin, error) {
	return sketchrecord.EdgeJoin(edge, segment)
}

func falsifyLoopJoins(name string, joins []loopJoin) error {
	return sketchrecord.FalsifyLoopJoins(name, joins)
}

func falsifyChainJoins(joins []loopJoin) error { return sketchrecord.FalsifyChainJoins(joins) }
