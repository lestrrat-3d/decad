package decad

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// This file is the seam conversion of docs/sketch-seam-design.md: the one
// place a live sketch profile becomes the structural records the evaluator
// carries. sketch answers every 2D question and decad consumes the answers —
// nothing here re-derives a trim, projects a point, or fits a curve.

// MeasuredProfile is a snapshot of a validated sketch profile for bounded
// plane-local measurements. Its recorded geometry is private.
type MeasuredProfile struct {
	record momentinput.Profile
}

// MeasureProfile authenticates a current profile from s and records its
// boundary for 2D measurement. It returns the feature calls' profile admission
// errors. Neither s nor p may be nil.
func MeasureProfile(s *sketch.Sketch, p *sketch.Profile) (MeasuredProfile, error) {
	record, _, _, err := recordProfile(s, p)
	if err != nil {
		return MeasuredProfile{}, err
	}
	return MeasuredProfile{record: record}, nil
}

// Area measures the profile's plane-local area with a bound.
func (p MeasuredProfile) Area() (Measurement, error) {
	m, err := p.record.Area()
	return measurementFromInternal(m), err
}

// Centroid measures the profile's plane-local centroid with a bound.
// The value is (u, v, 0); use the sketch plane's frame to place it in space.
func (p MeasuredProfile) Centroid() (VecMeasurement, error) {
	m, err := p.record.Centroid()
	return vecMeasurementFromInternal(m), err
}

// SecondMoments measures area moments about the sketch plane's origin.
func (p MeasuredProfile) SecondMoments() (SecondMoments, error) {
	uu, uv, vv, err := p.record.SecondMomentReadings()
	if err != nil {
		return SecondMoments{}, err
	}
	return SecondMoments{
		UU: measurementFromInternal(uu), UV: measurementFromInternal(uv),
		VV: measurementFromInternal(vv),
	}, nil
}

// recordProfile converts a sketch profile into the structural records
// the evaluator carries: the region as a profile record — the entity's own
// defining data per boundary edge, plus the recorded range — and the sketch
// plane, read through s.Plane().Frame(), as the plane record that lifts the
// plane-local region into world space. The feature calls run this conversion.
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
// The return value includes authenticated area so feature callers never read
// a caller-mutable field after admission.
func recordProfile(s *sketch.Sketch, p *sketch.Profile) (profileRecord, planeRecord, float64, error) {
	return momentinput.RecordProfileWithArea(s, p)
}

// recordArrangedProfile records a profile from a fresh sketch arrangement.
// Callers that skip sketchrecord.AuthenticateProfile must own the sketch and
// pass a profile returned by that same arrangement. The record walk checks
// each fragment's TExact claim, range, and loop closure.
func recordArrangedProfile(p *sketch.Profile) (profileRecord, error) {
	outer, holes, err := sketchrecord.RecordProfileLoops(p)
	if err != nil {
		return profileRecord{}, err
	}
	return profileRecord{Outer: outer, Holes: holes}, nil
}

// recordChain converts a sketch chain — Profile's open counterpart — into the
// structural records the evaluator carries: the open walk as a chain record
// and the sketch plane as a plane record, read through s.Plane().Frame().
// ExtrudeChain and RevolveChain run this conversion.
//
// Every profile admission gate also applies to chains
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
func recordChain(s *sketch.Sketch, ch *sketch.Chain) (chainRecord, planeRecord, error) {
	return sketchrecord.RecordChain(s, ch)
}
