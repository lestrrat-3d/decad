package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
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
	record, _, _, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return MeasuredProfile{}, err
	}
	return MeasuredProfile{record: record}, nil
}

// Area measures the profile's plane-local area with a bound.
func (p MeasuredProfile) Area() (Measurement, error) {
	m, err := p.record.Area()
	return m, err
}

// Centroid measures the profile's plane-local centroid with a bound.
// The value is (u, v, 0); use the sketch plane's frame to place it in space.
func (p MeasuredProfile) Centroid() (VecMeasurement, error) {
	m, err := p.record.Centroid()
	return m, err
}

// SecondMoments measures area moments about the sketch plane's origin.
func (p MeasuredProfile) SecondMoments() (SecondMoments, error) {
	uu, uv, vv, err := p.record.SecondMomentReadings()
	if err != nil {
		return SecondMoments{}, err
	}
	return SecondMoments{
		UU: uu, UV: uv,
		VV: vv,
	}, nil
}

// recordChain converts a sketch chain — Profile's open counterpart — into the
// structural records the evaluator carries: the open walk as a chain record
// and the sketch plane as a plane record, read through s.Plane().Frame().
// chain Extrude and chain Revolve run this conversion.
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
	if sectioncert.BoundedSectionSketch(s) {
		return chainRecord{}, planeRecord{},
			fmt.Errorf("%w: this bounded Section chain needs a consumer that carries its source bound", ErrUnsupported)
	}
	return sketchrecord.RecordChain(s, ch)
}

// SecondMoments is a region's bounded second moments of area about the plane
// origin. Each reading has Kind SecondMomentOfArea (mm⁴). Re-reference them to
// another axis with the region's Area and Centroid.
//
// The readings come from the mass-property engine of docs/evaluator-design.md
// §4. sketch decides topology and admissibility; internal/momentregion
// integrates admitted records by closed-form boundary integrals (Green's
// theorem). Line and Tier A free-form walks (docs/spline-design.md Table F)
// integrate to exact rationals, so a region built only from them is published
// as its own rational rounded ONCE and retains a zero bound wherever that
// rational is representable; circular evaluations have no exact rational and
// carry outward bounds instead. Every other free-form kind is unsupported.
// The engine composes internal/proofbound/bounded.go (the bounded-scalar
// arithmetic every published reading is composed in),
// internal/proofbound/rat_interval.go (the exact rational interval arithmetic
// the certified terms are proven in) and internal/circularbounds/moments.go
// (the circular segment's own enclosures).
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, and UV is ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}
