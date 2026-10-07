package decad

import (
	"fmt"
	"slices"

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
	if s == nil || p == nil {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: RecordProfile requires a sketch and a profile`, ErrDegenerate)
	}
	if p.Sketch() != s {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: the profile was built from a different sketch, so its plane-local coordinates are another plane's`, ErrForeignProfile)
	}
	if p.IsStale() {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: the sketch has changed since this profile was built; rebuild with Sketch.Profiles`, ErrStaleProfile)
	}
	if !p.Valid {
		return ProfileRecord{}, PlaneRecord{}, 0, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, ErrInvalidProfile)
	}

	trusted, err := authenticateProfile(s, p)
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
// Callers that skip authenticateProfile must own the sketch and pass a profile
// returned by that same arrangement. recordLoop still checks every fragment's
// TExact claim, its range, and the resulting loop's closure.
func recordArrangedProfile(p *sketch.Profile) (ProfileRecord, error) {
	if !p.Valid {
		return ProfileRecord{}, fmt.Errorf(`%w: a self-intersecting or degenerate region is never silently swept`, ErrInvalidProfile)
	}
	outer, err := recordLoop("outer", p.Outer)
	if err != nil {
		return ProfileRecord{}, err
	}
	var holes []LoopRecord
	for i, h := range p.Holes {
		loop, err := recordLoop(fmt.Sprintf("hole %d", i), h)
		if err != nil {
			return ProfileRecord{}, err
		}
		holes = append(holes, loop)
	}
	return ProfileRecord{Outer: outer, Holes: holes}, nil
}

// authenticateProfile rejects caller changes to the exported Profile fields
// and returns a fresh snapshot built by sketch. Boundary ownership is checked
// first so a foreign or typed-nil entity never reaches Geometry.
func authenticateProfile(s *sketch.Sketch, p *sketch.Profile) (*sketch.Profile, error) {
	owned := make(map[sketch.Entity]struct{})
	for _, ent := range s.Entities() {
		owned[ent] = struct{}{}
	}
	if err := authenticateBoundaryLoop(owned, p.Outer); err != nil {
		return nil, err
	}
	for _, hole := range p.Holes {
		if err := authenticateBoundaryLoop(owned, hole); err != nil {
			return nil, err
		}
	}

	var trusted *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !sameProfileSnapshot(p, candidate) {
			continue
		}
		if trusted != nil {
			return nil, fmt.Errorf(`%w: the profile snapshot matches more than one current region; rebuild with Sketch.Profiles`, ErrInvalidProfile)
		}
		trusted = candidate
	}
	if trusted == nil {
		return nil, fmt.Errorf(`%w: the profile snapshot was altered after Sketch.Profiles returned it; rebuild and pass the profile unchanged`, ErrInvalidProfile)
	}
	return trusted, nil
}

func authenticateBoundaryLoop(owned map[sketch.Entity]struct{}, edges []sketch.BoundaryEdge) error {
	for _, edge := range edges {
		if isNilSketchEntity(edge.Entity) {
			return fmt.Errorf(`%w: the profile boundary contains a nil entity; rebuild with Sketch.Profiles`, ErrInvalidProfile)
		}
		if _, ok := owned[edge.Entity]; !ok {
			return fmt.Errorf(`%w: the profile boundary contains an entity not owned by its source sketch`, ErrForeignProfile)
		}
	}
	return nil
}

func isNilSketchEntity(ent sketch.Entity) bool {
	switch ent := ent.(type) {
	case nil:
		return true
	case *sketch.Line:
		return ent == nil
	case *sketch.Circle:
		return ent == nil
	case *sketch.Arc:
		return ent == nil
	case *sketch.Ellipse:
		return ent == nil
	case *sketch.EllipticalArc:
		return ent == nil
	case *sketch.Conic:
		return ent == nil
	case *sketch.Spline:
		return ent == nil
	case *sketch.ClosedSpline:
		return ent == nil
	case *sketch.FitSpline:
		return ent == nil
	case *sketch.NURBS:
		return ent == nil
	default:
		return false
	}
}

func sameProfileSnapshot(a, b *sketch.Profile) bool {
	if a.Sketch() != b.Sketch() || a.Revision() != b.Revision() ||
		a.Area != b.Area || a.Valid != b.Valid || a.SelfIntersecting != b.SelfIntersecting ||
		(a.Entities == nil) != (b.Entities == nil) || !slices.Equal(a.Entities, b.Entities) ||
		(a.Holes == nil) != (b.Holes == nil) || !sameBoundaryLoop(a.Outer, b.Outer) ||
		len(a.Holes) != len(b.Holes) {
		return false
	}
	for i := range a.Holes {
		if !sameBoundaryLoop(a.Holes[i], b.Holes[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryLoop(a, b []sketch.BoundaryEdge) bool {
	if (a == nil) != (b == nil) || len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameBoundaryEdge(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameBoundaryEdge(a, b sketch.BoundaryEdge) bool {
	return a.Entity == b.Entity &&
		a.Partial == b.Partial &&
		a.Reversed == b.Reversed &&
		a.TStart == b.TStart &&
		a.TEnd == b.TEnd &&
		a.TExact == b.TExact &&
		(a.Polyline == nil) == (b.Polyline == nil) &&
		slices.Equal(a.Polyline, b.Polyline)
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
	if s == nil || ch == nil {
		return ChainRecord{}, PlaneRecord{}, fmt.Errorf(`%w: RecordChain requires a sketch and a chain`, ErrDegenerate)
	}
	if ch.Sketch() != s {
		return ChainRecord{}, PlaneRecord{}, fmt.Errorf(`%w: the chain was built from a different sketch, so its plane-local coordinates are another plane's`, ErrForeignProfile)
	}
	if ch.IsStale() {
		return ChainRecord{}, PlaneRecord{}, fmt.Errorf(`%w: the sketch has changed since this chain was built; rebuild with Sketch.Chains`, ErrStaleProfile)
	}
	if !ch.Valid {
		return ChainRecord{}, PlaneRecord{}, fmt.Errorf(`%w: a self-intersecting or degenerate walk is never silently swept`, ErrInvalidProfile)
	}

	trusted, err := authenticateChain(s, ch)
	if err != nil {
		return ChainRecord{}, PlaneRecord{}, err
	}

	frame, err := s.Plane().Frame()
	if err != nil {
		return ChainRecord{}, PlaneRecord{}, fmt.Errorf(`decad: failed to resolve the sketch plane: %w`, err)
	}
	plane := PlaneRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}

	segs, err := recordChainSegments(trusted.Edges)
	if err != nil {
		return ChainRecord{}, PlaneRecord{}, err
	}
	return ChainRecord{Segments: segs}, plane, nil
}

// authenticateChain rejects caller changes to the exported Chain fields and
// returns a fresh snapshot built by sketch. Boundary ownership is checked
// first so a foreign or typed-nil entity never reaches Geometry — a nil or
// foreign chain entity is [ErrForeignProfile], docs/surface-design.md §13.3's
// own table, rather than [ErrInvalidProfile] as a profile's is: the chain's
// own admission table groups both under the one foreign-source row.
func authenticateChain(s *sketch.Sketch, ch *sketch.Chain) (*sketch.Chain, error) {
	owned := make(map[sketch.Entity]struct{})
	for _, ent := range s.Entities() {
		owned[ent] = struct{}{}
	}
	if err := authenticateChainEdges(owned, ch.Edges); err != nil {
		return nil, err
	}

	var trusted *sketch.Chain
	for _, candidate := range s.Chains() {
		if !sameChainSnapshot(ch, candidate) {
			continue
		}
		if trusted != nil {
			return nil, fmt.Errorf(`%w: the chain snapshot matches more than one current chain; rebuild with Sketch.Chains`, ErrInvalidProfile)
		}
		trusted = candidate
	}
	if trusted == nil {
		return nil, fmt.Errorf(`%w: the chain snapshot was altered after Sketch.Chains returned it; rebuild and pass the chain unchanged`, ErrInvalidProfile)
	}
	return trusted, nil
}

func authenticateChainEdges(owned map[sketch.Entity]struct{}, edges []sketch.BoundaryEdge) error {
	for _, edge := range edges {
		if isNilSketchEntity(edge.Entity) {
			return fmt.Errorf(`%w: the chain contains a nil entity; rebuild with Sketch.Chains`, ErrForeignProfile)
		}
		if _, ok := owned[edge.Entity]; !ok {
			return fmt.Errorf(`%w: the chain contains an entity not owned by its source sketch`, ErrForeignProfile)
		}
	}
	return nil
}

// sameChainSnapshot compares every exported Chain field two values publish —
// Entities, Edges, Length, Valid, SelfIntersecting — the identical set
// docs/surface-design.md §13.3's table requires to match exactly.
// sameBoundaryLoop is Profile's own comparison over []sketch.BoundaryEdge and
// applies unchanged to a Chain's Edges.
func sameChainSnapshot(a, b *sketch.Chain) bool {
	return a.Sketch() == b.Sketch() && a.Revision() == b.Revision() &&
		a.Length == b.Length && a.Valid == b.Valid && a.SelfIntersecting == b.SelfIntersecting &&
		(a.Entities == nil) == (b.Entities == nil) && slices.Equal(a.Entities, b.Entities) &&
		sameBoundaryLoop(a.Edges, b.Edges)
}

type loopJoin = sketchrecord.LoopJoin

func recordChainSegments(edges []sketch.BoundaryEdge) ([]CurveSegment, error) {
	return sketchrecord.RecordChainSegments(edges)
}

func recordLoop(name string, edges []sketch.BoundaryEdge) (LoopRecord, error) {
	return sketchrecord.RecordLoop(name, edges)
}

func recordEdge(edge sketch.BoundaryEdge) (CurveSegment, error) { return sketchrecord.RecordEdge(edge) }

func edgeJoin(edge sketch.BoundaryEdge, segment CurveSegment) (loopJoin, error) {
	return sketchrecord.EdgeJoin(edge, segment)
}

func falsifyLoopJoins(name string, joins []loopJoin) error {
	return sketchrecord.FalsifyLoopJoins(name, joins)
}

func falsifyChainJoins(joins []loopJoin) error { return sketchrecord.FalsifyChainJoins(joins) }
