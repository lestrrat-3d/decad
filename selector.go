package decad

import (
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// selector seals the two query families inside this package.
type selector interface{ selector() }

const (
	selKindEdges = "edges"
	selKindFaces = "faces"
)

// This file is the selector vocabulary of docs/api-design.md §9: intent, not
// identity. A feature is given a query, never a pointer — handles do not
// survive an edit and index order is not stable, so an edge or face is named
// by geometric predicate and provenance instead. Features accept the
// interfaces (EdgeSelector/FaceSelector); the constructors return the
// concrete query types that implement them and carry the cardinality
// assertions. Both interfaces embed a private root so callers cannot add
// selector variants that features do not understand.
//
// Resolution is a filter pipeline over the body's live topology
// (docs/evaluator-design.md §7): gather (Body.Edges()/Faces()), apply each
// predicate as a pure function of the analytic data, then enforce the
// cardinality assertion. Matching is decided on what an entity IS — a
// predicate that needs analytic identity an entity does not have simply does
// not match it — and the result keeps the topology accessors' order.

// EdgeSelector is what an edge-consuming feature (fillet, chamfer) accepts:
// an unresolved edge query.
type EdgeSelector interface {
	selector
	// SelectEdges resolves the query against a live body's topology.
	SelectEdges(*Body) ([]*Edge, error)
}

// FaceSelector is what a face-consuming feature (shell) accepts: an
// unresolved face query. It embeds the sealed selector root, so every
// selector a feature accepts is recordable (core §9/§6.2).
type FaceSelector interface {
	selector
	// SelectFaces resolves the query against a live body's topology.
	SelectFaces(*Body) ([]*Face, error)
}

// cardKind discriminates the cardinality assertion a query carries.
type cardKind int

const (
	// cardNone asserts nothing: zero matches at resolve is ErrNoMatch.
	cardNone cardKind = iota
	// cardExactly asserts exactly n matches; anything else is ErrCardinality.
	cardExactly
	// cardAtLeast asserts at least n matches; fewer is ErrCardinality.
	cardAtLeast
)

// cardinality is the recorded cardinality assertion of a query. The zero
// value asserts nothing.
type cardinality struct {
	kind cardKind
	n    int
}

// EdgeQuery is the concrete edge selector: a conjunction of edge predicates
// plus an optional cardinality assertion. Build one with Edges.
type EdgeQuery struct {
	preds []EdgePredicate
	card  cardinality
}

// FaceQuery is the concrete face selector: a conjunction of face predicates
// plus an optional cardinality assertion. Build one with Faces.
type FaceQuery struct {
	preds []FacePredicate
	card  cardinality
}

// Edges returns a query matching every edge that satisfies all of preds; no
// predicates matches every edge. A query that matches nothing at resolve is
// an error, loudly — ErrNoMatch, or ErrCardinality when asserted (core §9).
func Edges(preds ...EdgePredicate) *EdgeQuery {
	return &EdgeQuery{preds: slices.Clone(preds)}
}

// Faces returns a query matching every face that satisfies all of preds; no
// predicates matches every face. A query that matches nothing at resolve is
// an error, loudly — ErrNoMatch, or ErrCardinality when asserted (core §9).
func Faces(preds ...FacePredicate) *FaceQuery {
	return &FaceQuery{preds: slices.Clone(preds)}
}

// SelectEdges resolves the query against the body's topology: gather
// (Body.Edges(), whose order the result keeps), filter by each predicate,
// then enforce the cardinality assertion (docs/evaluator-design.md §7). A
// nil body has no topology to select from and is ErrDegenerate; zero matches
// is ErrCardinality when asserted, else ErrNoMatch (core §12 precedence).
func (q *EdgeQuery) SelectEdges(body *Body) ([]*Edge, error) {
	if q == nil {
		return nil, errNilSelector
	}
	if body == nil {
		return nil, fmt.Errorf(`%w: a nil body has no edges to select`, ErrDegenerate)
	}
	// Predicates are validated up front, so a degenerate direction or a
	// malformed length is rejected regardless of what the body holds.
	for _, p := range q.preds {
		if err := p.validate(); err != nil {
			return nil, err
		}
	}
	edges := body.Edges()
	matched := make([]*Edge, 0, len(edges))
	for _, e := range edges {
		if !edgeMatchesAll(e, q.preds) {
			continue
		}
		matched = append(matched, e)
	}
	if err := q.card.enforce(len(matched), "edges"); err != nil {
		return nil, q.enrich(body, len(matched), err)
	}
	return matched, nil
}

// enrich turns a cardinality-enforcement failure into a SelectionError with
// the per-clause residuals an agent needs to repair the query (core §9). A
// malformed-count assertion (Exactly(0) and the like) is ErrDegenerate, not a
// selection outcome, and passes through unwrapped.
func (q *EdgeQuery) enrich(body *Body, matched int, err error) error {
	switch {
	case errors.Is(err, ErrNoMatch):
		return q.selectionError(body, matched, q.card.expected(), ErrNoMatch)
	case errors.Is(err, ErrCardinality):
		return q.selectionError(body, matched, q.card.expected(), ErrCardinality)
	default:
		return err
	}
}

// SelectFaces resolves the query against the body's topology: gather
// (Body.Faces(), whose order the result keeps), filter by each predicate,
// then enforce the cardinality assertion (docs/evaluator-design.md §7). A
// nil body has no topology to select from and is ErrDegenerate; zero matches
// is ErrCardinality when asserted, else ErrNoMatch (core §12 precedence).
func (q *FaceQuery) SelectFaces(body *Body) ([]*Face, error) {
	if q == nil {
		return nil, errNilSelector
	}
	if body == nil {
		return nil, fmt.Errorf(`%w: a nil body has no faces to select`, ErrDegenerate)
	}
	for _, p := range q.preds {
		if err := p.validate(); err != nil {
			return nil, err
		}
	}
	faces := body.Faces()
	matched := make([]*Face, 0, len(faces))
	for _, f := range faces {
		if !faceMatchesAll(f, q.preds) {
			continue
		}
		matched = append(matched, f)
	}
	if err := q.card.enforce(len(matched), "faces"); err != nil {
		return nil, q.enrich(body, len(matched), err)
	}
	return matched, nil
}

// enrich is the face analog of EdgeQuery.enrich.
func (q *FaceQuery) enrich(body *Body, matched int, err error) error {
	switch {
	case errors.Is(err, ErrNoMatch):
		return q.selectionError(body, matched, q.card.expected(), ErrNoMatch)
	case errors.Is(err, ErrCardinality):
		return q.selectionError(body, matched, q.card.expected(), ErrCardinality)
	default:
		return err
	}
}

// enforce applies the recorded cardinality assertion to a match count: a
// failed assertion is ErrCardinality even at zero matches, and ErrNoMatch is
// reserved for a query that asserts nothing and matched nothing (core §12).
func (c cardinality) enforce(n int, what string) error {
	if c.kind != cardNone && c.n <= 0 {
		// Exactly(0)/AtLeast(0) would let "matches nothing" read as
		// success — the outcome core §9 makes an error — and a negative
		// count asserts nothing at all. Both are malformed questions.
		return fmt.Errorf(`%w: a cardinality assertion needs a positive count, got %d`, ErrDegenerate, c.n)
	}
	switch c.kind {
	case cardNone:
		if n == 0 {
			return fmt.Errorf(`%w: the query matched no %s`, ErrNoMatch, what)
		}
	case cardExactly:
		if n != c.n {
			return fmt.Errorf(`%w: the query matched %d %s, asserted exactly %d`, ErrCardinality, n, what, c.n)
		}
	case cardAtLeast:
		if n < c.n {
			return fmt.Errorf(`%w: the query matched %d %s, asserted at least %d`, ErrCardinality, n, what, c.n)
		}
	default:
		return fmt.Errorf(`%w: unknown cardinality kind %d`, ErrDegenerate, int(c.kind))
	}
	return nil
}

// Exactly asserts the query resolves to exactly n matches; anything else —
// zero included — is ErrCardinality at resolve. It replaces any earlier
// cardinality assertion and returns the receiver for chaining.
func (q *EdgeQuery) Exactly(n int) *EdgeQuery {
	q.card = cardinality{kind: cardExactly, n: n}
	return q
}

// AtLeast asserts the query resolves to at least n matches; fewer is
// ErrCardinality at resolve. It replaces any earlier cardinality assertion
// and returns the receiver for chaining.
func (q *EdgeQuery) AtLeast(n int) *EdgeQuery {
	q.card = cardinality{kind: cardAtLeast, n: n}
	return q
}

// Exactly asserts the query resolves to exactly n matches; anything else —
// zero included — is ErrCardinality at resolve. It replaces any earlier
// cardinality assertion and returns the receiver for chaining.
func (q *FaceQuery) Exactly(n int) *FaceQuery {
	q.card = cardinality{kind: cardExactly, n: n}
	return q
}

// AtLeast asserts the query resolves to at least n matches; fewer is
// ErrCardinality at resolve. It replaces any earlier cardinality assertion
// and returns the receiver for chaining.
func (q *FaceQuery) AtLeast(n int) *FaceQuery {
	q.card = cardinality{kind: cardAtLeast, n: n}
	return q
}

// The queries seal into the private selector root.
func (q *EdgeQuery) selector() {}
func (q *FaceQuery) selector() {}

// EdgePredicate is one clause of an EdgeQuery. Predicates come
// only from the package constructors — Convex, Concave, ParallelTo,
// EndpointAt, LongerThan, CreatedBy, Circular, Free — and compose by conjunction; the zero
// value names no predicate and is rejected at resolve.
type EdgePredicate struct {
	kind   string
	dir    r3.Vec
	point  r3.Vec
	length units.Value
	ref    FeatureRef
}

// FacePredicate is one clause of a FaceQuery. Predicates come
// only from the package constructors — Planar, Cylindrical, NormalTo,
// Facing, FaceCreatedBy — and compose by conjunction; the zero value names no
// predicate and is rejected at resolve.
type FacePredicate struct {
	kind string
	dir  r3.Vec
	ref  FeatureRef
}

// Stable predicate names are shared by query rendering and resolution.
const (
	predKindConvex        = "convex"
	predKindConcave       = "concave"
	predKindParallelTo    = "parallel_to"
	predKindEndpointAt    = "endpoint_at"
	predKindLongerThan    = "longer_than"
	predKindCreatedBy     = "created_by"
	predKindCircular      = "circular"
	predKindPlanar        = "planar"
	predKindCylindrical   = "cylindrical"
	predKindNormalTo      = "normal_to"
	predKindFacing        = "facing"
	predKindFaceCreatedBy = "face_created_by"
	predKindFree          = "free"
)

// Convex matches edges Edge.IsConvex reports convex — the walked-boundary
// convention, not the 3D material angle across the edge. Read Edge.IsConvex
// before selecting on it: a hole's rim edges are CONCAVE, so this predicate
// never picks them.
func Convex() EdgePredicate { return EdgePredicate{kind: predKindConvex} }

// Concave matches edges Edge.IsConvex reports concave — the walked-boundary
// convention, not the 3D material angle across the edge. This is the predicate
// that picks a hole's rim edges, and the rim of a concave round bitten out of
// the outer boundary.
func Concave() EdgePredicate { return EdgePredicate{kind: predKindConcave} }

// ParallelTo matches edges whose direction is parallel to v. The vector is
// recorded exactly as given — a dimensionless direction (core §5.2); a
// degenerate (zero or non-finite) direction is rejected at resolve, not here.
func ParallelTo(v r3.Vec) EdgePredicate {
	return EdgePredicate{kind: predKindParallelTo, dir: v}
}

// EndpointAt matches an edge whose stored start or end position equals p
// component-wise. It uses no tolerance; a non-finite p fails at resolve.
func EndpointAt(p r3.Vec) EdgePredicate {
	return EdgePredicate{kind: predKindEndpointAt, point: p}
}

// LongerThan matches edges strictly longer than l. The quantity is recorded
// exactly as given; a non-length or negative l is rejected at resolve
// (ErrUnitKind / ErrNegativeMagnitude), not here.
func LongerThan(l units.Value) EdgePredicate {
	return EdgePredicate{kind: predKindLongerThan, length: l}
}

// CreatedBy matches edges by provenance: edges created by the feature role f
// names. Provenance is structural, so it survives body rebuilds
// (docs/evaluator-design.md §3). A negative producer identity or empty role is rejected at
// resolve as ErrDegenerate.
func CreatedBy(f FeatureRef) EdgePredicate {
	return EdgePredicate{kind: predKindCreatedBy, ref: f}
}

// Circular matches edges whose curve is a full circle or a circular arc.
func Circular() EdgePredicate { return EdgePredicate{kind: predKindCircular} }

// Free matches edges Edge.IsFree reports free — exactly one adjacent face, a
// sheet's boundary (docs/surface-design.md §2.2, §3.1). It composes with
// every other clause: Edges(Free(), Circular()) picks the circular free
// edges, Edges(Free()).Exactly(8) asserts a surface extrude's rim count. On a
// body with no free edge — every solid, and a closed sheet — it matches
// nothing, an ordinary ErrNoMatch at resolve or ErrCardinality under an
// assertion, never an error in itself.
func Free() EdgePredicate { return EdgePredicate{kind: predKindFree} }

// Planar matches faces whose surface is a plane.
func Planar() FacePredicate { return FacePredicate{kind: predKindPlanar} }

// Cylindrical matches faces whose surface is a cylinder. On a Faceted body
// the analytic identity is gone, so it matches nothing there — matching is
// decided on what the face IS (docs/evaluator-design.md §7).
func Cylindrical() FacePredicate { return FacePredicate{kind: predKindCylindrical} }

// NormalTo matches planar faces whose normal is parallel to v. The vector is
// recorded exactly as given — a dimensionless direction (core §5.2); a
// degenerate (zero or non-finite) direction is rejected at resolve, not here.
func NormalTo(v r3.Vec) FacePredicate {
	return FacePredicate{kind: predKindNormalTo, dir: v}
}

// Facing matches the planar face whose OUTWARD (material-leaving) normal points
// ALONG v — parallel to v AND the same sense, a positive projection. NormalTo
// matches on either sense, so a slab's two parallel caps both match NormalTo(z);
// Facing(z) picks only the top one. The vector is recorded exactly as given — a
// dimensionless direction (core §5.2); a degenerate (zero or non-finite)
// direction is rejected at resolve, not here.
func Facing(v r3.Vec) FacePredicate {
	return FacePredicate{kind: predKindFacing, dir: v}
}

// FaceCreatedBy matches faces by provenance — the face analog of CreatedBy.
// A canonicalization merge unions the merged faces' roles and this matches
// on any of them, so provenance survives the merge
// (docs/evaluator-design.md §3). A negative producer identity or empty role is rejected at
// resolve as ErrDegenerate.
func FaceCreatedBy(f FeatureRef) FacePredicate {
	return FacePredicate{kind: predKindFaceCreatedBy, ref: f}
}

// CapStart names the start-cap role of b's own producing feature, so
// FaceCreatedBy(CapStart(b)) selects that cap without a "capStart" string
// literal a typo could break. It reads b.Origin().producer (§6) and pairs it with
// the fixed role. The role exists only where the feature mints it — an extrude, a
// partial revolve, a shell that built a tube, or a Placed/PlacedCopy/Duplicate
// of one; on any other body the ref is still well-formed and simply matches
// nothing (an ordinary ErrNoMatch at resolve). To name a cap a boolean's
// operand contributed, use FaceCreatedBy(CapStart(originalBody)) against the
// upstream body, never CapStart of the boolean result.
func CapStart(b *Body) FeatureRef {
	return FeatureRef{producer: b.Origin().producer, Role: roleCapStart}
}

// CapEnd names the end-cap role of b's own producing feature — the sibling of
// CapStart. A body whose feature mints no end cap (a cup, a full revolution) still
// gets a well-formed ref that matches nothing.
func CapEnd(b *Body) FeatureRef {
	return FeatureRef{producer: b.Origin().producer, Role: roleCapEnd}
}

// parallelEps decides "parallel": two directions are parallel when the
// magnitude of their cross product is within this relative tolerance of zero
// — sign-insensitive, so either sense matches.
const parallelEps = 1e-9

// parallelDirs reports whether the two nonzero directions are parallel,
// either sense.
//
// The ordinary path is the exact float comparison cross <= eps*la*lb, so a
// direction of any ordinary magnitude resolves bit-for-bit as it always has.
// That comparison breaks only for a finite but EXTREME caller-supplied
// direction (near math.MaxFloat64, or the smallest normals), where Len squares
// the components: a MaxFloat64 direction overflows a length or the tolerance
// product to +Inf (so the test reads finite <= +Inf and EVERY partner reads
// parallel), and a subnormal one underflows a length or the product to 0. Only
// in those cases does the code rescale to infinity-norm 1 — scale-invariant, so
// it changes no ordinary answer — where nothing overflows or underflows.
func parallelDirs(a, b r3.Vec) bool {
	if zeroVec(a) || zeroVec(b) {
		return false
	}
	la, lb := a.Len(), b.Len()
	cl := a.Cross(b).Len()
	prod := parallelEps * la * lb
	if !math.IsInf(la, 0) && !math.IsInf(lb, 0) && !math.IsInf(cl, 0) &&
		!math.IsInf(prod, 0) && prod > 0 {
		return cl <= prod
	}
	// A length overflowed to +Inf or the product underflowed to 0 — a genuine
	// extreme direction. Rescale both to infinity-norm 1 and retry.
	a, b = scaleToUnitInfNorm(a), scaleToUnitInfNorm(b)
	return a.Cross(b).Len() <= parallelEps*a.Len()*b.Len()
}

// scaleToUnitInfNorm returns v scaled by the reciprocal of its
// largest-magnitude component, so the result has infinity-norm 1 and names the
// same ray. It keeps a finite-but-extreme direction (near math.MaxFloat64, or
// near the smallest normals) from overflowing or underflowing when a later
// step takes its Euclidean length. The caller guarantees v is finite and
// nonzero (validateDirection), so the divisor is finite and nonzero.
func scaleToUnitInfNorm(v r3.Vec) r3.Vec {
	m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	return r3.NewVec(v.X/m, v.Y/m, v.Z/m)
}

// validateDirection gates a caller-supplied predicate direction at resolve:
// a non-finite component is ErrNotFinite and the zero vector is ErrDegenerate
// — it names no direction to compare against.
func validateDirection(v r3.Vec, what string) error {
	for _, c := range []float64{v.X, v.Y, v.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return fmt.Errorf(`%w: a %s direction component is not finite`, ErrNotFinite, what)
		}
	}
	if zeroVec(v) {
		return fmt.Errorf(`%w: a zero %s direction names no direction`, ErrDegenerate, what)
	}
	return nil
}

// validatePredicateRef rejects provenance that cannot name a feature role.
func validatePredicateRef(ref FeatureRef, what string) error {
	if ref.producer < 0 {
		return fmt.Errorf(`%w: a %s predicate cannot reference negative producer %d`, ErrDegenerate, what, ref.producer)
	}
	if ref.Role == "" {
		return fmt.Errorf(`%w: a %s predicate requires a non-empty provenance role`, ErrDegenerate, what)
	}
	return nil
}

// validate gates one edge clause's recorded parameters at resolve
// (core §9/§12): a degenerate or non-finite direction, and a non-length,
// non-finite or negative LongerThan quantity, are rejected before any edge
// is examined. A kind the constructors never produce is malformed input.
func (p EdgePredicate) validate() error {
	switch p.kind {
	case predKindConvex, predKindConcave, predKindCircular, predKindFree:
		return nil
	case predKindCreatedBy:
		return validatePredicateRef(p.ref, "created-by")
	case predKindParallelTo:
		return validateDirection(p.dir, "parallel-to")
	case predKindEndpointAt:
		for _, c := range []float64{p.point.X, p.point.Y, p.point.Z} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return fmt.Errorf(`%w: an endpoint-at position component is not finite`, ErrNotFinite)
			}
		}
		return nil
	case predKindLongerThan:
		_, err := magnitudeIn(p.length, units.Length, units.Millimeter, "the longer-than length")
		return err
	case "":
		return fmt.Errorf(`%w: edge predicate names no kind; use the package constructors`, ErrDegenerate)
	default:
		return fmt.Errorf(`%w: unknown edge predicate kind %q`, ErrDegenerate, p.kind)
	}
}

// validate gates one face clause's recorded parameters at resolve, the face
// analog of EdgePredicate.validate.
func (p FacePredicate) validate() error {
	switch p.kind {
	case predKindPlanar, predKindCylindrical:
		return nil
	case predKindFaceCreatedBy:
		return validatePredicateRef(p.ref, "face-created-by")
	case predKindNormalTo:
		return validateDirection(p.dir, "normal-to")
	case predKindFacing:
		return validateDirection(p.dir, "facing")
	case "":
		return fmt.Errorf(`%w: face predicate names no kind; use the package constructors`, ErrDegenerate)
	default:
		return fmt.Errorf(`%w: unknown face predicate kind %q`, ErrDegenerate, p.kind)
	}
}

// edgeMatchesAll reports whether the edge satisfies every clause — predicates
// compose by conjunction (core §9). Predicates were validated up front.
func edgeMatchesAll(e *Edge, preds []EdgePredicate) bool {
	for _, p := range preds {
		if !p.matches(e) {
			return false
		}
	}
	return true
}

// faceMatchesAll reports whether the face satisfies every clause.
func faceMatchesAll(f *Face, preds []FacePredicate) bool {
	for _, p := range preds {
		if !p.matches(f) {
			return false
		}
	}
	return true
}

// matches decides one edge clause on the analytic data the edge holds
// (docs/evaluator-design.md §7):
//
//   - convex/concave read the decided IsConvex answer — the walked-boundary
//     convention, so a hole's rim edges are concave;
//   - parallel_to compares a LINEAR edge's direction (start vertex toward
//     end vertex) against the recorded vector, either sense — a curved edge
//     has no single direction, so it does not match;
//   - endpoint_at compares either stored endpoint with the stated position
//     component-wise, without a tolerance;
//   - longer_than compares Edge.Length() strictly against the recorded
//     quantity;
//   - created_by matches provenance through the edge's adjacent faces: an
//     edge is created by the role that created a face it bounds, so it
//     matches when ANY adjacent face's Origins() carries the ref;
//   - circular matches an edge whose curve is a full circle or a circular
//     arc;
//   - free matches an edge Edge.IsFree reports free — exactly one adjacent
//     face.
func (p EdgePredicate) matches(e *Edge) bool {
	switch p.kind {
	case predKindConvex:
		return e.convex
	case predKindConcave:
		return !e.convex
	case predKindFree:
		return e.IsFree()
	case predKindParallelTo:
		if _, ok := e.curve.(Line3); !ok {
			return false
		}
		return parallelDirs(e.end.position.Sub(e.start.position), p.dir)
	case predKindEndpointAt:
		return e.start.position == p.point || e.end.position == p.point
	case predKindLongerThan:
		// validate ran magnitudeIn already, so the conversion cannot fail.
		mm, err := p.length.In(units.Millimeter)
		if err != nil {
			return false
		}
		return e.length > mm
	case predKindCreatedBy:
		for _, f := range e.faces {
			if slices.Contains(f.origins, p.ref) {
				return true
			}
		}
		return false
	case predKindCircular:
		switch e.curve.(type) {
		case Circle3, Arc3:
			return true
		default:
			return false
		}
	default:
		return false
	}
}

// matches decides one face clause on the analytic data the face holds
// (docs/evaluator-design.md §7):
//
//   - planar/cylindrical match the Surface variant — matching is decided on
//     what the face IS, so a face whose analytic identity is gone (Faceted)
//     matches neither;
//   - normal_to matches a PLANAR face whose plane normal is parallel to the
//     recorded vector, either sense;
//   - facing matches a PLANAR face whose OUTWARD (material-leaving) normal
//     points along the recorded vector — parallel AND the same sense, so one
//     of a slab's two parallel caps, never both;
//   - face_created_by matches when the face's Origins() carries the ref —
//     a canonicalization merge unions roles, and any of them matches.
func (p FacePredicate) matches(f *Face) bool {
	switch p.kind {
	case predKindPlanar:
		_, ok := f.surface.(Plane)
		return ok
	case predKindCylindrical:
		_, ok := f.surface.(Cylinder)
		return ok
	case predKindNormalTo:
		pl, ok := f.surface.(Plane)
		if !ok {
			return false
		}
		return parallelDirs(pl.Frame.N(), p.dir)
	case predKindFacing:
		pl, ok := f.surface.(Plane)
		if !ok {
			return false
		}
		// The face's outward normal is the plane normal flipped when the
		// material lies on the +N side (Face.reversed, the same sign
		// Face.NormalAt applies). Facing wants it pointing ALONG v: parallel
		// AND a positive projection — same-sense, so exactly one of a slab's
		// two caps.
		n := pl.Frame.N()
		if f.reversed {
			n = n.Scale(-1)
		}
		// Scale the recorded direction down by its largest-magnitude
		// component before the length-based parallel test. A finite but huge
		// direction (a component near math.MaxFloat64) overflows to +Inf when
		// its length squares the components, which would make every finite
		// cross length compare parallel; a finite but tiny one underflows the
		// same way. validate already rejected the zero and non-finite vectors,
		// so the largest component is finite and nonzero and the scaled
		// direction names the same ray, robustly. The sign of the dot is
		// unchanged by a positive scale.
		d := scaleToUnitInfNorm(p.dir)
		return parallelDirs(n, d) && n.Dot(d) > 0
	case predKindFaceCreatedBy:
		return slices.Contains(f.origins, p.ref)
	default:
		return false
	}
}

// errNilSelector rejects a nil query.
var errNilSelector = fmt.Errorf(`%w: nil selector`, ErrDegenerate)
