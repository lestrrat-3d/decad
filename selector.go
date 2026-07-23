package decad

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the selector vocabulary of docs/api-design.md §9: intent, not
// identity. A feature is given a query, never a pointer — handles do not
// survive an edit and index order is not stable, so an edge or face is named
// by geometric predicate and provenance instead. Features accept the
// interfaces (EdgeSelector/FaceSelector); the constructors return the
// concrete query types that implement them and carry the cardinality
// assertions. Both interfaces embed the sealed Selector root (recipe.go), so
// every selector a feature accepts is a value a Recipe can record, and the
// serializability rule of core §6.2 reaches the query types: a query is
// recorded content — its predicates and its cardinality assertion — and it
// ships with its tagged codec below.
//
// Resolution is a filter pipeline over the body's live topology
// (docs/evaluator-design.md §7): gather (Body.Edges()/Faces()), apply each
// predicate as a pure function of the analytic data, then enforce the
// cardinality assertion. Matching is decided on what an entity IS — a
// predicate that needs analytic identity an entity does not have simply does
// not match it — and the result keeps the topology accessors' order, so a
// recipe replay selects identically.

// EdgeSelector is what an edge-consuming feature (fillet, chamfer) accepts:
// an unresolved edge query. It embeds the sealed Selector root, so every
// selector a feature accepts is recordable (core §9/§6.2).
type EdgeSelector interface {
	Selector
	// SelectEdges resolves the query against a live body's topology.
	SelectEdges(*Body) ([]*Edge, error)
}

// FaceSelector is what a face-consuming feature (shell) accepts: an
// unresolved face query. It embeds the sealed Selector root, so every
// selector a feature accepts is recordable (core §9/§6.2).
type FaceSelector interface {
	Selector
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
// plus an optional cardinality assertion. Build one with Edges; it is an
// EdgeSelector, and — sealed into Selector — what a Recipe Step stores.
type EdgeQuery struct {
	preds []EdgePredicate
	card  cardinality
}

// FaceQuery is the concrete face selector: a conjunction of face predicates
// plus an optional cardinality assertion. Build one with Faces; it is a
// FaceSelector, and — sealed into Selector — what a Recipe Step stores.
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

// The queries seal into Selector (core §6.2), which is what a Recipe Step
// stores.
func (q *EdgeQuery) selector() {}
func (q *FaceQuery) selector() {}

// EdgePredicate is one clause of an EdgeQuery (core §6.2). Predicates come
// only from the package constructors — Convex, Concave, ParallelTo,
// LongerThan, CreatedBy, Circular — and compose by conjunction; the zero
// value names no predicate and refuses to encode.
type EdgePredicate struct {
	kind   string
	dir    r3.Vec
	length units.Value
	ref    FeatureRef
}

// FacePredicate is one clause of a FaceQuery (core §6.2). Predicates come
// only from the package constructors — Planar, Cylindrical, NormalTo,
// Facing, FaceCreatedBy — and compose by conjunction; the zero value names no
// predicate and refuses to encode.
type FacePredicate struct {
	kind string
	dir  r3.Vec
	ref  FeatureRef
}

// The wire vocabulary of the two predicate tiers. Like the two extent tiers,
// the names are distinct even where the meaning is parallel ("created_by" /
// "face_created_by"), so a tagged object reads unambiguously on its own.
const (
	predKindConvex        = "convex"
	predKindConcave       = "concave"
	predKindParallelTo    = "parallel_to"
	predKindLongerThan    = "longer_than"
	predKindCreatedBy     = "created_by"
	predKindCircular      = "circular"
	predKindPlanar        = "planar"
	predKindCylindrical   = "cylindrical"
	predKindNormalTo      = "normal_to"
	predKindFacing        = "facing"
	predKindFaceCreatedBy = "face_created_by"
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

// LongerThan matches edges strictly longer than l. The quantity is recorded
// exactly as given; a non-length or negative l is rejected at resolve
// (ErrUnitKind / ErrNegativeMagnitude), not here.
func LongerThan(l units.Value) EdgePredicate {
	return EdgePredicate{kind: predKindLongerThan, length: l}
}

// CreatedBy matches edges by provenance: edges created by the feature role f
// names. Provenance is structural, so it survives re-evaluation
// (docs/evaluator-design.md §3). A negative step or empty role is rejected at
// resolve as ErrDegenerate.
func CreatedBy(f FeatureRef) EdgePredicate {
	return EdgePredicate{kind: predKindCreatedBy, ref: f}
}

// Circular matches edges whose curve is a full circle or a circular arc.
func Circular() EdgePredicate { return EdgePredicate{kind: predKindCircular} }

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
// (docs/evaluator-design.md §3). A negative step or empty role is rejected at
// resolve as ErrDegenerate.
func FaceCreatedBy(f FeatureRef) FacePredicate {
	return FacePredicate{kind: predKindFaceCreatedBy, ref: f}
}

// CapStart names the start-cap role of b's own producing step, so
// FaceCreatedBy(CapStart(b)) selects that cap without a "capStart" string
// literal a typo could break. It reads b.Origin().Step (§6) and pairs it with
// the fixed role. The role exists only where the step mints it — an extrude, a
// partial revolve, a shell that built a tube, or a Placed/PlacedCopy/Duplicate
// of one; on any other body the ref is still well-formed and simply matches
// nothing (an ordinary ErrNoMatch at resolve). To name a cap a boolean's
// operand contributed, use FaceCreatedBy(CapStart(originalBody)) against the
// upstream body, never CapStart of the boolean result.
func CapStart(b *Body) FeatureRef {
	return FeatureRef{Step: b.Origin().Step, Role: roleCapStart}
}

// CapEnd names the end-cap role of b's own producing step — the sibling of
// CapStart. A body whose step mints no end cap (a cup, a full revolution) still
// gets a well-formed ref that matches nothing.
func CapEnd(b *Body) FeatureRef {
	return FeatureRef{Step: b.Origin().Step, Role: roleCapEnd}
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
	if ref.Step < 0 {
		return fmt.Errorf(`%w: a %s predicate cannot reference negative step %d`, ErrDegenerate, what, ref.Step)
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
	case predKindConvex, predKindConcave, predKindCircular:
		return nil
	case predKindCreatedBy:
		return validatePredicateRef(p.ref, "created-by")
	case predKindParallelTo:
		return validateDirection(p.dir, "parallel-to")
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
//   - longer_than compares Edge.Length() strictly against the recorded
//     quantity;
//   - created_by matches provenance through the edge's adjacent faces: an
//     edge is created by the role that created a face it bounds, so it
//     matches when ANY adjacent face's Origins() carries the ref;
//   - circular matches an edge whose curve is a full circle or a circular
//     arc.
func (p EdgePredicate) matches(e *Edge) bool {
	switch p.kind {
	case predKindConvex:
		return e.convex
	case predKindConcave:
		return !e.convex
	case predKindParallelTo:
		if _, ok := e.curve.(Line3); !ok {
			return false
		}
		return parallelDirs(e.end.position.Sub(e.start.position), p.dir)
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

// Selector is a closed variant set decad owns, so decad ships its codec
// (core §6.2): tagged objects, dispatch on the tag, no fallback. The variants
// seal in with pointer receivers, so — unlike the value-receiver sets — there
// is no value form to normalize to; instead the codec and the clone helpers
// guarantee that no caller-owned pointer survives into a recorded step and
// no recorded pointer escapes through Recipe().

const (
	selKindEdges = "edges"
	selKindFaces = "faces"
)

// errNilSelector rejects a nil variant pointer: it names no query to record.
// It wraps ErrDegenerate so a typed nil pointer is branchable exactly like
// any other degenerate input.
var errNilSelector = fmt.Errorf(`%w: nil selector`, ErrDegenerate)

// jsonQuery is a query's wire shape: the kind tag, the predicate list as
// tagged objects, and at most one cardinality assertion.
type jsonQuery struct {
	Kind    string            `json:"kind"`
	Preds   []json.RawMessage `json:"preds"`
	Exactly *int              `json:"exactly,omitempty"`
	AtLeast *int              `json:"at_least,omitempty"`
}

// marshalSelector encodes one query as its tagged object.
func marshalSelector(sel Selector) ([]byte, error) {
	switch q := sel.(type) {
	case *EdgeQuery:
		if q == nil {
			return nil, errNilSelector
		}
		preds := make([]json.RawMessage, 0, len(q.preds))
		for _, p := range q.preds {
			b, err := marshalEdgePredicate(p)
			if err != nil {
				return nil, err
			}
			preds = append(preds, b)
		}
		return marshalQuery(selKindEdges, preds, q.card)
	case *FaceQuery:
		if q == nil {
			return nil, errNilSelector
		}
		preds := make([]json.RawMessage, 0, len(q.preds))
		for _, p := range q.preds {
			b, err := marshalFacePredicate(p)
			if err != nil {
				return nil, err
			}
			preds = append(preds, b)
		}
		return marshalQuery(selKindFaces, preds, q.card)
	default:
		return nil, fmt.Errorf(`decad: unencodable selector type %T`, sel)
	}
}

// marshalQuery assembles the wire shape shared by the two query kinds.
func marshalQuery(kind string, preds []json.RawMessage, card cardinality) ([]byte, error) {
	out := jsonQuery{Kind: kind, Preds: preds}
	if card.kind != cardNone && card.n <= 0 {
		return nil, fmt.Errorf(`%w: a cardinality assertion needs a positive count, got %d`, ErrDegenerate, card.n)
	}
	switch card.kind {
	case cardNone:
		// no assertion recorded
	case cardExactly:
		n := card.n
		out.Exactly = &n
	case cardAtLeast:
		n := card.n
		out.AtLeast = &n
	default:
		return nil, fmt.Errorf(`decad: unencodable cardinality kind %d`, int(card.kind))
	}
	return json.Marshal(out)
}

// unmarshalSelector dispatches on the kind tag; an unknown or missing tag is
// an error — the set is closed. The wire struct uses pointer fields, so an
// absent predicate list is malformed, never silently a match-all query.
func unmarshalSelector(data []byte) (Selector, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return nil, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode selector tag: %w`, err))
	}
	switch probe.Kind {
	case selKindEdges, selKindFaces:
	case "":
		return nil, prependCodecPath(fmt.Errorf(`decad: selector is missing its kind tag`), "kind")
	default:
		return nil, prependCodecPath(fmt.Errorf(`decad: unknown selector kind %q`, probe.Kind), "kind")
	}
	var raw struct {
		Preds   *[]json.RawMessage `json:"preds"`
		Exactly *int               `json:"exactly"`
		AtLeast *int               `json:"at_least"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return nil, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode %s query: %w`, probe.Kind, err))
	}
	if raw.Preds == nil {
		return nil, prependCodecPath(fmt.Errorf(`decad: a %s query requires preds`, probe.Kind), "preds")
	}
	if raw.Exactly != nil && raw.AtLeast != nil {
		return nil, prependCodecPath(fmt.Errorf(`decad: a %s query carries at most one cardinality assertion`, probe.Kind), "exactly")
	}
	var card cardinality
	if raw.Exactly != nil {
		if *raw.Exactly <= 0 {
			return nil, prependCodecPath(fmt.Errorf(`%w: a %s query's exactly assertion needs a positive count, got %d`, ErrDegenerate, probe.Kind, *raw.Exactly), "exactly")
		}
		card = cardinality{kind: cardExactly, n: *raw.Exactly}
	}
	if raw.AtLeast != nil {
		if *raw.AtLeast <= 0 {
			return nil, prependCodecPath(fmt.Errorf(`%w: a %s query's at_least assertion needs a positive count, got %d`, ErrDegenerate, probe.Kind, *raw.AtLeast), "at_least")
		}
		card = cardinality{kind: cardAtLeast, n: *raw.AtLeast}
	}
	if probe.Kind == selKindEdges {
		q := &EdgeQuery{card: card}
		if len(*raw.Preds) > 0 {
			q.preds = make([]EdgePredicate, 0, len(*raw.Preds))
			for i, b := range *raw.Preds {
				p, err := unmarshalEdgePredicate(b)
				if err != nil {
					return nil, prependCodecPath(err, fmt.Sprintf(`preds[%d]`, i))
				}
				q.preds = append(q.preds, p)
			}
		}
		return q, nil
	}
	q := &FaceQuery{card: card}
	if len(*raw.Preds) > 0 {
		q.preds = make([]FacePredicate, 0, len(*raw.Preds))
		for i, b := range *raw.Preds {
			p, err := unmarshalFacePredicate(b)
			if err != nil {
				return nil, prependCodecPath(err, fmt.Sprintf(`preds[%d]`, i))
			}
			q.preds = append(q.preds, p)
		}
	}
	return q, nil
}

// marshalEdgePredicate encodes one clause as its tagged object. A zero-value
// predicate has no kind and refuses to encode — it came from nothing the
// constructors return.
func marshalEdgePredicate(p EdgePredicate) ([]byte, error) {
	switch p.kind {
	case predKindConvex, predKindConcave, predKindCircular:
		return marshalTagged(p.kind, struct{}{})
	case predKindParallelTo:
		return marshalTagged(p.kind, struct {
			Dir r3.Vec `json:"dir"`
		}{Dir: p.dir})
	case predKindLongerThan:
		return marshalTagged(p.kind, struct {
			L units.Value `json:"l"`
		}{L: p.length})
	case predKindCreatedBy:
		return marshalTagged(p.kind, struct {
			Ref FeatureRef `json:"ref"`
		}{Ref: p.ref})
	case "":
		return nil, fmt.Errorf(`decad: edge predicate is missing its kind; use the package constructors`)
	default:
		return nil, fmt.Errorf(`decad: unencodable edge predicate kind %q`, p.kind)
	}
}

// marshalFacePredicate encodes one clause as its tagged object. A zero-value
// predicate has no kind and refuses to encode — it came from nothing the
// constructors return.
func marshalFacePredicate(p FacePredicate) ([]byte, error) {
	switch p.kind {
	case predKindPlanar, predKindCylindrical:
		return marshalTagged(p.kind, struct{}{})
	case predKindNormalTo, predKindFacing:
		return marshalTagged(p.kind, struct {
			Dir r3.Vec `json:"dir"`
		}{Dir: p.dir})
	case predKindFaceCreatedBy:
		return marshalTagged(p.kind, struct {
			Ref FeatureRef `json:"ref"`
		}{Ref: p.ref})
	case "":
		return nil, fmt.Errorf(`decad: face predicate is missing its kind; use the package constructors`)
	default:
		return nil, fmt.Errorf(`decad: unencodable face predicate kind %q`, p.kind)
	}
}

// unmarshalEdgePredicate dispatches one clause on its kind tag. The payload
// wire structs use pointer fields, so an absent payload is malformed, never
// silently a zero direction, length or provenance.
func unmarshalEdgePredicate(data []byte) (EdgePredicate, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return EdgePredicate{}, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode edge predicate tag: %w`, err))
	}
	switch probe.Kind {
	case predKindConvex, predKindConcave, predKindCircular:
		return EdgePredicate{kind: probe.Kind}, nil
	case predKindParallelTo:
		var raw struct {
			Dir *r3.Vec `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return EdgePredicate{}, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode parallel-to predicate: %w`, err))
		}
		if raw.Dir == nil {
			return EdgePredicate{}, prependCodecPath(fmt.Errorf(`decad: a parallel-to predicate requires dir`), "dir")
		}
		return EdgePredicate{kind: probe.Kind, dir: *raw.Dir}, nil
	case predKindLongerThan:
		var raw struct {
			L *units.Value `json:"l"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return EdgePredicate{}, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode longer-than predicate: %w`, err))
		}
		if raw.L == nil {
			return EdgePredicate{}, prependCodecPath(fmt.Errorf(`decad: a longer-than predicate requires l`), "l")
		}
		return EdgePredicate{kind: probe.Kind, length: *raw.L}, nil
	case predKindCreatedBy:
		ref, err := unmarshalPredicateRef(data, "created-by")
		if err != nil {
			return EdgePredicate{}, err
		}
		return EdgePredicate{kind: probe.Kind, ref: ref}, nil
	case "":
		return EdgePredicate{}, prependCodecPath(fmt.Errorf(`decad: edge predicate is missing its kind tag`), "kind")
	default:
		return EdgePredicate{}, prependCodecPath(fmt.Errorf(`decad: unknown edge predicate kind %q`, probe.Kind), "kind")
	}
}

// unmarshalFacePredicate dispatches one clause on its kind tag. The payload
// wire structs use pointer fields, so an absent payload is malformed, never
// silently a zero direction or provenance.
func unmarshalFacePredicate(data []byte) (FacePredicate, error) {
	var probe struct {
		Kind string `json:"kind"`
	}
	if err := json.Unmarshal(data, &probe); err != nil {
		return FacePredicate{}, codecJSONErrorAt(data, &probe, fmt.Errorf(`decad: failed to decode face predicate tag: %w`, err))
	}
	switch probe.Kind {
	case predKindPlanar, predKindCylindrical:
		return FacePredicate{kind: probe.Kind}, nil
	case predKindNormalTo, predKindFacing:
		var raw struct {
			Dir *r3.Vec `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return FacePredicate{}, codecJSONErrorAt(data, &raw,
				fmt.Errorf(`decad: failed to decode %s predicate: %w`, facePredicateDisplayName(probe.Kind), err))
		}
		if raw.Dir == nil {
			return FacePredicate{}, prependCodecPath(fmt.Errorf(`decad: a %s predicate requires dir`, facePredicateDisplayName(probe.Kind)), "dir")
		}
		return FacePredicate{kind: probe.Kind, dir: *raw.Dir}, nil
	case predKindFaceCreatedBy:
		ref, err := unmarshalPredicateRef(data, "face-created-by")
		if err != nil {
			return FacePredicate{}, err
		}
		return FacePredicate{kind: probe.Kind, ref: ref}, nil
	case "":
		return FacePredicate{}, prependCodecPath(fmt.Errorf(`decad: face predicate is missing its kind tag`), "kind")
	default:
		return FacePredicate{}, prependCodecPath(fmt.Errorf(`decad: unknown face predicate kind %q`, probe.Kind), "kind")
	}
}

// facePredicateDisplayName maps a direction-bearing face-predicate kind tag to
// its established human-readable name for error messages. The wire token and
// the display name differ deliberately (normal_to -> normal-to), so a missing
// dir reports the name a caller sees in the API, not the raw tag.
func facePredicateDisplayName(kind string) string {
	switch kind {
	case predKindFacing:
		return "facing"
	default:
		return "normal-to"
	}
}

// unmarshalPredicateRef decodes a provenance predicate's payload with pointer
// fields: an absent ref, step or role is malformed, never silently step 0 or
// an empty role.
func unmarshalPredicateRef(data []byte, what string) (FeatureRef, error) {
	var raw struct {
		Ref *struct {
			Step *json.RawMessage `json:"step"`
			Role *string          `json:"role"`
		} `json:"ref"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return FeatureRef{}, codecJSONErrorAt(data, &raw, fmt.Errorf(`decad: failed to decode %s predicate: %w`, what, err))
	}
	if raw.Ref == nil {
		return FeatureRef{}, prependCodecPath(fmt.Errorf(`decad: a %s predicate requires ref with step and role`, what), "ref")
	}
	if raw.Ref.Step == nil {
		return FeatureRef{}, prependCodecPath(fmt.Errorf(`decad: a %s predicate requires ref with step and role`, what), "ref.step")
	}
	if raw.Ref.Role == nil {
		return FeatureRef{}, prependCodecPath(fmt.Errorf(`decad: a %s predicate requires ref with step and role`, what), "ref.role")
	}
	stepToken := bytes.TrimSpace(*raw.Ref.Step)
	if len(stepToken) > 0 && stepToken[0] == '-' {
		if negativeJSONNumberIsNonzero(stepToken) {
			return FeatureRef{}, fmt.Errorf(`%w: a %s predicate cannot reference negative step %s`, ErrDegenerate, what, stepToken)
		}
		stepToken = []byte("0")
	}
	var step StepRef
	if err := json.Unmarshal(stepToken, &step); err != nil {
		return FeatureRef{}, fmt.Errorf(`decad: failed to decode %s predicate: %w`, what, err)
	}
	ref := FeatureRef{Step: step, Role: *raw.Ref.Role}
	if err := validatePredicateRef(ref, what); err != nil {
		return FeatureRef{}, err
	}
	return ref, nil
}

// negativeJSONNumberIsNonzero reports whether a negative JSON number's
// significand contains a nonzero digit. The exponent never changes zero, so
// this classifies very large exponents without allocating a large number.
func negativeJSONNumberIsNonzero(token []byte) bool {
	significand := token[1:]
	if i := bytes.IndexAny(significand, "eE"); i >= 0 {
		significand = significand[:i]
	}
	return bytes.ContainsAny(significand, "123456789")
}

// cloneSelectors deep-copies a step's recorded queries: the selector variants
// seal in with pointer receivers, so keeping Recipe a value (core §6.2) means
// a fresh query per clone — no caller-owned pointer survives into a recorded
// step, and no recorded pointer escapes through Recipe().
func cloneSelectors(sels []Selector) []Selector {
	if sels == nil {
		return nil
	}
	out := make([]Selector, len(sels))
	for i, sel := range sels {
		out[i] = cloneSelector(sel)
	}
	return out
}

// cloneSelector deep-copies one query. The predicates hold only value fields,
// so cloning the slice is a deep copy. A malformed nil pointer stays as-is —
// the codec rejects it at its own gate.
func cloneSelector(sel Selector) Selector {
	switch q := sel.(type) {
	case *EdgeQuery:
		if q == nil {
			return sel
		}
		return &EdgeQuery{preds: slices.Clone(q.preds), card: q.card}
	case *FaceQuery:
		if q == nil {
			return sel
		}
		return &FaceQuery{preds: slices.Clone(q.preds), card: q.card}
	default:
		return sel
	}
}
