package decad

import (
	"errors"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/edgewedge"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// selector seals the two query families inside this package.
type selector interface{ selector() }

const (
	selKindEdges = "edges"
	selKindFaces = "faces"
)

// This file is the selector vocabulary of agent-docs/api-design.md §9: intent, not
// identity. A feature is given a query, never a pointer — handles do not
// survive an edit and index order is not stable, so an edge or face is named
// by geometric predicate and provenance instead. Features accept the
// interfaces (EdgeSelector/FaceSelector); the constructors return the
// concrete query types that implement them and carry the cardinality
// assertions. Both interfaces embed a private root so callers cannot add
// selector variants that features do not understand.
//
// Resolution is a filter pipeline over the body's live topology
// (agent-docs/evaluator-design.md §7): gather (Body.Edges()/Faces()), apply each
// predicate as a pure function of the analytic data, keep an entity when
// every predicate of at least one branch holds, then enforce the cardinality
// assertion on the kept set. Matching is decided on what an entity IS — a
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

// EdgeQuery is the concrete edge selector: a union of branches, each a
// conjunction of edge predicates, plus an optional cardinality assertion over
// the union. Build one with Edges and add branches with Or.
type EdgeQuery struct {
	// branches holds the conjunctions in query order: branches[0] is the
	// clause list Edges took, and each Or appends one more. Only a zero-value
	// query holds none, and it reads as Edges(): one empty branch.
	branches [][]EdgePredicate
	card     selectorquery.Cardinality
}

// FaceQuery is the concrete face selector: a union of branches, each a
// conjunction of face predicates, plus an optional cardinality assertion over
// the union. Build one with Faces and add branches with Or.
type FaceQuery struct {
	// branches holds the conjunctions in query order, as EdgeQuery's does.
	branches [][]FacePredicate
	card     selectorquery.Cardinality
}

// Edges returns a query matching every edge that satisfies all of preds; no
// predicates matches every edge. A query that matches nothing at resolve is
// an error, loudly — ErrNoMatch, or ErrCardinality when asserted (core §9).
func Edges(preds ...EdgePredicate) *EdgeQuery {
	return &EdgeQuery{branches: [][]EdgePredicate{slices.Clone(preds)}}
}

// Faces returns a query matching every face that satisfies all of preds; no
// predicates matches every face. A query that matches nothing at resolve is
// an error, loudly — ErrNoMatch, or ErrCardinality when asserted (core §9).
func Faces(preds ...FacePredicate) *FaceQuery {
	return &FaceQuery{branches: [][]FacePredicate{slices.Clone(preds)}}
}

// Or adds a union branch: the query then also matches every edge that
// satisfies all of preds. An edge that several branches match is selected
// once, in Body.Edges() order, and counts once toward Exactly or AtLeast,
// which assert the size of the whole union whether they are called before or
// after Or. An Or with no predicates matches every edge, as Edges() does. The
// predicates are copied, so a later change to the caller's slice does not
// reach the query. Or returns the receiver for chaining.
func (q *EdgeQuery) Or(preds ...EdgePredicate) *EdgeQuery {
	if len(q.branches) == 0 {
		q.branches = [][]EdgePredicate{nil}
	}
	q.branches = append(q.branches, slices.Clone(preds))
	return q
}

// Or adds a union branch: the query then also matches every face that
// satisfies all of preds. A face that several branches match is selected
// once, in Body.Faces() order, and counts once toward Exactly or AtLeast,
// which assert the size of the whole union whether they are called before or
// after Or. An Or with no predicates matches every face, as Faces() does. The
// predicates are copied, so a later change to the caller's slice does not
// reach the query. Or returns the receiver for chaining.
//
// Faces(FaceCreatedBy(CapStart(b))).Or(FaceCreatedBy(CapEnd(b))).Exactly(2)
// names both angular caps of a partial-turn revolve b, which no single
// conjunction of geometric predicates names when the caps are not coplanar.
func (q *FaceQuery) Or(preds ...FacePredicate) *FaceQuery {
	if len(q.branches) == 0 {
		q.branches = [][]FacePredicate{nil}
	}
	q.branches = append(q.branches, slices.Clone(preds))
	return q
}

// SelectEdges resolves the query against the body's topology: gather
// (Body.Edges(), whose order the result keeps), keep each edge that satisfies
// every predicate of at least one branch, then enforce the cardinality
// assertion on that set (agent-docs/evaluator-design.md §7). A
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
	for _, branch := range q.branches {
		for _, p := range branch {
			if err := p.validate(); err != nil {
				return nil, err
			}
		}
	}
	edges := body.Edges()
	matched := make([]*Edge, 0, len(edges))
	for _, e := range edges {
		if !selectorquery.MatchAny(e, q.branches, EdgePredicate.matches) {
			continue
		}
		matched = append(matched, e)
	}
	if err := q.card.Enforce(len(matched), "edges"); err != nil {
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
		return q.selectionError(body, matched, q.card.Expected(), ErrNoMatch)
	case errors.Is(err, ErrCardinality):
		return q.selectionError(body, matched, q.card.Expected(), ErrCardinality)
	default:
		return err
	}
}

// SelectFaces resolves the query against the body's topology: gather
// (Body.Faces(), whose order the result keeps), keep each face that satisfies
// every predicate of at least one branch, then enforce the cardinality
// assertion on that set (agent-docs/evaluator-design.md §7). A
// nil body has no topology to select from and is ErrDegenerate; zero matches
// is ErrCardinality when asserted, else ErrNoMatch (core §12 precedence).
func (q *FaceQuery) SelectFaces(body *Body) ([]*Face, error) {
	if q == nil {
		return nil, errNilSelector
	}
	if body == nil {
		return nil, fmt.Errorf(`%w: a nil body has no faces to select`, ErrDegenerate)
	}
	for _, branch := range q.branches {
		for _, p := range branch {
			if err := p.validate(); err != nil {
				return nil, err
			}
		}
	}
	faces := body.Faces()
	matched := make([]*Face, 0, len(faces))
	for _, f := range faces {
		if !selectorquery.MatchAny(f, q.branches, FacePredicate.matches) {
			continue
		}
		matched = append(matched, f)
	}
	if err := q.card.Enforce(len(matched), "faces"); err != nil {
		return nil, q.enrich(body, len(matched), err)
	}
	return matched, nil
}

// enrich is the face analog of EdgeQuery.enrich.
func (q *FaceQuery) enrich(body *Body, matched int, err error) error {
	switch {
	case errors.Is(err, ErrNoMatch):
		return q.selectionError(body, matched, q.card.Expected(), ErrNoMatch)
	case errors.Is(err, ErrCardinality):
		return q.selectionError(body, matched, q.card.Expected(), ErrCardinality)
	default:
		return err
	}
}

// Exactly asserts the query resolves to exactly n matches; anything else —
// zero included — is ErrCardinality at resolve. It replaces any earlier
// cardinality assertion and returns the receiver for chaining.
func (q *EdgeQuery) Exactly(n int) *EdgeQuery {
	q.card = selectorquery.Cardinality{Kind: selectorquery.CardExactly, N: n}
	return q
}

// AtLeast asserts the query resolves to at least n matches; fewer is
// ErrCardinality at resolve. It replaces any earlier cardinality assertion
// and returns the receiver for chaining.
func (q *EdgeQuery) AtLeast(n int) *EdgeQuery {
	q.card = selectorquery.Cardinality{Kind: selectorquery.CardAtLeast, N: n}
	return q
}

// Exactly asserts the query resolves to exactly n matches; anything else —
// zero included — is ErrCardinality at resolve. It replaces any earlier
// cardinality assertion and returns the receiver for chaining.
func (q *FaceQuery) Exactly(n int) *FaceQuery {
	q.card = selectorquery.Cardinality{Kind: selectorquery.CardExactly, N: n}
	return q
}

// AtLeast asserts the query resolves to at least n matches; fewer is
// ErrCardinality at resolve. It replaces any earlier cardinality assertion
// and returns the receiver for chaining.
func (q *FaceQuery) AtLeast(n int) *FaceQuery {
	q.card = selectorquery.Cardinality{Kind: selectorquery.CardAtLeast, N: n}
	return q
}

// The queries seal into the private selector root.
func (q *EdgeQuery) selector() {}
func (q *FaceQuery) selector() {}

// EdgePredicate is one clause of an EdgeQuery. Predicates come
// only from the package constructors — Convex, Concave, Reflex, Salient,
// ParallelTo, EndpointAt, LongerThan, CreatedBy, Circular, Free — and compose by
// conjunction within a branch (EdgeQuery.Or adds a branch); the zero
// value names no predicate and is rejected at resolve.
type EdgePredicate struct {
	kind   string
	dir    r3.Vec
	point  r3.Vec
	length units.Value
	ref    provenance.Ref
}

// FacePredicate is one clause of a FaceQuery. Predicates come
// only from the package constructors — Planar, Cylindrical, NormalTo,
// Facing, FaceCreatedBy — and compose by conjunction within a branch
// (FaceQuery.Or adds a branch); the zero value names no predicate and is
// rejected at resolve.
type FacePredicate struct {
	kind string
	dir  r3.Vec
	ref  provenance.Ref
}

// Convex matches edges Edge.IsConvex reports convex — the walked-boundary
// convention, not the 3D material angle across the edge. Read Edge.IsConvex
// before selecting on it: a hole's rim edges are CONCAVE, so this predicate
// never picks them.
func Convex() EdgePredicate { return EdgePredicate{kind: selectorquery.ConvexKind} }

// Concave matches edges Edge.IsConvex reports concave — the walked-boundary
// convention, not the 3D material angle across the edge. This is the predicate
// that picks a hole's rim edges, and the rim of a concave round bitten out of
// the outer boundary.
func Concave() EdgePredicate { return EdgePredicate{kind: selectorquery.ConcaveKind} }

// Reflex matches edges Edge.Wedge reads WedgeReflex: the material angle across
// the edge is over a half turn, as at a boss root or an inside corner. It is
// the predicate that picks the root of a boss on a Union result, which
// Concave does not. An edge whose wedge is WedgeUndecided matches neither
// Reflex nor Salient.
func Reflex() EdgePredicate { return EdgePredicate{kind: selectorquery.ReflexKind} }

// Salient matches edges Edge.Wedge reads WedgeSalient: the material angle
// across the edge is under a half turn, as at a box's outer edge or a hole's
// rim. An edge whose wedge is WedgeUndecided matches neither Salient nor
// Reflex.
func Salient() EdgePredicate { return EdgePredicate{kind: selectorquery.SalientKind} }

// ParallelTo matches edges whose direction is parallel to v. The vector is
// recorded exactly as given — a dimensionless direction (core §5.2); a
// degenerate (zero or non-finite) direction is rejected at resolve, not here.
func ParallelTo(v r3.Vec) EdgePredicate {
	return EdgePredicate{kind: selectorquery.ParallelToKind, dir: v}
}

// EndpointAt matches an edge whose stored start or end position equals p
// component-wise. It uses no tolerance; a non-finite p fails at resolve.
func EndpointAt(p r3.Vec) EdgePredicate {
	return EdgePredicate{kind: selectorquery.EndpointAtKind, point: p}
}

// LongerThan matches edges strictly longer than l. The quantity is recorded
// exactly as given; a non-length or negative l is rejected at resolve
// (ErrUnitKind / ErrNegativeMagnitude), not here.
func LongerThan(l units.Value) EdgePredicate {
	return EdgePredicate{kind: selectorquery.LongerThanKind, length: l}
}

// CreatedBy matches edges by provenance: edges created by the feature role f
// names. Provenance is structural, so it survives body rebuilds
// (agent-docs/evaluator-design.md §3). A negative producer identity or empty role is rejected at
// resolve as ErrDegenerate.
func CreatedBy(f FeatureRef) EdgePredicate {
	return EdgePredicate{kind: selectorquery.CreatedByKind, ref: refOf(f)}
}

// OuterLoopOf matches edges on the outer boundary loop of a face carrying f.
// On a holed cap, it excludes every inner-loop rim edge, including a bore.
func OuterLoopOf(f FeatureRef) EdgePredicate {
	return EdgePredicate{kind: selectorquery.OuterLoopOfKind, ref: refOf(f)}
}

// Circular matches edges whose curve is a full circle or a circular arc.
func Circular() EdgePredicate { return EdgePredicate{kind: selectorquery.CircularKind} }

// Free matches edges Edge.IsFree reports free — exactly one adjacent face, a
// sheet's boundary (agent-docs/surface-design.md §2.2, §3.1). It composes with
// every other clause: Edges(Free(), Circular()) picks the circular free
// edges, Edges(Free()).Exactly(8) asserts a surface extrude's rim count. On a
// body with no free edge — every solid, and a closed sheet — it matches
// nothing, an ordinary ErrNoMatch at resolve or ErrCardinality under an
// assertion, never an error in itself.
func Free() EdgePredicate { return EdgePredicate{kind: selectorquery.FreeKind} }

// Planar matches faces whose surface is a plane.
func Planar() FacePredicate { return FacePredicate{kind: selectorquery.PlanarKind} }

// Cylindrical matches faces whose surface is a cylinder. On a Faceted body
// the analytic identity is gone, so it matches nothing there — matching is
// decided on what the face IS (agent-docs/evaluator-design.md §7).
func Cylindrical() FacePredicate { return FacePredicate{kind: selectorquery.CylindricalKind} }

// NormalTo matches planar faces whose normal is parallel to v. The vector is
// recorded exactly as given — a dimensionless direction (core §5.2); a
// degenerate (zero or non-finite) direction is rejected at resolve, not here.
func NormalTo(v r3.Vec) FacePredicate {
	return FacePredicate{kind: selectorquery.NormalToKind, dir: v}
}

// Facing matches the planar face whose OUTWARD (material-leaving) normal points
// ALONG v — parallel to v AND the same sense, a positive projection. NormalTo
// matches on either sense, so a slab's two parallel caps both match NormalTo(z);
// Facing(z) picks only the top one. The vector is recorded exactly as given — a
// dimensionless direction (core §5.2); a degenerate (zero or non-finite)
// direction is rejected at resolve, not here.
func Facing(v r3.Vec) FacePredicate {
	return FacePredicate{kind: selectorquery.FacingKind, dir: v}
}

// FaceCreatedBy matches faces by provenance — the face analog of CreatedBy.
// A canonicalization merge unions the merged faces' roles and this matches
// on any of them, so provenance survives the merge
// (agent-docs/evaluator-design.md §3). A negative producer identity or empty role is rejected at
// resolve as ErrDegenerate.
func FaceCreatedBy(f FeatureRef) FacePredicate {
	return FacePredicate{kind: selectorquery.FaceCreatedByKind, ref: refOf(f)}
}

// CapStart names the start-cap role of b's own producing feature, so
// FaceCreatedBy(CapStart(b)) selects that cap without a "capStart" string
// literal a typo could break. It reads b.Origin().producer (§6) and pairs it with
// the fixed role. The role exists only where the feature mints it — an extrude, a
// partial revolve, a shell that built a tube, an authenticated axial
// Loft/cylinder join or its centered bore Cut, or a Transform/Copy
// of one; on any other body the ref is still well-formed and simply matches
// nothing (an ordinary ErrNoMatch at resolve). An ordinary mesh boolean
// preserves only its operands' cap origins, so name those with
// FaceCreatedBy(CapStart(originalBody)).
func CapStart(b *Body) FeatureRef {
	return featureRefOf(provenance.Ref{Producer: b.n.Origin.Producer, Role: provenance.RoleCapStart})
}

// CapEnd names the end-cap role of b's own producing feature — the sibling of
// CapStart. A body whose feature mints no end cap (a cup, a full revolution) still
// gets a well-formed ref that matches nothing.
func CapEnd(b *Body) FeatureRef {
	return featureRefOf(provenance.Ref{Producer: b.n.Origin.Producer, Role: provenance.RoleCapEnd})
}

// Walls matches every face a sweep made: the faces carrying a side(i, j) role
// of b's own producing feature. It is the sibling of CapStart and CapEnd, and
// it is what Draft takes as the complete wall set of an extrude
// (agent-docs/draft-design.md §10). On a body whose producer mints no side roles it
// matches nothing, an ordinary ErrNoMatch at resolve. b must not be nil.
func Walls(b *Body) FacePredicate {
	return FacePredicate{kind: selectorquery.WallsKind, ref: provenance.Ref{Producer: b.n.Origin.Producer, Role: selectorquery.RoleSidePrefix}}
}

// validatePredicateRef rejects provenance that cannot name a feature role.
func validatePredicateRef(ref provenance.Ref, what string) error {
	if ref.Producer < 0 {
		return fmt.Errorf(`%w: a %s predicate cannot reference negative producer %d`, ErrDegenerate, what, ref.Producer)
	}
	if ref.Role == "" {
		return fmt.Errorf(`%w: a %s predicate requires a non-empty provenance role`, ErrDegenerate, what)
	}
	return nil
}

func (p EdgePredicate) clause() selectorquery.EdgeClause[provenance.Ref] {
	return selectorquery.EdgeClause[provenance.Ref]{
		Kind: p.kind, Direction: p.dir, Point: p.point, Length: p.length, Ref: p.ref,
	}
}

func (p FacePredicate) clause() selectorquery.FaceClause[provenance.Ref] {
	return selectorquery.FaceClause[provenance.Ref]{Kind: p.kind, Direction: p.dir, Ref: p.ref}
}

func (p EdgePredicate) validate() error { return p.clause().Validate(validatePredicateRef) }
func (p FacePredicate) validate() error { return p.clause().Validate(validatePredicateRef) }

func (p EdgePredicate) matches(e *Edge) bool {
	return p.clause().Matches(edgewedge.View{Edge: edgeNode(e)})
}
func (p FacePredicate) matches(f *Face) bool {
	return p.clause().Matches(selectorquery.FaceNode{Face: faceNode(f)})
}

// errNilSelector rejects a nil query.
var errNilSelector = fmt.Errorf(`%w: nil selector`, ErrDegenerate)
