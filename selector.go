package decad

import (
	"encoding/json"
	"fmt"
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
// Resolution — the filter pipeline over a live body's topology
// (docs/evaluator-design.md §7) — lands with evaluator increment 2
// (docs/evaluator-design.md §11); until then SelectEdges/SelectFaces are
// staged explicitly via ErrUnsupported, never silently empty.

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

// errSelectorResolutionStaged is the staged-resolution rejection: the query
// records exactly, but no evaluator resolves it yet
// (docs/evaluator-design.md §11 increment 2). Explicit, never silently empty.
var errSelectorResolutionStaged = fmt.Errorf(`%w: selector resolution lands with evaluator increment 2`, ErrUnsupported)

// SelectEdges resolves the query against a live body's topology: gather,
// filter by each predicate, then enforce the cardinality assertion
// (docs/evaluator-design.md §7). Resolution is staged: it is ErrUnsupported
// until evaluator increment 2.
func (q *EdgeQuery) SelectEdges(*Body) ([]*Edge, error) {
	return nil, errSelectorResolutionStaged
}

// SelectFaces resolves the query against a live body's topology: gather,
// filter by each predicate, then enforce the cardinality assertion
// (docs/evaluator-design.md §7). Resolution is staged: it is ErrUnsupported
// until evaluator increment 2.
func (q *FaceQuery) SelectFaces(*Body) ([]*Face, error) {
	return nil, errSelectorResolutionStaged
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
// FaceCreatedBy — and compose by conjunction; the zero value names no
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
	predKindFaceCreatedBy = "face_created_by"
)

// Convex matches edges whose dihedral angle is convex.
func Convex() EdgePredicate { return EdgePredicate{kind: predKindConvex} }

// Concave matches edges whose dihedral angle is concave.
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
// (docs/evaluator-design.md §3).
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

// FaceCreatedBy matches faces by provenance — the face analog of CreatedBy.
// A canonicalization merge unions the merged faces' roles and this matches
// on any of them, so provenance survives the merge
// (docs/evaluator-design.md §3).
func FaceCreatedBy(f FeatureRef) FacePredicate {
	return FacePredicate{kind: predKindFaceCreatedBy, ref: f}
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
		return nil, fmt.Errorf(`decad: failed to decode selector tag: %w`, err)
	}
	switch probe.Kind {
	case selKindEdges, selKindFaces:
	case "":
		return nil, fmt.Errorf(`decad: selector is missing its kind tag`)
	default:
		return nil, fmt.Errorf(`decad: unknown selector kind %q`, probe.Kind)
	}
	var raw struct {
		Preds   *[]json.RawMessage `json:"preds"`
		Exactly *int               `json:"exactly"`
		AtLeast *int               `json:"at_least"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return nil, fmt.Errorf(`decad: failed to decode %s query: %w`, probe.Kind, err)
	}
	if raw.Preds == nil {
		return nil, fmt.Errorf(`decad: a %s query requires preds`, probe.Kind)
	}
	if raw.Exactly != nil && raw.AtLeast != nil {
		return nil, fmt.Errorf(`decad: a %s query carries at most one cardinality assertion`, probe.Kind)
	}
	var card cardinality
	if raw.Exactly != nil {
		card = cardinality{kind: cardExactly, n: *raw.Exactly}
	}
	if raw.AtLeast != nil {
		card = cardinality{kind: cardAtLeast, n: *raw.AtLeast}
	}
	if probe.Kind == selKindEdges {
		q := &EdgeQuery{card: card}
		if len(*raw.Preds) > 0 {
			q.preds = make([]EdgePredicate, 0, len(*raw.Preds))
			for _, b := range *raw.Preds {
				p, err := unmarshalEdgePredicate(b)
				if err != nil {
					return nil, err
				}
				q.preds = append(q.preds, p)
			}
		}
		return q, nil
	}
	q := &FaceQuery{card: card}
	if len(*raw.Preds) > 0 {
		q.preds = make([]FacePredicate, 0, len(*raw.Preds))
		for _, b := range *raw.Preds {
			p, err := unmarshalFacePredicate(b)
			if err != nil {
				return nil, err
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
	case predKindNormalTo:
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
		return EdgePredicate{}, fmt.Errorf(`decad: failed to decode edge predicate tag: %w`, err)
	}
	switch probe.Kind {
	case predKindConvex, predKindConcave, predKindCircular:
		return EdgePredicate{kind: probe.Kind}, nil
	case predKindParallelTo:
		var raw struct {
			Dir *r3.Vec `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return EdgePredicate{}, fmt.Errorf(`decad: failed to decode parallel-to predicate: %w`, err)
		}
		if raw.Dir == nil {
			return EdgePredicate{}, fmt.Errorf(`decad: a parallel-to predicate requires dir`)
		}
		return EdgePredicate{kind: probe.Kind, dir: *raw.Dir}, nil
	case predKindLongerThan:
		var raw struct {
			L *units.Value `json:"l"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return EdgePredicate{}, fmt.Errorf(`decad: failed to decode longer-than predicate: %w`, err)
		}
		if raw.L == nil {
			return EdgePredicate{}, fmt.Errorf(`decad: a longer-than predicate requires l`)
		}
		return EdgePredicate{kind: probe.Kind, length: *raw.L}, nil
	case predKindCreatedBy:
		ref, err := unmarshalPredicateRef(data, "created-by")
		if err != nil {
			return EdgePredicate{}, err
		}
		return EdgePredicate{kind: probe.Kind, ref: ref}, nil
	case "":
		return EdgePredicate{}, fmt.Errorf(`decad: edge predicate is missing its kind tag`)
	default:
		return EdgePredicate{}, fmt.Errorf(`decad: unknown edge predicate kind %q`, probe.Kind)
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
		return FacePredicate{}, fmt.Errorf(`decad: failed to decode face predicate tag: %w`, err)
	}
	switch probe.Kind {
	case predKindPlanar, predKindCylindrical:
		return FacePredicate{kind: probe.Kind}, nil
	case predKindNormalTo:
		var raw struct {
			Dir *r3.Vec `json:"dir"`
		}
		if err := json.Unmarshal(data, &raw); err != nil {
			return FacePredicate{}, fmt.Errorf(`decad: failed to decode normal-to predicate: %w`, err)
		}
		if raw.Dir == nil {
			return FacePredicate{}, fmt.Errorf(`decad: a normal-to predicate requires dir`)
		}
		return FacePredicate{kind: probe.Kind, dir: *raw.Dir}, nil
	case predKindFaceCreatedBy:
		ref, err := unmarshalPredicateRef(data, "face-created-by")
		if err != nil {
			return FacePredicate{}, err
		}
		return FacePredicate{kind: probe.Kind, ref: ref}, nil
	case "":
		return FacePredicate{}, fmt.Errorf(`decad: face predicate is missing its kind tag`)
	default:
		return FacePredicate{}, fmt.Errorf(`decad: unknown face predicate kind %q`, probe.Kind)
	}
}

// unmarshalPredicateRef decodes a provenance predicate's payload with pointer
// fields: an absent ref, step or role is malformed, never silently step 0 or
// an empty role.
func unmarshalPredicateRef(data []byte, what string) (FeatureRef, error) {
	var raw struct {
		Ref *struct {
			Step *StepRef `json:"step"`
			Role *string  `json:"role"`
		} `json:"ref"`
	}
	if err := json.Unmarshal(data, &raw); err != nil {
		return FeatureRef{}, fmt.Errorf(`decad: failed to decode %s predicate: %w`, what, err)
	}
	if raw.Ref == nil || raw.Ref.Step == nil || raw.Ref.Role == nil {
		return FeatureRef{}, fmt.Errorf(`decad: a %s predicate requires ref with step and role`, what)
	}
	return FeatureRef{Step: *raw.Ref.Step, Role: *raw.Ref.Role}, nil
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
