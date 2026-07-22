package decad

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/r3"
)

// This file is the diagnostic surface of a selector failure
// (docs/api-design.md §9): SelectionError and the stable query rendering the
// error, a verification Diagnostic and equality all reuse. A cardinality
// error that reported only a count would force an agent to reconstruct the
// query from source and probe it one clause at a time; SelectionError instead
// carries what the agent needs to repair the query directly — the stable
// rendering, the body it resolved against, the expected/actual counts, and the
// per-clause running match count that names the clause which emptied the set.
// It wraps ErrNoMatch or ErrCardinality (core §12), so errors.Is still
// branches.

// SelectorKind names which entity a selector picks — an edge or a face. Its
// String is the singular "edge" / "face" naming the entity, deliberately
// distinct from a query's plural "edges" / "faces" prefix.
type SelectorKind int

const (
	// EdgeSelectorKind is the kind of an EdgeQuery selection.
	EdgeSelectorKind SelectorKind = iota
	// FaceSelectorKind is the kind of a FaceQuery selection.
	FaceSelectorKind
)

// String reports the entity a selector kind picks: "edge" or "face".
func (k SelectorKind) String() string {
	switch k {
	case EdgeSelectorKind:
		return "edge"
	case FaceSelectorKind:
		return "face"
	default:
		return fmt.Sprintf("SelectorKind(%d)", int(k))
	}
}

// PredicateResidual is the running match count after one clause of a query's
// conjunction, evaluated cumulatively in query order: Remaining is the number
// of candidates still matching after this clause AND every clause before it.
// The clause whose Remaining reaches zero is the one that emptied the set.
type PredicateResidual struct {
	// Predicate is the clause's stable rendering — "convex",
	// "parallel_to(0,0,1)".
	Predicate string
	// Remaining is the candidate count still matching after this clause and
	// every clause before it. It starts at the body's whole edge or face
	// count and can only fall.
	Remaining int
}

// SelectionError is returned by SelectEdges / SelectFaces — and by the
// implicit exactly-one of a ToFace, ToFaceAngular or EdgeAxis — when a query
// resolves to a count its cardinality assertion refuses (core §9/§12). It
// wraps ErrNoMatch or ErrCardinality, so errors.Is still branches, and carries
// the diagnostic enrichment an agent needs to repair the query directly. The
// residuals are computed only on the failing path, so a resolving query pays
// nothing.
type SelectionError struct {
	// Kind is the entity the selector picks — edge or face.
	Kind SelectorKind
	// Query is the query's stable rendering — q.String().
	Query string
	// Body is the body the query resolved against; Body.Origin() names its
	// producing feature.
	Body *Body
	// Expected is the cardinality assertion rendered in prose — "exactly 4",
	// "at least 1", or "any" for none; "exactly 1" for an implicit
	// exactly-one.
	Expected string
	// Actual is the total match count.
	Actual int
	// Residuals is the running match count after each clause, in query order.
	Residuals []PredicateResidual
	// err is the wrapped sentinel — ErrNoMatch or ErrCardinality — Unwrap
	// returns for errors.Is.
	err error
}

// Error renders the human diagnostic. The wrapped sentinel's own text leads,
// so errors printed by string still read as the sentinel; the query, body,
// counts and the emptied clause follow.
func (e *SelectionError) Error() string {
	noun := "edges"
	if e.Kind == FaceSelectorKind {
		noun = "faces"
	}
	subject := e.Query
	if e.Body != nil {
		subject += " on body " + renderRef(e.Body.Origin())
	}
	msg := fmt.Sprintf("%s: %s matched %d %s, expected %s", e.err, subject, e.Actual, noun, e.Expected)
	if clause, ok := e.emptiedClause(); ok {
		msg += fmt.Sprintf("; the clause %s matched none", clause)
	}
	return msg
}

// Unwrap returns the wrapped sentinel — ErrNoMatch or ErrCardinality — so
// errors.Is branches on the selection outcome exactly as before the type
// existed.
func (e *SelectionError) Unwrap() error { return e.err }

// emptiedClause names the first clause whose running match count reached zero,
// if any — the clause that emptied the conjunction.
func (e *SelectionError) emptiedClause() (string, bool) {
	for _, r := range e.Residuals {
		if r.Remaining == 0 {
			return r.Predicate, true
		}
	}
	return "", false
}

// expectedExactlyOne is the Expected an implicit exactly-one (ToFace,
// ToFaceAngular, EdgeAxis) renders.
const expectedExactlyOne = "exactly 1"

// expected renders a cardinality assertion in prose for SelectionError.Expected:
// "exactly <n>", "at least <n>", or "any" for no assertion.
func (c cardinality) expected() string {
	switch c.kind {
	case cardExactly:
		return fmt.Sprintf("exactly %d", c.n)
	case cardAtLeast:
		return fmt.Sprintf("at least %d", c.n)
	default:
		return "any"
	}
}

// suffix renders a cardinality assertion as a query's trailing token: empty
// for no assertion, ".exactly(<n>)" or ".at_least(<n>)" — the codec's own
// keys.
func (c cardinality) suffix() string {
	switch c.kind {
	case cardExactly:
		return fmt.Sprintf(".exactly(%d)", c.n)
	case cardAtLeast:
		return fmt.Sprintf(".at_least(%d)", c.n)
	default:
		return ""
	}
}

// String renders the query canonically: edges(<pred>, <pred>, …)<cardinality>.
// The rendering is a deterministic function of the recorded content — equal
// recorded queries render identically, and a query and its decoded round-trip
// render identically — built from the codec's own tagged vocabulary. It is an
// identity for diagnostics and equality, not a parseable format.
func (q *EdgeQuery) String() string {
	if q == nil {
		return selKindEdges + "()"
	}
	preds := make([]string, len(q.preds))
	for i, p := range q.preds {
		preds[i] = p.render()
	}
	return renderQuery(selKindEdges, preds, q.card)
}

// String renders the query canonically: faces(<pred>, <pred>, …)<cardinality>,
// the face analog of EdgeQuery.String.
func (q *FaceQuery) String() string {
	if q == nil {
		return selKindFaces + "()"
	}
	preds := make([]string, len(q.preds))
	for i, p := range q.preds {
		preds[i] = p.render()
	}
	return renderQuery(selKindFaces, preds, q.card)
}

// renderQuery assembles the shared query shape: the plural kind token, the
// clauses in query order comma-separated, and the cardinality suffix. No
// clause renders the empty parentheses of a predicate-less query.
func renderQuery(kind string, preds []string, card cardinality) string {
	var b strings.Builder
	b.WriteString(kind)
	b.WriteByte('(')
	b.WriteString(strings.Join(preds, ", "))
	b.WriteByte(')')
	b.WriteString(card.suffix())
	return b.String()
}

// render renders one edge clause by its codec kind token and payload. It is
// total: a zero-value, kind-less predicate — which the constructors never
// produce, but a half-decoded query might carry — renders "<invalid>" rather
// than panicking.
func (p EdgePredicate) render() string {
	switch p.kind {
	case predKindConvex, predKindConcave, predKindCircular:
		return p.kind
	case predKindParallelTo:
		return p.kind + "(" + renderVec(p.dir) + ")"
	case predKindLongerThan:
		return p.kind + "(" + p.length.String() + ")"
	case predKindCreatedBy:
		return p.kind + "(" + renderRef(p.ref) + ")"
	default:
		return "<invalid>"
	}
}

// render renders one face clause by its codec kind token and payload, the face
// analog of EdgePredicate.render.
func (p FacePredicate) render() string {
	switch p.kind {
	case predKindPlanar, predKindCylindrical:
		return p.kind
	case predKindNormalTo:
		return p.kind + "(" + renderVec(p.dir) + ")"
	case predKindFaceCreatedBy:
		return p.kind + "(" + renderRef(p.ref) + ")"
	default:
		return "<invalid>"
	}
}

// renderVec renders a direction as <x>,<y>,<z> — comma-separated, no spaces,
// each coordinate as the shortest round-tripping float. A negative zero
// renders "0" (the normalization is load-bearing for the equal-queries-render-
// identically contract: -0.0 == 0.0, so two value-equal directions must render
// the same). Rendering is total: a non-finite component reads NaN / +Inf /
// -Inf as the formatter writes it.
func renderVec(v r3.Vec) string {
	return renderCoord(v.X) + "," + renderCoord(v.Y) + "," + renderCoord(v.Z)
}

// renderCoord renders one coordinate as its shortest round-tripping float,
// normalizing a negative zero to "0" before formatting.
func renderCoord(c float64) string {
	if c == 0 {
		// -0.0 == 0.0, so this catches math.Copysign(0, -1) too; FormatFloat
		// would otherwise write it "-0".
		return "0"
	}
	return strconv.FormatFloat(c, 'g', -1, 64)
}

// renderRef renders a FeatureRef as <step>:<role> — the StepRef in decimal and
// the role quoted, so a role that itself holds parentheses or commas stays
// unambiguous: 3:"capStart", 2:"side(0,1)".
func renderRef(f FeatureRef) string {
	return strconv.Itoa(int(f.Step)) + ":" + strconv.Quote(f.Role)
}

// selectionError builds the SelectionError an edge query's failing resolution
// reports, computing the per-clause residuals over the body's edges. Residuals
// are computed only here, on the failing path.
func (q *EdgeQuery) selectionError(body *Body, actual int, expected string, sentinel error) *SelectionError {
	return &SelectionError{
		Kind:      EdgeSelectorKind,
		Query:     q.String(),
		Body:      body,
		Expected:  expected,
		Actual:    actual,
		Residuals: q.residuals(body),
		err:       sentinel,
	}
}

// selectionError builds the SelectionError a face query's failing resolution
// reports, the face analog of EdgeQuery.selectionError.
func (q *FaceQuery) selectionError(body *Body, actual int, expected string, sentinel error) *SelectionError {
	return &SelectionError{
		Kind:      FaceSelectorKind,
		Query:     q.String(),
		Body:      body,
		Expected:  expected,
		Actual:    actual,
		Residuals: q.residuals(body),
		err:       sentinel,
	}
}

// residuals evaluates the predicate conjunction cumulatively over the body's
// edges, recording the running match count after each clause in query order.
// Predicates were validated before this runs, so p.matches is safe.
func (q *EdgeQuery) residuals(body *Body) []PredicateResidual {
	if len(q.preds) == 0 {
		return nil
	}
	cands := body.Edges()
	out := make([]PredicateResidual, 0, len(q.preds))
	for _, p := range q.preds {
		kept := make([]*Edge, 0, len(cands))
		for _, e := range cands {
			if p.matches(e) {
				kept = append(kept, e)
			}
		}
		cands = kept
		out = append(out, PredicateResidual{Predicate: p.render(), Remaining: len(cands)})
	}
	return out
}

// residuals is the face analog of EdgeQuery.residuals.
func (q *FaceQuery) residuals(body *Body) []PredicateResidual {
	if len(q.preds) == 0 {
		return nil
	}
	cands := body.Faces()
	out := make([]PredicateResidual, 0, len(q.preds))
	for _, p := range q.preds {
		kept := make([]*Face, 0, len(cands))
		for _, f := range cands {
			if p.matches(f) {
				kept = append(kept, f)
			}
		}
		cands = kept
		out = append(out, PredicateResidual{Predicate: p.render(), Remaining: len(cands)})
	}
	return out
}

// impliedOneFace builds the implicit exactly-one SelectionError a ToFace /
// ToFaceAngular stop reports when its selector resolves to a count that is not
// one (core §9). Expected is rewritten to "exactly 1" and Actual is the
// resolved count, preserving the resolution's Kind, Query, Body and Residuals.
// It takes the concrete *FaceQuery: the implicit-one callers gate the selector
// to the built-in variant (selectStopFace) before ever reaching here, so this
// path cannot miss a SelectionError.
func impliedOneFace(body *Body, q *FaceQuery, actual int) error {
	return q.selectionError(body, actual, expectedExactlyOne, ErrCardinality)
}

// impliedOneEdge builds the implicit exactly-one SelectionError an EdgeAxis
// reports when its selector resolves to a count that is not one, the edge
// analog of impliedOneFace. It takes the concrete *EdgeQuery for the same
// reason: resolveEdgeAxis gates the selector to the built-in variant first.
func impliedOneEdge(body *Body, q *EdgeQuery, actual int) error {
	return q.selectionError(body, actual, expectedExactlyOne, ErrCardinality)
}
