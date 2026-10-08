package decad

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/decad/internal/selectorquery"
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

// PredicateResidual is the running match count after one clause of a query
// branch's conjunction, evaluated cumulatively in query order: Remaining is
// the number of candidates still matching after this clause AND every earlier
// clause of the same branch. The clause whose Remaining reaches zero is the
// one that emptied its branch.
type PredicateResidual struct {
	// Branch is the zero-based union branch containing the clause.
	Branch int
	// Predicate is the clause's stable rendering.
	Predicate string
	// Remaining is the candidate count after this and all preceding clauses.
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
	// Residuals is the running match count after each clause, in query
	// order: the branches one after another, each restarting from the whole
	// count. A branch with no clauses contributes no entry.
	Residuals []PredicateResidual
	// branches is the query's branch count; above one, Error labels each
	// emptied clause with its branch.
	branches int
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
	var b strings.Builder
	fmt.Fprintf(&b, "%s: %s matched %d %s, expected %s", e.err, subject, e.Actual, noun, e.Expected)
	for _, r := range e.emptiedClauses() {
		if e.branches > 1 {
			fmt.Fprintf(&b, "; the clause %s of branch %d matched none", r.Predicate, r.Branch)
			continue
		}
		fmt.Fprintf(&b, "; the clause %s matched none", r.Predicate)
	}
	return b.String()
}

// Unwrap returns the wrapped sentinel — ErrNoMatch or ErrCardinality — so
// errors.Is branches on the selection outcome exactly as before the type
// existed.
func (e *SelectionError) Unwrap() error { return e.err }

// emptiedClauses returns, per branch in order, the first clause whose running
// match count reached zero — the clause that emptied that branch's
// conjunction. A branch that kept a candidate contributes nothing.
func (e *SelectionError) emptiedClauses() []PredicateResidual {
	var out []PredicateResidual
	for i, r := range e.Residuals {
		if r.Remaining != 0 {
			continue
		}
		// Only the first zero of a branch: every later clause of the same
		// branch reads zero too.
		if i > 0 && e.Residuals[i-1].Branch == r.Branch && e.Residuals[i-1].Remaining == 0 {
			continue
		}
		out = append(out, r)
	}
	return out
}

// expectedExactlyOne is the Expected an implicit exactly-one (ToFace,
// ToFaceAngular, EdgeAxis) renders.
const expectedExactlyOne = "exactly 1"

// expected renders the recorded cardinality for a selection error.
func (c cardinality) expected() string {
	return (selectorquery.Cardinality{Kind: c.kind, N: c.n}).Expected()
}

// String renders the query canonically:
// edges(<pred>, <pred>, …).or(<pred>, …)…<cardinality>, one .or(…) per Or
// call in call order and the cardinality last. The rendering is a deterministic function of the recorded content — equal
// recorded queries render identically, and a query and its decoded round-trip
// render identically — built from the codec's own tagged vocabulary. It is an
// identity for diagnostics and equality, not a parseable format.
func (q *EdgeQuery) String() string {
	if q == nil {
		return selKindEdges + "()"
	}
	branches := make([][]string, len(q.branches))
	for i, branch := range q.branches {
		branches[i] = make([]string, len(branch))
		for j, p := range branch {
			branches[i][j] = p.render()
		}
	}
	return renderQuery(selKindEdges, branches, q.card)
}

// String renders the query canonically:
// faces(<pred>, <pred>, …).or(<pred>, …)…<cardinality>, the face analog of
// EdgeQuery.String.
func (q *FaceQuery) String() string {
	if q == nil {
		return selKindFaces + "()"
	}
	branches := make([][]string, len(q.branches))
	for i, branch := range q.branches {
		branches[i] = make([]string, len(branch))
		for j, p := range branch {
			branches[i][j] = p.render()
		}
	}
	return renderQuery(selKindFaces, branches, q.card)
}

// renderQuery assembles the ordered selector branches.
func renderQuery(kind string, branches [][]string, card cardinality) string {
	return selectorquery.Render(kind, branches, selectorquery.Cardinality{Kind: card.kind, N: card.n})
}

func (p EdgePredicate) render() string { return p.clause().Render(renderRef) }
func (p FacePredicate) render() string { return p.clause().Render(renderRef) }

func renderVec(v r3.Vec) string    { return selectorquery.RenderVec(v) }
func renderCoord(c float64) string { return selectorquery.RenderCoord(c) }

// renderRef renders a FeatureRef as <producer>:<role> — the private ID in decimal and
// the role quoted, so a role that itself holds parentheses or commas stays
// unambiguous: 3:"capStart", 2:"side(0,1)".
func renderRef(f FeatureRef) string {
	return strconv.Itoa(int(f.producer)) + ":" + strconv.Quote(f.Role)
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
		branches:  len(q.branches),
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
		branches:  len(q.branches),
		err:       sentinel,
	}
}

// residuals evaluates edge predicates cumulatively on the failing path.
func (q *EdgeQuery) residuals(body *Body) []PredicateResidual {
	return selectorResiduals(selectorquery.Residuals(body.Edges(), q.branches, EdgePredicate.matches, EdgePredicate.render))
}

// residuals evaluates face predicates cumulatively on the failing path.
func (q *FaceQuery) residuals(body *Body) []PredicateResidual {
	return selectorResiduals(selectorquery.Residuals(body.Faces(), q.branches, FacePredicate.matches, FacePredicate.render))
}

func selectorResiduals(records []selectorquery.Residual) []PredicateResidual {
	if len(records) == 0 {
		return nil
	}
	out := make([]PredicateResidual, len(records))
	for i, record := range records {
		out[i] = PredicateResidual{Branch: record.Branch, Predicate: record.Predicate, Remaining: record.Remaining}
	}
	return out
}

// impliedOneFace builds the implicit exactly-one SelectionError a ToFace /
// ToFaceAngular stop or a MirrorFace reports when its selector resolves to a
// count that is not one (core §9). Expected is rewritten to "exactly 1" and
// Actual is the resolved count, preserving the resolution's Kind, Query, Body
// and Residuals. It takes the concrete *FaceQuery: the implicit-one callers
// gate the selector to the built-in variant (selectImpliedOneFace) before ever
// reaching here, so this path cannot miss a SelectionError.
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
