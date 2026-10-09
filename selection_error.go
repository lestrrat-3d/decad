package decad

import (
	"strconv"

	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/r3"
)

// SelectorKind names the entity a selector picks.
type SelectorKind = selectorquery.SelectorKind

const (
	EdgeSelectorKind = selectorquery.EdgeSelectorKind
	FaceSelectorKind = selectorquery.FaceSelectorKind
)

// PredicateResidual records the running count after one query clause.
type PredicateResidual = selectorquery.Residual

// SelectionError reports a failed edge or face query with the body's identity,
// asserted count, and the clause counts that explain the failure.
type SelectionError = selectorquery.SelectionError[*Body]

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
	return selectorquery.NewSelectionError(EdgeSelectorKind, q.String(), body, renderBodyRef(body),
		expected, actual, q.residuals(body), len(q.branches), sentinel)
}

// selectionError builds the SelectionError a face query's failing resolution
// reports, the face analog of EdgeQuery.selectionError.
func (q *FaceQuery) selectionError(body *Body, actual int, expected string, sentinel error) *SelectionError {
	return selectorquery.NewSelectionError(FaceSelectorKind, q.String(), body, renderBodyRef(body),
		expected, actual, q.residuals(body), len(q.branches), sentinel)
}

func renderBodyRef(body *Body) string {
	if body == nil {
		return ""
	}
	return renderRef(body.Origin())
}

// residuals evaluates edge predicates cumulatively on the failing path.
func (q *EdgeQuery) residuals(body *Body) []PredicateResidual {
	return selectorquery.Residuals(body.Edges(), q.branches, EdgePredicate.matches, EdgePredicate.render)
}

// residuals evaluates face predicates cumulatively on the failing path.
func (q *FaceQuery) residuals(body *Body) []PredicateResidual {
	return selectorquery.Residuals(body.Faces(), q.branches, FacePredicate.matches, FacePredicate.render)
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
