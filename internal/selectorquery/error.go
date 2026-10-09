package selectorquery

import (
	"fmt"
	"strings"
)

// SelectorKind names the entity a selector picks.
type SelectorKind int

const (
	EdgeSelectorKind SelectorKind = iota
	FaceSelectorKind
)

// String reports the selected entity in singular form.
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

// SelectionError records a failed query's cardinality and per-clause counts.
// Body retains the caller's body type; the root adapter supplies its stable
// feature reference for the message without giving this package topology.
type SelectionError[B any] struct {
	Kind      SelectorKind
	Query     string
	Body      B
	Expected  string
	Actual    int
	Residuals []Residual
	branches  int
	bodyRef   string
	err       error
}

// NewSelectionError records the details of one failed resolution. bodyRef is
// empty when the query has no body to name.
func NewSelectionError[B any](kind SelectorKind, query string, body B, bodyRef, expected string,
	actual int, residuals []Residual, branches int, err error) *SelectionError[B] {
	return &SelectionError[B]{
		Kind: kind, Query: query, Body: body, Expected: expected,
		Actual: actual, Residuals: residuals, branches: branches,
		bodyRef: bodyRef, err: err,
	}
}

// Error renders the wrapped sentinel, query, counts, and emptied clauses.
func (e *SelectionError[B]) Error() string {
	noun := "edges"
	if e.Kind == FaceSelectorKind {
		noun = "faces"
	}
	subject := e.Query
	if e.bodyRef != "" {
		subject += " on body " + e.bodyRef
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

// Unwrap returns the sentinel used to classify the selection failure.
func (e *SelectionError[B]) Unwrap() error { return e.err }

// emptiedClauses keeps the first zero-count clause of each branch.
func (e *SelectionError[B]) emptiedClauses() []Residual {
	var out []Residual
	for i, r := range e.Residuals {
		if r.Remaining != 0 {
			continue
		}
		if i > 0 && e.Residuals[i-1].Branch == r.Branch && e.Residuals[i-1].Remaining == 0 {
			continue
		}
		out = append(out, r)
	}
	return out
}
