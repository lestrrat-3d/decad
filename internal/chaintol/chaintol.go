// Package chaintol carries a document's chain tolerance to the boolean
// evaluator that reads it (agent-docs/faceted-vertex-bounds-design.md §5). The
// tolerance rides on the context an operation already passes down, so no
// evaluator signature grows a parameter for it.
package chaintol

import "context"

type key struct{}

// With returns ctx carrying tol, the coarsest bound in millimetres that a
// held facet may carry where a boolean's pair meets it. Zero means the pair's
// own chord tolerance.
func With(ctx context.Context, tol float64) context.Context {
	return context.WithValue(ctx, key{}, tol)
}

// Of returns the tolerance ctx carries, and zero when it carries none.
func Of(ctx context.Context) float64 {
	tol, _ := ctx.Value(key{}).(float64)
	return tol
}
