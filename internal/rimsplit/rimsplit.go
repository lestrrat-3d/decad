// Package rimsplit carries a document's choice to split the pieces of a mesh
// boolean beside their rims to the evaluator that reads it
// (agent-docs/faceted-vertex-bounds-design.md §3.7). The choice rides on the
// context an operation already passes down, as the chain tolerance does.
package rimsplit

import "context"

type key struct{}

// With returns ctx carrying the choice: on splits the pieces of every mesh
// boolean the context reaches.
func With(ctx context.Context, on bool) context.Context {
	return context.WithValue(ctx, key{}, on)
}

// Of reports whether ctx carries the choice to split, and false when it
// carries none.
func Of(ctx context.Context) bool {
	on, _ := ctx.Value(key{}).(bool)
	return on
}
