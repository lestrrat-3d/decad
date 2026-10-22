// Package gaptol carries a document's gap tolerance to the evaluator code that
// reads it (agent-docs/gap-tolerance-design.md). The tolerance rides on the
// context an operation already passes down, so no evaluator signature grows a
// parameter for it.
package gaptol

import "context"

// Default is the reach, in millimetres, of a document that sets none: a
// hundredth of a millimetre, a tenth of the smallest feature the unaided eye
// resolves at reading distance.
const Default = 0.01

type key struct{}

// With returns ctx carrying reach, the farthest in millimetres an evaluator
// may move a held vertex from the exact point it stands for. Zero means exact
// only.
func With(ctx context.Context, reach float64) context.Context {
	return context.WithValue(ctx, key{}, reach)
}

// Reach returns the reach ctx carries, and zero when it carries none: code
// that no root operation called is exact-only.
func Reach(ctx context.Context) float64 {
	reach, _ := ctx.Value(key{}).(float64)
	return reach
}
