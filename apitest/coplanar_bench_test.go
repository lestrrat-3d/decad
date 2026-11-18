package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
)

// BenchmarkFlushUnion times the Union of two boxes meeting face to face under
// the default gap tolerance (agent-docs/coplanar-contact-design.md §10).
func BenchmarkFlushUnion(b *testing.B) {
	for range b.N {
		// A Union consumes its operands, so each run builds its own.
		b.StopTimer()
		doc := decad.New()
		// The upper box is moved into place by a Transform, so the pair reaches
		// the mesh evaluator rather than the analytic stacked union.
		lower := boxBody(b, doc, 0, 0, 10, 10, 10)
		upper := translated(b, boxBody(b, doc, 0, 0, 10, 10, 10), 0.5, 0, 10)
		b.StartTimer()
		if _, err := decad.Union(context.Background(), lower, upper); err != nil {
			b.Fatal(err)
		}
	}
}
