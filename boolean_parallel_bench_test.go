package decad

import (
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

// BenchmarkMeshBooleanWorkers keeps operand tessellations cached so each
// sub-benchmark compares the same read-only boolean pipeline at one worker cap.
func BenchmarkMeshBooleanWorkers(b *testing.B) {
	for _, workers := range []int{1, 2, 4, 8, 12} {
		b.Run(strconv.Itoa(workers), func(b *testing.B) {
			doc := New()
			a := internalDiscBody(b, doc, 10, 20)
			tool := internalDiscBody(b, doc, 7, 22)
			tr, err := r3.Translation(r3.Vec{X: 10, Z: -11})
			if err != nil {
				b.Fatal(err)
			}
			placed, err := Transform(b.Context(), tool, tr)
			if err != nil {
				b.Fatal(err)
			}
			ctx := meshbool.WithContactWorkers(b.Context(), workers)
			if _, err := evaluateBoolean(ctx, meshbool.OpUnion, a, placed); err != nil {
				b.Fatal(err)
			}
			b.ResetTimer()
			for b.Loop() {
				if _, err := evaluateBoolean(ctx, meshbool.OpUnion, a, placed); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
