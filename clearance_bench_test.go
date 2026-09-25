package decad_test

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad"
)

// BenchmarkVerifyClearanceManyBoxes measures the requested gap for every pair
// of disjoint, sketch-built rectangular prisms.
func BenchmarkVerifyClearanceManyBoxes(b *testing.B) {
	for _, count := range []int{2, 4, 8} {
		b.Run(fmt.Sprintf("%d_bodies", count), func(b *testing.B) {
			doc := decad.New()
			for i := range count {
				x := float64(i * 20)
				benchBoxBody(b, doc, x, 0, x+10, 10, 10)
			}
			wantPairs := count * (count - 1) / 2
			b.ResetTimer()
			for b.Loop() {
				report, err := doc.Verify(b.Context(), decad.WithClearances())
				if err != nil {
					b.Fatal(err)
				}
				if report.Status != decad.Sound || len(report.Clearances) != wantPairs {
					b.Fatalf("unexpected report: status=%s, rows=%d", report.Status, len(report.Clearances))
				}
			}
		})
	}
}

// BenchmarkVerifyClearanceManyRods measures requested gaps between disjoint
// cylindrical prisms built from circle sketches.
func BenchmarkVerifyClearanceManyRods(b *testing.B) {
	for _, count := range []int{2, 4, 8} {
		b.Run(fmt.Sprintf("%d_bodies", count), func(b *testing.B) {
			doc := decad.New()
			for i := range count {
				benchRodBody(b, doc, float64(i*20), 0, 2)
			}
			wantPairs := count * (count - 1) / 2
			b.ResetTimer()
			for b.Loop() {
				report, err := doc.Verify(b.Context(), decad.WithClearances())
				if err != nil {
					b.Fatal(err)
				}
				if report.Status != decad.Sound || len(report.Clearances) != wantPairs {
					b.Fatalf("unexpected report: status=%s, rows=%d", report.Status, len(report.Clearances))
				}
			}
		})
	}
}
