package decad_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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

// BenchmarkVerifyClearanceBoxes measures a complete clearance request for two
// disjoint, sketch-built solids. Their nearest vertical edges are 3 by 2 mm apart.
func BenchmarkVerifyClearanceBoxes(b *testing.B) {
	doc := decad.New()
	benchBoxBody(b, doc, 0, 0, 10, 10, 10)
	benchBoxBody(b, doc, 13, 12, 23, 22, 10)
	b.ResetTimer()
	for b.Loop() {
		report, err := doc.Verify(b.Context(), decad.WithClearances())
		if err != nil {
			b.Fatal(err)
		}
		if report.Status != decad.Sound || len(report.Clearances) != 1 {
			b.Fatalf("unexpected clearance report: status=%s, rows=%d", report.Status, len(report.Clearances))
		}
	}
}

// BenchmarkVerifyClearanceTori measures the complete certified gap request
// for two parallel, sketch-built tori. Their tube surfaces are 6 mm apart.
func BenchmarkVerifyClearanceTori(b *testing.B) {
	doc := decad.New()
	torusBody(b, doc, 10, 2)
	torus := torusBody(b, doc, 10, 2)
	shift, err := r3.Translation(r3.NewVec(0, 30, 0))
	if err != nil {
		b.Fatal(err)
	}
	if _, err := torus.Placed(b.Context(), shift); err != nil {
		b.Fatal(err)
	}
	b.ResetTimer()
	for b.Loop() {
		report, err := doc.Verify(b.Context(), decad.WithClearances())
		if err != nil {
			b.Fatal(err)
		}
		if len(report.Clearances) != 1 || math.Abs(report.Clearances[0].Gap.Value.Mag()-6) > 1e-6 {
			b.Fatalf("unexpected torus clearance report: %#v", report)
		}
	}
}
