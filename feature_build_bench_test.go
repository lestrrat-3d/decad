package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
)

// BenchmarkSweepComposite measures the complete build of a solid along two
// circular spans joined by a line. The solved profile and path are reused.
func BenchmarkSweepComposite(b *testing.B) {
	s, profile := orthogonalSweepProfile(b)
	path := orthogonalSweepPath(b)
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Sweep(b.Context(), s, profile, path)
		if err != nil {
			b.Fatal(err)
		}
		if len(body.Faces()) != 14 {
			b.Fatalf("unexpected sweep face count: %d", len(body.Faces()))
		}
	}
}

// BenchmarkLoftFrustum measures a complete ruled loft between different-sized
// square sections. The solved section sketches are reused.
func BenchmarkLoftFrustum(b *testing.B) {
	s0, p0, s1, p1 := loftSquares(b, 20, 10)
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Loft(b.Context(), s0, p0, s1, p1)
		if err != nil {
			b.Fatal(err)
		}
		if len(body.Faces()) != 10 {
			b.Fatalf("unexpected loft face count: %d", len(body.Faces()))
		}
	}
}
