package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// BenchmarkVerifyClearanceBoxes measures the complete clearance report for
// two disjoint axis-aligned rectangular prisms built from real sketches.
func BenchmarkVerifyClearanceBoxes(b *testing.B) {
	doc := decad.New()
	benchBoxBody(b, doc, 0, 0, 10, 10, 10)
	benchBoxBody(b, doc, 13, 12, 23, 22, 10)
	b.ResetTimer()
	for b.Loop() {
		report, err := doc.Verify(b.Context(), decad.WithClearances())
		require.NoError(b, err)
		require.Len(b, report.Clearances, 1)
	}
}
