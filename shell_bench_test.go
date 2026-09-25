package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkShellCupWithPost measures a one-cap shell around a circular hole.
// Each iteration builds a fresh receiver because Shell retires its input.
func BenchmarkShellCupWithPost(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc, box := circleHoledBox(b, [3]float64{50, 30, 8})
		b.StartTimer()
		body, err := box.Shell(b.Context(), topCap(box), units.Millimeters(5))
		require.NoError(b, err)
		require.True(b, body.IsSolid())
		require.Len(b, body.Faces(), 14)
		require.Equal(b, []*decad.Body{body}, doc.Bodies())
	}
}
