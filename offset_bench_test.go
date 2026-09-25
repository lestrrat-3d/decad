package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkOffsetPrismCircle measures a complete circular sheet offset. Each
// iteration builds a fresh source outside the timed region.
func BenchmarkOffsetPrismCircle(b *testing.B) {
	b.ReportAllocs()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		sheet := offsetCircleSheet(b, doc, 10)
		b.StartTimer()
		result, err := sheet.Offset(b.Context(), units.Millimeters(2))
		b.StopTimer()
		require.NoError(b, err)
		require.Len(b, result.Faces(), 1)
		area, err := result.Area()
		require.NoError(b, err)
		require.InDelta(b, 240*math.Pi, area.Value.Base(), 1e-8)
		box, err := result.Bounds()
		require.NoError(b, err)
		require.Equal(b, r3.NewVec(-12, -12, 0), box.Min)
		require.Equal(b, r3.NewVec(12, 12, 10), box.Max)
		require.Len(b, doc.Bodies(), 2)
		b.StartTimer()
	}
}

// BenchmarkOffsetPrismNarrowNeck measures the admitted concave-section offset
// and its crossing audit. Each iteration builds a fresh source outside time.
func BenchmarkOffsetPrismNarrowNeck(b *testing.B) {
	b.ReportAllocs()
	for b.Loop() {
		b.StopTimer()
		doc, sheet := thickenNeckSheet(b)
		b.StartTimer()
		result, err := sheet.Offset(b.Context(), units.Millimeters(1.5),
			decad.WithOffsetSide(decad.OffsetNegative))
		b.StopTimer()
		require.NoError(b, err)
		require.Len(b, result.Faces(), 16)
		area, err := result.Area()
		require.NoError(b, err)
		require.InDelta(b, 1080+30*math.Pi, area.Value.Base(), 1e-8)
		require.Len(b, doc.Bodies(), 2)
		b.StartTimer()
	}
}
