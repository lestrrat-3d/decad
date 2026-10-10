package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

func BenchmarkFreeformArcLengthBracket(b *testing.B) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	spans, err := splinebezier.SplineBezierSpans(splineSeg{Control: point2ToRecordSlice(control), TStart: 0, TEnd: 1}, &freeform.FreeformWork{})
	if err != nil {
		b.Fatal(err)
	}

	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		if _, _, err := freeform.FreeformArcLength(spans, &freeform.FreeformWork{}); err != nil {
			b.Fatal(err)
		}
	}
}
