package decad

import "testing"

func BenchmarkFreeformArcLengthBracket(b *testing.B) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	if err != nil {
		b.Fatal(err)
	}

	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		if _, _, err := freeformArcLength(spans, &freeformWork{}); err != nil {
			b.Fatal(err)
		}
	}
}
