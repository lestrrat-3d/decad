package decad

import "testing"

var (
	dyadicBenchmarkSink dyadic
	dyadicCompareSink   int
)

func BenchmarkDyadicCompareEqualExponent(b *testing.B) {
	a, c := mustDyOf(1.25), mustDyOf(1.75)
	b.ReportAllocs()
	for b.Loop() {
		dyadicCompareSink = dyCmp(a, c)
	}
}

func BenchmarkDyadicCompareZero(b *testing.B) {
	a := mustDyOf(1.25)
	b.ReportAllocs()
	for b.Loop() {
		dyadicCompareSink = dyCmp(dyZero(), a)
	}
}

func BenchmarkDyadicAddEqualExponent(b *testing.B) {
	a, c := mustDyOf(1.25), mustDyOf(1.75)
	b.ReportAllocs()
	for b.Loop() {
		dyadicBenchmarkSink = dyAdd(a, c)
	}
}

func BenchmarkDyadicAddMixedExponent(b *testing.B) {
	a, c := mustDyOf(1.25), mustDyOf(0x1p-40)
	b.ReportAllocs()
	for b.Loop() {
		dyadicBenchmarkSink = dyAdd(a, c)
	}
}
