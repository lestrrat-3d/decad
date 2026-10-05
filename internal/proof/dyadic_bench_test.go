package proof_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
)

var (
	dyadicBenchmarkSink proof.Dyadic
	dyadicCompareSink   int
)

func BenchmarkDyadicCompareEqualExponent(b *testing.B) {
	a, c := proof.MustDyOf(1.25), proof.MustDyOf(1.75)
	b.ReportAllocs()
	for b.Loop() {
		dyadicCompareSink = proof.DyCmp(a, c)
	}
}

func BenchmarkDyadicCompareZero(b *testing.B) {
	a := proof.MustDyOf(1.25)
	b.ReportAllocs()
	for b.Loop() {
		dyadicCompareSink = proof.DyCmp(proof.DyZero(), a)
	}
}

func BenchmarkDyadicAddEqualExponent(b *testing.B) {
	a, c := proof.MustDyOf(1.25), proof.MustDyOf(1.75)
	b.ReportAllocs()
	for b.Loop() {
		dyadicBenchmarkSink = proof.DyAdd(a, c)
	}
}

func BenchmarkDyadicAddMixedExponent(b *testing.B) {
	a, c := proof.MustDyOf(1.25), proof.MustDyOf(0x1p-40)
	b.ReportAllocs()
	for b.Loop() {
		dyadicBenchmarkSink = proof.DyAdd(a, c)
	}
}
