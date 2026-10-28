package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// atan2BenchArgs returns n float-held direction pairs around a radius-3
// circle, the shape of the arc endpoint offsets the circular brackets read.
func atan2BenchArgs(n int) [][2]*big.Rat {
	rng := rand.New(rand.NewPCG(7, 11))
	out := make([][2]*big.Rat, n)
	for i := range out {
		a := rng.Float64() * 2 * math.Pi
		r := 3 + rng.Float64()
		out[i] = [2]*big.Rat{new(big.Rat).SetFloat64(r * math.Sin(a)), new(big.Rat).SetFloat64(r * math.Cos(a))}
	}
	return out
}

// BenchmarkAtan2IntervalRepeated cycles through 230 distinct directions, the
// distinct-argument count one fillet-then-chamfer test asks Atan2Interval for
// across roughly 11k calls.
func BenchmarkAtan2IntervalRepeated(b *testing.B) {
	args := atan2BenchArgs(230)
	b.ReportAllocs()
	i := 0
	for b.Loop() {
		a := args[i%len(args)]
		proofbound.Atan2Interval(a[0], a[1], false)
		i++
	}
}

// BenchmarkAtan2IntervalDistinct asks for a direction never asked before on
// every iteration, so each call evaluates the series.
func BenchmarkAtan2IntervalDistinct(b *testing.B) {
	b.ReportAllocs()
	i := 0
	for b.Loop() {
		i++
		a := float64(i) * 0.6180339887498949
		y := new(big.Rat).SetFloat64(3 * math.Sin(a))
		x := new(big.Rat).SetFloat64(3 * math.Cos(a))
		proofbound.Atan2Interval(y, x, false)
	}
}
