package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

var benchmarkChordedAllow LoftChordedAllow

// BenchmarkLoftChordedCellAllows measures the exact wall-cell readings on a
// small, curved loft after station pairing and assembly have completed.
func BenchmarkLoftChordedCellAllows(b *testing.B) {
	const n = 8
	verts := make([]r3.Vec, 2*n)
	vIdx, wIdx := make([]int, n), make([]int, n)
	p := LoopPair{
		V:         make([]sectionrecord.Point2, n),
		ArcUpperV: make([]float64, n), ArcUpperW: make([]float64, n),
		MatchedDelta:   make([]float64, n),
		TangentEnergyV: make([]float64, n), TangentEnergyW: make([]float64, n),
		Faceted: make([]bool, n),
	}
	for j := range n {
		angle := 2 * math.Pi * float64(j) / n
		c, s := math.Cos(angle), math.Sin(angle)
		verts[j] = r3.NewVec(10*c, 10*s, 0)
		verts[n+j] = r3.NewVec(5*c, 5*s, 10)
		vIdx[j], wIdx[j] = j, n+j
		p.ArcUpperV[j], p.ArcUpperW[j] = 10*math.Pi/4, 5*math.Pi/4
		p.MatchedDelta[j] = 0.1
		p.TangentEnergyV[j], p.TangentEnergyW[j] = 1, 0.25
	}
	pairs := []LoopPair{p}
	b.ResetTimer()
	for b.Loop() {
		benchmarkChordedAllow = ComputeLoftChordedAllow(
			pairs, [][]int{vIdx}, [][]int{wIdx}, verts, r3.Vec{}, 0.1, 0, false,
		)
	}
}
