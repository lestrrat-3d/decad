package loftcylinder

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// BenchmarkRingEdgesInsideShared checks single-seam-edge rings over many
// teeth of the fixture circle. Each ring runs the whole per-tooth seam loop
// and is refused only afterwards, for lack of a unique occupied sector.
func BenchmarkRingEdgesInsideShared(b *testing.B) {
	var teeth []RingToothLevel
	for i := range 16 {
		teeth = append(teeth, RingToothLevel{
			Circle: fixtureCircle(), Level: big.NewRat(int64(i+1), 20), Bound2: big.NewRat(1, 64),
		})
	}
	var rings [][][2]proofarith.Xpt
	for _, x0 := range []float64{3.2, 3.4} {
		for _, y := range []float64{-0.2, -0.1, 0, 0.1, 0.2} {
			rings = append(rings, [][2]proofarith.Xpt{{
				proofarith.XptOf(r3.Vec{X: x0, Y: y}), proofarith.XptOf(r3.Vec{X: x0 + 0.5, Y: y}),
			}})
		}
	}
	for _, edges := range rings {
		got, err := RingEdgesInsideShared(b.Context(), edges, teeth, 3.5, 0.5, new(RingEdgeRecords))
		require.Equal(b, 1, got.Seam)
		require.ErrorContains(b, err, "no unique occupied tooth sector")
	}
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		for _, edges := range rings {
			_, _ = RingEdgesInsideShared(b.Context(), edges, teeth, 3.5, 0.5, new(RingEdgeRecords))
		}
	}
}
