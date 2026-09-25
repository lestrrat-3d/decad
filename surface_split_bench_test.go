package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkSplitBlockByRibbon measures the section arrangement and rebuild of
// two solids. Each iteration builds fresh operands because Split retires them.
func BenchmarkSplitBlockByRibbon(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		target := splitBlock(b, doc)
		tool := splitRibbon(b, doc, 110)
		b.StartTimer()
		pieces, err := doc.Split(b.Context(), target, tool)
		require.NoError(b, err)
		require.Len(b, pieces, 2)
		for _, piece := range pieces {
			require.True(b, piece.IsSolid())
			volume, err := piece.Volume()
			require.NoError(b, err)
			require.InDelta(b, 30000, volume.Value.Base(), 1e-8)
		}
	}
}

// BenchmarkSplitBlockByCircle measures the circular section arrangement and
// rebuild. The circular sheet spans the block's full 10 mm height.
func BenchmarkSplitBlockByCircle(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		target := splitBlock(b, doc)
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		center := s.CreatePoint(50, 30)
		s.Fix(center)
		s.CreateCircle(center, 20)
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		tool, err := doc.Extrude(s, s.Profiles()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(15)},
			Two: decad.DistanceSide{D: units.Millimeters(5)},
		}, decad.WithSurfaceResult())
		require.NoError(b, err)
		b.StartTimer()
		pieces, err := doc.Split(b.Context(), target, tool)
		require.NoError(b, err)
		require.Len(b, pieces, 2)
		total := 0.0
		for _, piece := range pieces {
			require.True(b, piece.IsSolid())
			volume, err := piece.Volume()
			require.NoError(b, err)
			require.Positive(b, volume.Value.Base())
			total += volume.Value.Base()
		}
		require.InDelta(b, 60000, total, 1e-8)
	}
}

// BenchmarkSplitBlockByTwoRibbons cuts the block at y=20 and y=40 with a
// single sketch-built open chain. The return path joins the cuts outside the
// block, so the real Split operation must build three solid cells.
func BenchmarkSplitBlockByTwoRibbons(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		target := splitBlock(b, doc)
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		a := s.CreatePoint(-10, 20)
		s.Fix(a)
		p := s.CreatePoint(110, 20)
		q := s.CreatePoint(110, 40)
		r := s.CreatePoint(-10, 40)
		s.CreateLine(a, p)
		s.CreateLine(p, q)
		s.CreateLine(q, r)
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		require.Len(b, s.Chains(), 1)
		tool, err := doc.ExtrudeChain(s, s.Chains()[0], decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(15)},
			Two: decad.DistanceSide{D: units.Millimeters(5)},
		})
		require.NoError(b, err)
		b.StartTimer()
		pieces, err := doc.Split(b.Context(), target, tool)
		require.NoError(b, err)
		require.Len(b, pieces, 3)
		for _, piece := range pieces {
			require.True(b, piece.IsSolid())
			volume, err := piece.Volume()
			require.NoError(b, err)
			require.InDelta(b, 20000, volume.Value.Base(), 1e-8)
		}
		require.Equal(b, pieces, doc.Bodies())
	}
}
