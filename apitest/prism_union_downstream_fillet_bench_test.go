package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func BenchmarkPrismUnionDownstreamFillet(b *testing.B) {
	want := 2000 - 2*(96*(1-math.Pi/4)+8*(1-math.Pi/6))
	b.ReportAllocs()
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		a := decadtest.NewBlock(b, doc, 0, 0, 10, 10, units.Millimeters(10))
		bSrc := decadtest.NewBlock(b, doc, 5, 5, 15, 15, units.Millimeters(10))
		farAbove, err := r3.Translation(r3.Vec{Z: 30})
		require.NoError(b, err)
		bBody, err := decad.Transform(b.Context(), bSrc, farAbove)
		require.NoError(b, err)
		union, err := decad.Union(b.Context(), a, bBody)
		require.NoError(b, err)
		b.StartTimer()

		out, err := decad.Fillet(b.Context(), union, decad.Edges().AtLeast(1), units.Millimeters(1))
		b.StopTimer()
		require.NoError(b, err)
		volume, err := out.Volume()
		require.NoError(b, err)
		require.InDelta(b, want, volume.Value.Base(), volume.Bound.Base())
		b.StartTimer()
	}
	b.StopTimer()
}
