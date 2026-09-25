package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkFilletBoxAllConvexEdges measures the complete lateral-edge rewrite
// and prism build. The receiver is rebuilt outside the timed region because
// Fillet retires it.
func BenchmarkFilletBoxAllConvexEdges(b *testing.B) {
	const radius = 10.0
	wantVolume := (100*60 - (4-math.Pi)*radius*radius) * 20
	selector := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		doc := decad.New()
		box := benchBoxBody(b, doc, 0, 0, 100, 60, 20)
		edges, err := selector.SelectEdges(box)
		require.NoError(b, err)
		require.Len(b, edges, 4)
		b.StartTimer()
		body, err := box.Fillet(b.Context(), selector, units.Millimeters(radius))
		b.StopTimer()
		require.NoError(b, err)
		require.True(b, body.IsSolid())
		require.Len(b, body.Faces(), 10)
		require.Equal(b, []*decad.Body{body}, doc.Bodies())
		volume, err := body.Volume()
		require.NoError(b, err)
		require.InDelta(b, wantVolume, volume.Value.Base(), 1e-8)
	}
}

// BenchmarkChamferBoxAllConvexEdges measures the complete lateral-edge rewrite
// and prism build. The receiver is rebuilt outside the timed region because
// Chamfer retires it.
func BenchmarkChamferBoxAllConvexEdges(b *testing.B) {
	const setback = 10.0
	wantVolume := (100*60 - 4*setback*setback/2) * 20
	selector := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		doc := decad.New()
		box := benchBoxBody(b, doc, 0, 0, 100, 60, 20)
		edges, err := selector.SelectEdges(box)
		require.NoError(b, err)
		require.Len(b, edges, 4)
		b.StartTimer()
		body, err := box.Chamfer(b.Context(), selector, units.Millimeters(setback))
		b.StopTimer()
		require.NoError(b, err)
		require.True(b, body.IsSolid())
		require.Len(b, body.Faces(), 10)
		require.Equal(b, []*decad.Body{body}, doc.Bodies())
		volume, err := body.Volume()
		require.NoError(b, err)
		require.InDelta(b, wantVolume, volume.Value.Base(), 1e-8)
	}
}
