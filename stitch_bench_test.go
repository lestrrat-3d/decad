package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkStitchPatchCappedTube measures Stitch's curved volume calculation
// after a real extruded sheet has been capped through Body.Patch.
func BenchmarkStitchPatchCappedTube(b *testing.B) {
	b.ReportAllocs()
	for b.Loop() {
		b.StopTimer()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		s.CreateCircle(s.CreatePoint(0, 0), 10)
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		doc := decad.New()
		sheet, err := doc.Extrude(s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			decad.WithSurfaceResult())
		require.NoError(b, err)
		capped, err := sheet.Patch(b.Context(), decad.Edges(decad.Free()).Exactly(2))
		require.NoError(b, err)
		b.StartTimer()
		solid, err := decad.Stitch(b.Context(), capped)
		b.StopTimer()
		require.NoError(b, err)
		require.Equal(b, decad.BodySolid, solid.Kind())
		volume, err := solid.Volume()
		require.NoError(b, err)
		require.InDelta(b, 1000*math.Pi, volume.Value.Base(), 1e-8)
		require.Equal(b, []*decad.Body{solid}, doc.Bodies())
		b.StartTimer()
	}
}

// BenchmarkStitchTorusRevolveSheet measures Stitch's curved volume calculation
// on a closed sheet with one cylindrical face and one toroidal face.
func BenchmarkStitchTorusRevolveSheet(b *testing.B) {
	b.ReportAllocs()
	wantVolume, _ := halfTorusAnalytics(10, 5)
	for b.Loop() {
		b.StopTimer()
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(b, err)
		start := s.CreatePoint(0, 10)
		s.Fix(start)
		end := s.CreatePoint(10, 10)
		center := s.CreatePoint(5, 10)
		s.CreateLine(start, end)
		s.CreateArc(center, end, start)
		_, err = s.Solve(b.Context())
		require.NoError(b, err)
		doc := decad.New()
		sheet, err := doc.Revolve(s, s.Profiles()[0], uAxis,
			decad.FullRevolution{}, decad.WithSurfaceResult())
		require.NoError(b, err)
		b.StartTimer()
		solid, err := decad.Stitch(b.Context(), sheet)
		b.StopTimer()
		require.NoError(b, err)
		require.Equal(b, decad.BodySolid, solid.Kind())
		volume, err := solid.Volume()
		require.NoError(b, err)
		require.InDelta(b, wantVolume, volume.Value.Base(), 1e-8)
		require.Equal(b, []*decad.Body{solid}, doc.Bodies())
		b.StartTimer()
	}
}
