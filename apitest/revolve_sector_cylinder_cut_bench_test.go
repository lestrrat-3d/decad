package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkRevolveSectorCylinderCut measures the bore Cut through a hub and
// radial lug join: radius 1 takes the annular route (sector_bore_cut.go),
// radius 2 the straight band route (sector_bore_band.go), and "repeated" bores
// a join of two lugs. Cut retires its operands, so each iteration builds
// fresh ones with the timer stopped and times only the Cut.
func BenchmarkRevolveSectorCylinderCut(b *testing.B) {
	for _, bc := range []struct {
		name     string
		radius   float64
		repeated bool
	}{
		{name: "annular", radius: 1},
		{name: "band", radius: 2},
		{name: "repeated annular", radius: 1, repeated: true},
	} {
		b.Run(bc.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				b.StopTimer()
				joined, tool := benchSectorBoreOperands(b, bc.radius, bc.repeated)
				b.StartTimer()
				if _, err := decad.Cut(b.Context(), joined, tool); err != nil {
					b.Fatalf("cut: %v", err)
				}
			}
		})
	}
}

// benchSectorBoreOperands builds the hub and lug join, with a second lug a
// half turn away when repeated, and a bore tool of the given radius.
func benchSectorBoreOperands(b *testing.B, radius float64, repeated bool) (*decad.Body, *decad.Body) {
	b.Helper()
	doc, hub, lug, _, _ := taperedHubAndRadialLug(b, 9, 7.5, 4.3, false)
	parts := []*decad.Body{lug}
	if repeated {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Degrees(180))
		require.NoError(b, err)
		second, err := decad.Copy(b.Context(), lug, decad.WithCopyPlacement(turn))
		require.NoError(b, err)
		parts = append(parts, second)
	}
	joined, err := decad.Union(b.Context(), hub, parts...)
	require.NoError(b, err)
	return joined, revolveSectorCylinderTool(b, doc, radius, 4, 6, 0)
}
