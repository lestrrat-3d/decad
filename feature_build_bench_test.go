package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// BenchmarkSweepComposite measures the complete build of a solid along two
// circular spans joined by a line. The solved profile and path are reused.
func BenchmarkSweepComposite(b *testing.B) {
	s, profile := orthogonalSweepProfile(b)
	path := orthogonalSweepPath(b)
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Sweep(b.Context(), s, profile, path)
		if err != nil {
			b.Fatal(err)
		}
		if len(body.Faces()) != 14 {
			b.Fatalf("unexpected sweep face count: %d", len(body.Faces()))
		}
	}
}

// BenchmarkLoftFrustum measures a complete ruled loft between different-sized
// square sections. The solved section sketches are reused.
func BenchmarkLoftFrustum(b *testing.B) {
	s0, p0, s1, p1 := loftSquares(b, 20, 10)
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Loft(b.Context(), s0, p0, s1, p1)
		if err != nil {
			b.Fatal(err)
		}
		if len(body.Faces()) != 10 {
			b.Fatalf("unexpected loft face count: %d", len(body.Faces()))
		}
	}
}

// BenchmarkSweepCurvedPathScale compares one curved span with the supported
// two-arc, one-line spatial path using the same solved section.
func BenchmarkSweepCurvedPathScale(b *testing.B) {
	s, profile := orthogonalSweepProfile(b)
	oneArc, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.ArcThrough{
		Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5),
	})
	require.NoError(b, err)
	for _, tc := range []struct {
		name       string
		path       *decad.Path
		wantVolume float64
	}{
		{"one_arc", oneArc, 10 * math.Pi},
		{"two_arcs_one_line", orthogonalSweepPath(b), 40 + 20*math.Pi},
	} {
		b.Run(tc.name, func(b *testing.B) {
			for b.Loop() {
				body, err := decad.New().Sweep(b.Context(), s, profile, tc.path)
				if err != nil {
					b.Fatal(err)
				}
				b.StopTimer()
				require.True(b, body.IsSolid())
				decadtest.MeasuresVolume(b, body, units.CubicMillimeters(tc.wantVolume))
				b.StartTimer()
			}
			b.StopTimer()
		})
	}
}

// BenchmarkLoftCurvedCircle measures a complete ruled loft between two
// circular sections on solved parallel planes.
func BenchmarkLoftCurvedCircle(b *testing.B) {
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(b, err)
	s0, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	c0 := s0.CreatePoint(0, 0)
	s0.Fix(c0)
	s0.CreateCircle(c0, 10)
	_, err = s0.Solve(b.Context())
	require.NoError(b, err)
	s1, err := w.CreateSketch(top)
	require.NoError(b, err)
	c1 := s1.CreatePoint(0, 0)
	s1.Fix(c1)
	s1.CreateCircle(c1, 5)
	_, err = s1.Solve(b.Context())
	require.NoError(b, err)
	b.ResetTimer()
	for b.Loop() {
		body, err := decad.New().Loft(b.Context(), s0, s0.Profiles()[0], s1, s1.Profiles()[0])
		if err != nil {
			b.Fatal(err)
		}
		b.StopTimer()
		require.True(b, body.IsSolid())
		const r0, r1, height = 10.0, 5.0, 10.0
		wantVolume := math.Pi * height * (r0*r0 + r0*r1 + r1*r1) / 3
		decadtest.MeasuresVolume(b, body, units.CubicMillimeters(wantVolume))
		b.StartTimer()
	}
	b.StopTimer()
}
