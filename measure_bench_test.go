package decad_test

// Benchmarks for the Body measurement accessors: Volume, Area and Centroid.
// A body computes and stores its measurements while it is built, so each
// accessor is a warm read of the stored result; there is no cold measurement
// path to time separately, and the build cost is what the modeling and
// tessellation benchmarks already cover. Each fixture builds its body outside
// the timed loop. The timed loop holds only the accessor call and a plain
// error check: a testify assertion walks the call stack on every call, which
// costs two orders of magnitude more than the read it would be checking, so
// the closed-form check runs once on the last reading after the loop. The
// accessor is a pure read of a stored value, so the last reading is every
// reading.

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// measureCase is a curved body with its closed-form volume, area and centroid.
type measureCase struct {
	name     string
	body     *decad.Body
	volume   float64
	area     float64
	centroid r3.Vec
}

func measureCases(b *testing.B) []measureCase {
	b.Helper()
	sphere := benchRevolve(func(s *sketch.Sketch) {
		o := s.CreatePoint(0, 0)
		s.Fix(o)
		left := s.CreatePoint(-8, 0)
		right := s.CreatePoint(8, 0)
		s.CreateArc(o, right, left)
		s.CreateLine(left, right)
	}, decad.FullRevolution{})
	// A 40x24x12 box with its four vertical edges rounded at radius 6: the
	// footprint loses (4-pi)r^2 at each corner and the wall runs the rounded
	// perimeter 2(40+24) - 8r + 2*pi*r.
	fillet := must(benchBox().Fillet(b.Context(),
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(6)))
	footprint := 40*24 - (4-math.Pi)*36
	perimeter := 2*(40+24) - 8*6 + 2*math.Pi*6
	return []measureCase{
		{
			name:     "Torus",
			body:     benchTorus(),
			volume:   2 * math.Pi * math.Pi * 10 * 3 * 3,
			area:     4 * math.Pi * math.Pi * 10 * 3,
			centroid: r3.NewVec(0, 0, 0),
		},
		{
			name:     "Sphere",
			body:     sphere,
			volume:   4.0 / 3.0 * math.Pi * 8 * 8 * 8,
			area:     4 * math.Pi * 8 * 8,
			centroid: r3.NewVec(0, 0, 0),
		},
		{
			name:     "FilletBox",
			body:     fillet,
			volume:   footprint * 12,
			area:     2*footprint + perimeter*12,
			centroid: r3.NewVec(20, 12, 6),
		},
	}
}

// BenchmarkMeasureVolume measures the warm Volume accessor on curved bodies.
func BenchmarkMeasureVolume(b *testing.B) {
	for _, tc := range measureCases(b) {
		b.Run(tc.name, func(b *testing.B) {
			b.ReportAllocs()
			var m decad.Measurement
			for b.Loop() {
				var err error
				m, err = tc.body.Volume()
				if err != nil {
					b.Fatal(err)
				}
			}
			require.InEpsilon(b, tc.volume, m.Value.Base(), 1e-9)
		})
	}
}

// BenchmarkMeasureArea measures the warm Area accessor on curved bodies.
func BenchmarkMeasureArea(b *testing.B) {
	for _, tc := range measureCases(b) {
		b.Run(tc.name, func(b *testing.B) {
			b.ReportAllocs()
			var m decad.Measurement
			for b.Loop() {
				var err error
				m, err = tc.body.Area()
				if err != nil {
					b.Fatal(err)
				}
			}
			require.InEpsilon(b, tc.area, m.Value.Base(), 1e-9)
		})
	}
}

// BenchmarkMeasureCentroid measures the warm Centroid accessor on curved
// bodies.
func BenchmarkMeasureCentroid(b *testing.B) {
	for _, tc := range measureCases(b) {
		b.Run(tc.name, func(b *testing.B) {
			b.ReportAllocs()
			var m decad.VecMeasurement
			for b.Loop() {
				var err error
				m, err = tc.body.Centroid()
				if err != nil {
					b.Fatal(err)
				}
			}
			require.InDelta(b, tc.centroid.X, m.Value.X, 1e-9)
			require.InDelta(b, tc.centroid.Y, m.Value.Y, 1e-9)
			require.InDelta(b, tc.centroid.Z, m.Value.Z, 1e-9)
		})
	}
}
