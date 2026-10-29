package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func benchmarkInfillSkin(b *testing.B) *decad.Body {
	b.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		b.Fatal(err)
	}
	rect := s.CreateRectangle(5, 5, 15, 15)
	s.Fix(rect.A)
	if _, err := s.Solve(b.Context()); err != nil {
		b.Fatal(err)
	}
	block, err := decad.Extrude(b.Context(), doc, s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(12), Dir: decad.Along,
	})
	if err != nil {
		b.Fatal(err)
	}
	out, err := decad.Infill(b.Context(), block, decad.InfillSpec{
		Kind: decad.GyroidInfill, Cell: units.Millimeters(10), Level: units.Scalar(0.6),
		Resolution: 8, Skin: units.Millimeters(1),
	})
	if err != nil {
		b.Fatal(err)
	}
	return out
}

func BenchmarkInfillSkin(b *testing.B) {
	b.Run("construct", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			benchmarkInfillSkin(b)
		}
	})

	b.Run("locate-grid-10x10x10", func(b *testing.B) {
		out := benchmarkInfillSkin(b)
		points := make([]r3.Vec, 0, 1000)
		for i := range 10 {
			for j := range 10 {
				for k := range 10 {
					points = append(points, r3.NewVec(
						5.3+9.4*float64(i)/9,
						5.3+9.4*float64(j)/9,
						0.3+11.4*float64(k)/9,
					))
				}
			}
		}
		b.ReportAllocs()
		b.ResetTimer()
		for range b.N {
			for _, point := range points {
				if _, err := out.LocatePoint(b.Context(), point, units.Millimeters(0.05)); err != nil {
					b.Fatal(err)
				}
			}
		}
	})
}
