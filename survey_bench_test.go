package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// BenchmarkVerifyHalfDiscPlate measures the half-disc plate build and its wall
// and concave-radius surveys. The solved profile is reused across iterations.
func BenchmarkVerifyHalfDiscPlate(b *testing.B) {
	const half, au, av = 20.0, 1.86, 1.02
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	if err != nil {
		b.Fatal(err)
	}
	rect := s.CreateRectangle(-half, -half, half, half)
	s.Fix(rect.A)
	o := s.CreatePoint(0, 0)
	p := s.CreatePoint(au, av)
	q := s.CreatePoint(-au, -av)
	s.CreateArc(o, p, q)
	s.CreateLine(q, p)
	if _, err := s.Solve(b.Context()); err != nil {
		b.Fatal(err)
	}
	var prof *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			prof = candidate
		}
	}
	if prof == nil {
		b.Fatal("the plate profile carries no half-disc hole")
	}

	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		doc := decad.New()
		if _, err := doc.Extrude(s, prof,
			decad.Distance{D: units.Millimeters(200), Dir: decad.Along},
		); err != nil {
			b.Fatal(err)
		}
		report, err := doc.Verify(b.Context(),
			decad.WithMinWallThickness(units.Millimeters(0.001)),
			decad.WithConcaveRadius(),
		)
		if err != nil {
			b.Fatal(err)
		}
		if len(report.Bodies) != 1 {
			b.Fatalf("expected one body report, got %d", len(report.Bodies))
		}
		body := report.Bodies[0]
		if body.Wall.Minimum == nil || body.ConcaveRadius.Minimum == nil {
			b.Fatal("verification omitted a requested minimum reading")
		}
		if _, err := body.Wall.Minimum.Value.In(units.Millimeter); err != nil {
			b.Fatal(err)
		}
		if _, err := body.ConcaveRadius.Minimum.Value.In(units.Millimeter); err != nil {
			b.Fatal(err)
		}
	}
}
