package decad_test

// Benchmarks for Body.Tessellate across every payload class it builds a mesh
// for: analytic prisms (box, fillet, free-form wall), a shell cup, a cap-loop
// chamfer, a loft, a mesh-boolean (faceted) result, and circular-generator
// revolves (line, sphere, torus, groove) both full and partial. Each fixture
// builds its body once outside the timed loop and reports triangle count and
// refusal rate alongside the timing, so a payload class that stages a refusal
// still yields a comparable number.

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

const benchTol = 0.2

func must[T any](v T, err error) T {
	if err != nil {
		panic(err)
	}
	return v
}

func benchRect(w *sketch.World, x0, y0, x1, y1 float64) *sketch.Sketch {
	s := must(w.CreateSketch(w.XY()))
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	must(s.Solve(context.Background()))
	return s
}

func benchBox() *decad.Body {
	w := sketch.NewWorld()
	s := benchRect(w, 0, 0, 40, 24)
	return must(decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(12), Dir: decad.Along}))
}

func benchFreeformPrism() *decad.Body {
	w := sketch.NewWorld()
	s := must(w.CreateSketch(w.XY()))
	a := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	e := s.CreatePoint(8, 0)
	must(s.CreateFitSpline(a, mid, e))
	s.CreateLine(e, a)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			prof = p
			break
		}
	}
	return must(decad.New().Extrude(s, prof, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}))
}

func benchRevolve(build func(s *sketch.Sketch), ext decad.AngularExtent) *decad.Body {
	w := sketch.NewWorld()
	s := must(w.CreateSketch(w.XY()))
	build(s)
	must(s.Solve(context.Background()))
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	return must(decad.New().Revolve(s, s.Profiles()[0], axis, ext))
}

func benchLoft() *decad.Body {
	w := sketch.NewWorld()
	s0 := benchRect(w, -20, -20, 20, 20)
	top := must(w.CreateOffsetPlane(w.XY(), 10))
	s1 := must(w.CreateSketch(top))
	tr := s1.CreateRectangle(-10, -10, 10, 10)
	s1.Fix(tr.A)
	must(s1.Solve(context.Background()))
	return must(decad.New().Loft(s0, s0.Profiles()[0], s1, s1.Profiles()[0]))
}

func benchCut() *decad.Body {
	w := sketch.NewWorld()
	doc := decad.New()
	plate := benchRect(w, 0, 0, 60, 40)
	pb := must(doc.Extrude(plate, plate.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along}))
	hs := must(w.CreateSketch(w.XY()))
	center := hs.CreatePoint(30, 20)
	hs.Fix(center)
	hs.CreateCircle(center, 8)
	must(hs.Solve(context.Background()))
	tool := must(doc.Extrude(hs, hs.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along}))
	return must(decad.Cut(pb, tool))
}

// runTess is the shared benchmark body: the body is built once, outside the
// timed loop, and Tessellate runs each iteration. A refusal is reported as a
// metric rather than a failure so a baseline tree that stages a payload class
// still yields a number.
func runTess(b *testing.B, body *decad.Body) {
	b.Helper()
	refused := 0.0
	var tris float64
	for b.Loop() {
		m, err := body.Tessellate(units.Millimeters(benchTol))
		if err != nil {
			if errors.Is(err, decad.ErrUnsupported) || errors.Is(err, decad.ErrDegenerate) {
				refused = 1
				continue
			}
			b.Fatal(err)
		}
		tris = float64(len(m.Triangles()))
	}
	b.ReportMetric(refused, "refused")
	b.ReportMetric(tris, "tris")
}

func BenchmarkTessPrismBox(b *testing.B) { runTess(b, benchBox()) }

func BenchmarkTessPrismFillet(b *testing.B) {
	body := must(benchBox().Fillet(decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(6)))
	runTess(b, body)
}

func BenchmarkTessPrismFreeform(b *testing.B) { runTess(b, benchFreeformPrism()) }

func BenchmarkTessCup(b *testing.B) {
	body := must(benchBox().Shell(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(1), units.Millimeters(4)))
	runTess(b, body)
}

func BenchmarkTessCapBlend(b *testing.B) {
	box := benchBox()
	body := must(box.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(box))), units.Millimeters(3)))
	runTess(b, body)
}

func BenchmarkTessLoft(b *testing.B) { runTess(b, benchLoft()) }

func BenchmarkTessFaceted(b *testing.B) { runTess(b, benchCut()) }

func BenchmarkTessRevolveLineFull(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		r := s.CreateRectangle(0, 4, 10, 8)
		s.Fix(r.A)
	}, decad.FullRevolution{})
	runTess(b, body)
}

func BenchmarkTessRevolveLinePartial(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		r := s.CreateRectangle(0, 4, 10, 8)
		s.Fix(r.A)
	}, decad.AngleExtent{A: units.Radians(1.5 * math.Pi), Dir: decad.Along})
	runTess(b, body)
}

func BenchmarkTessRevolveSphere(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		o := s.CreatePoint(0, 0)
		s.Fix(o)
		left := s.CreatePoint(-8, 0)
		right := s.CreatePoint(8, 0)
		s.CreateArc(o, right, left)
		s.CreateLine(left, right)
	}, decad.FullRevolution{})
	runTess(b, body)
}

func BenchmarkTessRevolveTorus(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	}, decad.FullRevolution{})
	runTess(b, body)
}

// A groove: z in [0, 18], rho in [4, 14], a semicircular groove of radius 3
// centred at z = 9 on the outer edge, walked so the torus wall is concave.
func benchGroove(ext decad.AngularExtent) *decad.Body {
	return benchRevolve(func(s *sketch.Sketch) {
		m := func(z, rho float64) *sketch.Point {
			p := s.CreatePoint(z, rho)
			s.Fix(p)
			return p
		}
		a := m(0, 4)
		bb := m(18, 4)
		c := m(18, 14)
		right := m(12, 14)
		left := m(6, 14)
		d := m(0, 14)
		centre := m(9, 14)
		s.CreateLine(a, bb)
		s.CreateLine(bb, c)
		s.CreateLine(c, right)
		s.CreateArc(centre, left, right)
		s.CreateLine(left, d)
		s.CreateLine(d, a)
	}, ext)
}

func BenchmarkTessRevolveGrooveFull(b *testing.B) { runTess(b, benchGroove(decad.FullRevolution{})) }

func BenchmarkTessRevolveGroovePartial(b *testing.B) {
	runTess(b, benchGroove(decad.AngleExtent{A: units.Radians(1.5 * math.Pi), Dir: decad.Along}))
}
