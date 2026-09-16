package decadtest

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Sketch returns an empty sketch on a fresh sketch.World's standard XY datum
// plane. The world is private to the returned sketch, so two calls never
// share geometry. It fails tb rather than returning an error.
func Sketch(tb testing.TB) *sketch.Sketch {
	tb.Helper()

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		tb.Fatalf("decadtest.Sketch: creating a sketch on the world XY plane failed: %v", err)
		return nil
	}
	return s
}

// Region solves s and returns its single valid closed region. s MUST NOT be
// nil. Validity is sketch's own verdict (sketch.Profile.Valid), consumed and
// never re-derived: decad's hard rule that a 2D answer is asked of sketch,
// never recomputed. The solve runs with tb.Context(), so a test that ends
// cancels it. Region fails tb, rather than returning an error, when s solves
// to any count of valid regions other than exactly one.
func Region(tb testing.TB, s *sketch.Sketch) *sketch.Profile {
	tb.Helper()

	if s == nil {
		tb.Fatalf("decadtest.Region: s must not be nil")
		return nil
	}

	if _, err := s.Solve(tb.Context()); err != nil {
		tb.Fatalf("decadtest.Region: solving the sketch failed: %v", err)
		return nil
	}

	profiles := s.Profiles()
	var valid []*sketch.Profile
	for _, p := range profiles {
		if p.Valid {
			valid = append(valid, p)
		}
	}

	if len(valid) != 1 {
		tb.Fatalf("decadtest.Region: the sketch holds %d valid region(s), want exactly 1 (of %d detected)", len(valid), len(profiles))
		return nil
	}
	return valid[0]
}

// Prism extrudes p on s by height, one-sided along the sketch plane's
// normal. doc, s and p MUST NOT be nil. The extent is
// decad.Distance{D: height, Dir: decad.Along}, and height is a units.Value
// of Kind Length; a wrong Kind is decad's own rejection (decad.ErrUnitKind),
// surfaced through the tb.Fatalf below. Prism fails tb, rather than
// returning an error, on a nil argument or an extrusion failure.
func Prism(tb testing.TB, doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, height units.Value) *decad.Body {
	tb.Helper()

	if doc == nil {
		tb.Fatalf("decadtest.Prism: doc must not be nil")
		return nil
	}
	if s == nil {
		tb.Fatalf("decadtest.Prism: s must not be nil")
		return nil
	}
	if p == nil {
		tb.Fatalf("decadtest.Prism: p must not be nil")
		return nil
	}

	body, err := doc.Extrude(s, p, decad.Distance{D: height, Dir: decad.Along})
	if err != nil {
		tb.Fatalf("decadtest.Prism: extruding by %s failed: %v", height, err)
		return nil
	}
	return body
}

// Block builds a rectangle on a fresh sketch's XY plane between corners
// (x0, y0) and (x1, y1), grounds corner A with sketch.Sketch.Fix so the
// rectangle solves fully constrained, and extrudes it by height. The four
// coordinates are millimetres, the docs/api-design.md §5.2 coordinate
// carve-out, which is why they are float64 rather than a units.Value; height
// is a scalar quantity, so §5.1 admits no bare float for it and it stays a
// units.Value. Block delegates to Sketch, Region and Prism and performs no
// nil check of its own — Prism owns the doc check. The result is a straight
// prism recorded as one decad.OpExtrude step, the fixture most tests start
// from.
func Block(tb testing.TB, doc *decad.Document, x0, y0, x1, y1 float64, height units.Value) *decad.Body {
	tb.Helper()

	s := Sketch(tb)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	p := Region(tb, s)
	return Prism(tb, doc, s, p, height)
}
