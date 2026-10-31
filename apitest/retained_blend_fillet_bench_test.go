package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// BenchmarkFacetedRetainedBlendFourSideFillet measures the four-edge fillet
// from TestFacetedRetainedBlendAreaCharge. The consumed scene is rebuilt while
// timing is stopped on each iteration.
func BenchmarkFacetedRetainedBlendFourSideFillet(b *testing.B) {
	for range b.N {
		b.StopTimer()
		sc := newRetentionScene()
		plate := benchRetainedRect(b, sc, 0, 0, 80, 60, 10)
		body, err := decad.Union(b.Context(), plate,
			benchRetainedPlaced(b, benchRetainedRect(b, sc, 30, 20, 50, 40, 15), 10))
		if err != nil {
			b.Fatal(err)
		}
		top := decad.CapEnd(plate)
		body, err = decad.Hole(b.Context(), body, decad.HoleSpec{Face: decad.Faces(decad.FaceCreatedBy(top)),
			At: r3.NewVec(15, 15, 10), Diameter: units.Millimeters(6.6), Depth: decad.ThroughHole{},
			Head: decad.Counterbore{Diameter: units.Millimeters(11), Depth: units.Millimeters(6.4)}})
		if err != nil {
			b.Fatal(err)
		}
		s, err := sc.w.CreateSketch(sc.w.XY())
		if err != nil {
			b.Fatal(err)
		}
		c := s.CreatePoint(0, 0)
		s.Fix(c)
		s.CreateCircle(c, 6)
		if _, err = s.Solve(b.Context()); err != nil {
			b.Fatal(err)
		}
		post, err := decad.Extrude(b.Context(), sc.doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
		if err != nil {
			b.Fatal(err)
		}
		move, err := r3.Translation(r3.Vec{X: 65, Y: 45, Z: 10})
		if err != nil {
			b.Fatal(err)
		}
		post, err = decad.Transform(b.Context(), post, move)
		if err != nil {
			b.Fatal(err)
		}
		body, err = decad.Union(b.Context(), body, post)
		if err != nil {
			b.Fatal(err)
		}
		var selector *decad.EdgeQuery
		for _, corner := range [][2]float64{{30, 20}, {50, 20}, {50, 40}, {30, 40}} {
			want := r3.NewVec(corner[0], corner[1], 25)
			var at *r3.Vec
			for _, vertex := range body.Vertices() {
				pos := vertex.Position().Value
				if pos.Sub(want).Len() < 1e-3 {
					at = &pos
					break
				}
			}
			if at == nil {
				b.Fatalf("no boss vertex near %v", want)
			}
			preds := []decad.EdgePredicate{decad.EndpointAt(*at), decad.ParallelTo(r3.NewVec(0, 0, 1))}
			if selector == nil {
				selector = decad.Edges(preds...)
				continue
			}
			selector = selector.Or(preds...)
		}
		selector = selector.Exactly(4)
		b.StartTimer()
		_, err = decad.Fillet(b.Context(), body, selector, units.Millimeters(2))
		b.StopTimer()
		if err != nil {
			b.Fatal(err)
		}
	}
}

func benchRetainedRect(b *testing.B, sc *retentionScene, x0, y0, x1, y1, h float64) *decad.Body {
	s, err := sc.w.CreateSketch(sc.w.XY())
	if err != nil {
		b.Fatal(err)
	}
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	if _, err = s.Solve(b.Context()); err != nil {
		b.Fatal(err)
	}
	out, err := decad.Extrude(b.Context(), sc.doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	if err != nil {
		b.Fatal(err)
	}
	return out
}
func benchRetainedPlaced(b *testing.B, body *decad.Body, dz float64) *decad.Body {
	tr, err := r3.Translation(r3.Vec{Z: dz})
	if err != nil {
		b.Fatal(err)
	}
	out, err := decad.Transform(b.Context(), body, tr)
	if err != nil {
		b.Fatal(err)
	}
	return out
}
