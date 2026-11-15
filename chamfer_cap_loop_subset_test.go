package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// capLoopSubsetBody extrudes the closed outline that draw sketches in the XY plane by 5 mm.
func capLoopSubsetBody(t *testing.T, draw func(s *sketch.Sketch)) *decad.Body {
	t.Helper()
	ctx := t.Context()
	w := sketch.NewWorld()
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			pr = p
		}
	}
	require.NotNil(t, pr)
	body, err := decad.Extrude(ctx, decad.New(), s, pr, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// topEdges selects top-face edges (z = 5): each clause names one edge by an endpoint and a direction.
type topEdge struct{ at, dir r3.Vec }

func topEdges(edges ...topEdge) *decad.EdgeQuery {
	var q *decad.EdgeQuery
	for _, e := range edges {
		preds := []decad.EdgePredicate{decad.EndpointAt(r3.Vec{X: e.at.X, Y: e.at.Y, Z: 5}), decad.ParallelTo(e.dir)}
		if q == nil {
			q = decad.Edges(preds...)
		} else {
			q = q.Or(preds...)
		}
	}
	return q
}

func volumeOf(t *testing.T, b *decad.Body) float64 {
	t.Helper()
	m, err := b.Volume()
	require.NoError(t, err)
	v, err := m.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	return v
}

// TestChamferCapLoopSubset chamfers runs of a cap loop's edges that end at sharp corners, as a CAD chamfer does.
func TestChamferCapLoopSubset(t *testing.T) {
	t.Parallel()
	const d = 0.5

	t.Run("two edges meeting at a corner", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(0, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10), s.CreatePoint(0, 10)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, e)
			s.CreateLine(e, a)
			s.Fix(a)
			s.Fix(b)
			s.Fix(c)
			s.Fix(e)
		})
		sel := topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}, topEdge{r3.Vec{}, r3.Vec{Y: 1}})
		out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		// Two wedges of d*d/2 by their lengths, less the pyramid d^3/3 they share at the mitred corner.
		want := 30*10*5 - (30+10)*d*d/2 + d*d*d/3
		require.InDelta(t, want, volumeOf(t, out), 1e-6)
	})

	t.Run("one edge of the loop", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(0, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10), s.CreatePoint(0, 10)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, e)
			s.CreateLine(e, a)
			s.Fix(a)
			s.Fix(b)
			s.Fix(c)
			s.Fix(e)
		})
		sel := topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}, topEdge{r3.Vec{}, r3.Vec{Y: 1}}, topEdge{r3.Vec{X: 30, Y: 10}, r3.Vec{X: 1}})
		out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		want := 30*10*5 - (30+10+30)*d*d/2 + 2*d*d*d/3
		require.InDelta(t, want, volumeOf(t, out), 1e-6)
	})

	// A long run chamfered next to a short run left sharp, with a 0.4 mm round inside the short run.
	t.Run("long run beside a short run with a round", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(2, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10), s.CreatePoint(0, 10)
			g, h := s.CreatePoint(0, 6), s.CreatePoint(2, 6)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, e)
			l1 := s.CreateLine(e, g)
			l2 := s.CreateLine(g, h)
			s.CreateLine(h, a)
			_, err := sketch.CreateFillet(s, l1, l2, g, 0.4)
			require.NoError(t, err)
			for _, p := range []*sketch.Point{a, b, c, e, h} {
				s.Fix(p)
			}
			s.Fix(g)
		})
		sel := topEdges(topEdge{r3.Vec{X: 2}, r3.Vec{X: 1}}, topEdge{r3.Vec{X: 30}, r3.Vec{Y: 1}},
			topEdge{r3.Vec{Y: 10}, r3.Vec{X: 1}})
		out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		before := volumeOf(t, body)
		got := volumeOf(t, out)
		// The three runs (28, 10 and 30 mm) lose d*d/2 each per millimetre, less the two mitred corners' pyramids.
		want := before - (28+10+30)*d*d/2 + 2*d*d*d/3
		// The round's facets hold the Cut's mesh within a few microns of its arc.
		require.InDelta(t, want, got, 5e-3)
	})

	// A run of two lines and an arc that continue one another, ending at two sharp corners.
	t.Run("run through an arc", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(0, 0), s.CreatePoint(25, 0), s.CreatePoint(25, 10), s.CreatePoint(0, 10)
			centre := s.CreatePoint(25, 5)
			s.CreateLine(a, b)
			s.CreateArc(centre, b, c)
			s.CreateLine(c, e)
			s.CreateLine(e, a)
			for _, p := range []*sketch.Point{a, b, c, e, centre} {
				s.Fix(p)
			}
		})
		before := volumeOf(t, body)
		sel := topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}, topEdge{r3.Vec{Y: 10}, r3.Vec{X: 1}}).
			Or(decad.EndpointAt(r3.Vec{X: 25, Z: 5}), decad.Circular())
		out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		want := before - (2*25*d*d/2 + math.Pi*(5-d/3)*d*d/2)
		// The Cut holds the arc's wall as chords inside its circle, about 2/3 of the chord tolerance over its area.
		require.InDelta(t, want, volumeOf(t, out), 0.05)
	})

	t.Run("both caps in one call", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(0, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10), s.CreatePoint(0, 10)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, e)
			s.CreateLine(e, a)
			for _, p := range []*sketch.Point{a, b, c, e} {
				s.Fix(p)
			}
		})
		sel := decad.Edges(decad.EndpointAt(r3.Vec{Z: 5}), decad.ParallelTo(r3.Vec{X: 1})).
			Or(decad.EndpointAt(r3.Vec{}), decad.ParallelTo(r3.Vec{X: 1}))
		out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		require.InDelta(t, 30*10*5-2*30*d*d/2, volumeOf(t, out), 1e-6)
	})

	t.Run("run that ends at a concave corner", func(t *testing.T) {
		t.Parallel()
		// An L outline: the edge from (30,10) to (10,10) ends at the reflex corner (10,10).
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			pts := []*sketch.Point{
				s.CreatePoint(0, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10),
				s.CreatePoint(10, 10), s.CreatePoint(10, 30), s.CreatePoint(0, 30),
			}
			for i, p := range pts {
				s.CreateLine(p, pts[(i+1)%len(pts)])
				s.Fix(p)
			}
		})
		_, err := decad.Chamfer(t.Context(), body, topEdges(topEdge{r3.Vec{X: 30, Y: 10}, r3.Vec{X: 1}}), units.Millimeters(d))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("run that stops inside a tangent chain", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e := s.CreatePoint(0, 0), s.CreatePoint(25, 0), s.CreatePoint(25, 10), s.CreatePoint(0, 10)
			centre := s.CreatePoint(25, 5)
			s.CreateLine(a, b)
			s.CreateArc(centre, b, c)
			s.CreateLine(c, e)
			s.CreateLine(e, a)
			for _, p := range []*sketch.Point{a, b, c, e, centre} {
				s.Fix(p)
			}
		})
		_, err := decad.Chamfer(t.Context(), body, topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}), units.Millimeters(d))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
