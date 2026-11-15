package decad_test

import (
	"fmt"
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
		before := volumeOf(t, body)
		out, err := decad.Chamfer(t.Context(), body, topEdges(topEdge{r3.Vec{X: 30, Y: 10}, r3.Vec{X: 1}}), units.Millimeters(d))
		require.NoError(t, err)
		require.True(t, out.IsSolid())
		// A 90 degree concave end stops the bevel on the next wall's plane, x = 10.
		require.InDelta(t, before-20*d*d/2, volumeOf(t, out), 1e-5)
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

// polyBody extrudes the closed polygon through pts, with every x scaled by sx, so that sx = -1 draws the mirror image.
func polyBody(t *testing.T, sx float64, pts [][2]float64) *decad.Body {
	t.Helper()
	return capLoopSubsetBody(t, func(s *sketch.Sketch) {
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(sx*p[0], p[1])
		}
		for i, p := range ps {
			s.CreateLine(p, ps[(i+1)%len(ps)])
			s.Fix(p)
		}
	})
}

// lineEdge selects the top-face edge from a to b of a polygon drawn with polyBody and the same sx.
func lineEdge(sx float64, a, b [2]float64) topEdge {
	dir := r3.Vec{X: sx * (b[0] - a[0]), Y: b[1] - a[1]}
	n, _ := dir.Normalize()
	return topEdge{r3.Vec{X: sx * a[0], Y: a[1]}, n}
}

// sectionAt reads the outline of body at depth w below the top face, with chord tolerance tol.
func sectionAt(t *testing.T, body *decad.Body, w, tol float64) decad.SectionOutline {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: 5 - w}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	out, err := body.SectionOutline(t.Context(), f, units.Millimeters(tol))
	require.NoError(t, err)
	return out
}

func nearestLoopPoint(o decad.SectionOutline, x, y float64) float64 {
	best := math.Inf(1)
	for _, loop := range o.Loops {
		for _, p := range loop {
			best = min(best, math.Hypot(p.X-x, p.Y-y))
		}
	}
	return best
}

// TestChamferCapLoopSubsetEnds chamfers runs whose ends meet their next edge at an acute convex corner or a concave
// corner, and runs chained across a sharp corner.
func TestChamferCapLoopSubsetEnds(t *testing.T) {
	t.Parallel()
	const d = 0.5
	cot := func(deg float64) float64 { return 1 / math.Tan(deg*math.Pi/180) }
	chamfer := func(t *testing.T, body *decad.Body, sel *decad.EdgeQuery) (*decad.Body, error) {
		t.Helper()
		return decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
	}

	for _, sx := range []float64{1, -1} {
		// An L outline: the run from (30,10) to (10,10) ends at the reflex corner (10,10), turned 90 degrees.
		t.Run(fmt.Sprintf("concave end at 90 degrees, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			pts := [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {10, 30}, {0, 30}}
			body := polyBody(t, sx, pts)
			out, err := chamfer(t, body, topEdges(lineEdge(sx, pts[2], pts[3])))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			// The bevel stops on the next wall's plane, which is the perpendicular at the corner.
			require.InDelta(t, volumeOf(t, body)-20*d*d/2, volumeOf(t, out), 1e-5)
		})

		// The next wall leans over the run's band, so the bevel stops on that wall's plane carried back over the band.
		// It loses d^3 cot(phi)/6 of the perpendicular end's wedge.
		t.Run(fmt.Sprintf("concave end at 45 degrees, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			pts := [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {0, 20}}
			body := polyBody(t, sx, pts)
			out, err := chamfer(t, body, topEdges(lineEdge(sx, pts[2], pts[3])))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			want := volumeOf(t, body) - 20*d*d/2 + d*d*d*cot(45)/6
			require.InDelta(t, want, volumeOf(t, out), 1e-4)
			// 0.25 mm below the top the bevel's inner edge lies on the next wall's plane x + y = 20 (mirrored).
			o := sectionAt(t, out, 0.25, 0.01)
			require.Less(t, nearestLoopPoint(o, sx*10.25, 9.75), 1e-3)
			require.Less(t, nearestLoopPoint(o, sx*10, 10), 1e-3)
		})

		// The bevel runs out onto the next wall, removing d^3 cot(theta)/6 more than the perpendicular end's wedge. The
		// chain's other end turns 108 degrees, which gives back d^3 cot(108)/6, and its two edges mitre at (0,0).
		t.Run(fmt.Sprintf("convex ends at 60 and 108 degrees, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			pts := [][2]float64{{0, 0}, {30, 0}, {35, 5 * math.Sqrt(3)}, {0, 20}}
			body := polyBody(t, sx, pts)
			out, err := chamfer(t, body, topEdges(lineEdge(sx, pts[3], pts[0]), lineEdge(sx, pts[0], pts[1])))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			want := volumeOf(t, body) - (50*d*d/2 - d*d*d/3 + d*d*d*(cot(60)+cot(108))/6)
			require.InDelta(t, want, volumeOf(t, out), 1e-4)
		})

		// A clockwise arc wall outside the selection does not hide the loop's other runs.
		t.Run(fmt.Sprintf("concave arc wall outside the selection, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(x, y float64) *sketch.Point { return s.CreatePoint(sx*x, y) }
				a, b, c, e, g, h := p(0, 0), p(30, 0), p(30, 20), p(20, 20), p(10, 20), p(0, 20)
				centre := p(15, 20)
				s.CreateLine(a, b)
				s.CreateLine(b, c)
				s.CreateLine(c, e)
				if sx > 0 {
					s.CreateArc(centre, g, e)
				} else {
					s.CreateArc(centre, e, g)
				}
				s.CreateLine(g, h)
				s.CreateLine(h, a)
				for _, q := range []*sketch.Point{a, b, c, e, g, h, centre} {
					s.Fix(q)
				}
			})
			// The bottom and left edges form a chain that mitres at the origin.
			out, err := chamfer(t, body, topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}, topEdge{r3.Vec{}, r3.Vec{Y: 1}}))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			// The notch's arc wall reads a little inside its circle in the Cut's mesh.
			require.InDelta(t, volumeOf(t, body)-(50*d*d/2-d*d*d/3), volumeOf(t, out), 0.05)
		})

		// A line and a clockwise arc meet at a sharp 90 degree corner, which the chain's one cutter mitres.
		t.Run(fmt.Sprintf("chain of a line and a concave arc, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(x, y float64) *sketch.Point { return s.CreatePoint(sx*x, y) }
				a, b, c, e, g := p(0, 0), p(20, 0), p(30, 10), p(30, 20), p(0, 20)
				centre := p(30, 0)
				s.CreateLine(a, b)
				if sx > 0 {
					s.CreateArc(centre, c, b)
				} else {
					s.CreateArc(centre, b, c)
				}
				s.CreateLine(c, e)
				s.CreateLine(e, g)
				s.CreateLine(g, a)
				for _, q := range []*sketch.Point{a, b, c, e, g, centre} {
					s.Fix(q)
				}
			})
			sel := topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}).Or(decad.EndpointAt(r3.Vec{X: sx * 20, Z: 5}), decad.Circular())
			out, err := chamfer(t, body, sel)
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			// The arc's band lies outside its circle: pi/2 of (R d^2/2 + d^3/6), less the pyramid at the mitre.
			want := volumeOf(t, body) - (20*d*d/2 + math.Pi/2*(10*d*d/2+d*d*d/6)) + d*d*d/3
			require.InDelta(t, want, volumeOf(t, out), 0.1)
		})

		// The shape of the one-way clutch hub's outline: a long run that holds a concave arc, a +90 degree start, a
		// concave end turned 22.49 degrees, and a second run that starts at 88.2 degrees.
		t.Run(fmt.Sprintf("hub-like outline, mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			const (
				r     = 8.0
				sweep = 30.0
				phi   = 22.49
				theta = 88.2
			)
			rad := func(deg float64) float64 { return deg * math.Pi / 180 }
			// The arc runs clockwise about (10,-r) from 90 to 60 degrees; a line leaves along its tangent.
			arcEnd := [2]float64{10 + r*math.Cos(rad(90-sweep)), -r + r*math.Sin(rad(90-sweep))}
			tan := -sweep
			v := [2]float64{arcEnd[0] + 12*math.Cos(rad(tan)), arcEnd[1] + 12*math.Sin(rad(tan))}
			tan -= phi
			n := [2]float64{v[0] + 15*math.Cos(rad(tan)), v[1] + 15*math.Sin(rad(tan))}
			tan += theta
			turnR1 := 90 - tan // the run turns up here
			r1 := [2]float64{n[0] + 20*math.Cos(rad(tan)), n[1] + 20*math.Sin(rad(tan))}
			top := [2]float64{r1[0], 30}
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(q [2]float64) *sketch.Point { return s.CreatePoint(sx*q[0], q[1]) }
				a, b, c, vp, np, rp, tp, lp := p([2]float64{0, 0}), p([2]float64{10, 0}), p(arcEnd), p(v), p(n), p(r1),
					p(top), p([2]float64{0, 30})
				centre := p([2]float64{10, -r})
				s.CreateLine(a, b)
				if sx > 0 {
					s.CreateArc(centre, c, b)
				} else {
					s.CreateArc(centre, b, c)
				}
				s.CreateLine(c, vp)
				s.CreateLine(vp, np)
				s.CreateLine(np, rp)
				s.CreateLine(rp, tp)
				s.CreateLine(tp, lp)
				s.CreateLine(lp, a)
				for _, q := range []*sketch.Point{a, b, c, vp, np, rp, tp, lp, centre} {
					s.Fix(q)
				}
			})
			sel := topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}},
				lineEdge(sx, arcEnd, v), lineEdge(sx, n, r1)).
				Or(decad.EndpointAt(r3.Vec{X: sx * 10, Z: 5}), decad.Circular())
			out, err := chamfer(t, body, sel)
			require.NoError(t, err)
			require.True(t, out.IsSolid())

			// 0.25 mm below the top the bevel is 0.25 mm wide. Each run removes its length times that, the arc its
			// sector, the concave end gives back d^2 cot(phi)/2 and each acute convex end adds d^2 cot(turn)/2.
			const w, band = 0.25, d - 0.25
			before, after := sectionAt(t, body, w, 0.0005), sectionAt(t, out, w, 0.01)
			removed := (10+12+20)*band + math.Pi/6*(r*band+band*band/2) - band*band*cot(phi)/2 +
				band*band*(cot(theta)+cot(turnR1))/2
			got := before.PolygonArea().Base() - after.PolygonArea().Base()
			// The held mesh reads within about 0.005 mm of the arc's wall, so the areas agree to a few hundredths.
			require.InDelta(t, removed, got, 0.05)

			// The bevel's inner edge ends on the next wall's plane carried back over the band at the concave end, and on
			// the next wall at the acute convex end.
			endPoint := func(corner [2]float64, travelDeg, turnDeg float64, sign float64) {
				tr := [2]float64{math.Cos(rad(travelDeg)), math.Sin(rad(travelDeg))}
				in := [2]float64{-tr[1], tr[0]}
				x := corner[0] + band*(sign*cot(turnDeg)*tr[0]+in[0])
				y := corner[1] + band*(sign*cot(turnDeg)*tr[1]+in[1])
				require.Less(t, nearestLoopPoint(after, sx*x, y), 0.01)
			}
			endPoint(v, -sweep, phi, -1)
			endPoint(r1, tan, turnR1, 1)
		})
	}

	refusals := []struct {
		name string
		pts  [][2]float64
		from int // the selected edge runs from pts[from] to pts[from+1]
	}{
		{"convex end at 30 degrees", [][2]float64{{0, 0}, {30, 0}, {30 + 10*math.Cos(math.Pi/6), 5}, {0, 20}}, 0},
		{"concave end at 120 degrees", [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {15, 10 + 5*math.Sqrt(3)}, {0, 20}}, 2},
		{"run-out longer than the next edge", [][2]float64{{0, 0}, {30, 0}, {30.5, 0.5 * math.Sqrt(3)}, {0, 20}}, 0},
	}
	for _, rc := range refusals {
		t.Run("refuses "+rc.name, func(t *testing.T) {
			t.Parallel()
			body := polyBody(t, 1, rc.pts)
			sel := topEdges(lineEdge(1, rc.pts[rc.from], rc.pts[rc.from+1]))
			if rc.from == 0 {
				// The chain also holds the closing edge, which mitres with the first at the origin.
				sel = sel.Or(decad.EndpointAt(r3.Vec{Z: 5}), decad.ParallelTo(r3.Vec{Y: 1}))
			}
			_, err := chamfer(t, body, sel)
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}

	t.Run("refuses a concave end beside an arc", func(t *testing.T) {
		t.Parallel()
		body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
			a, b, c, e, g := s.CreatePoint(0, 0), s.CreatePoint(30, 0), s.CreatePoint(30, 10), s.CreatePoint(10, 10),
				s.CreatePoint(0, 20)
			centre := s.CreatePoint(0, 10)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, e)
			s.CreateArc(centre, e, g)
			s.CreateLine(g, a)
			for _, q := range []*sketch.Point{a, b, c, e, g, centre} {
				s.Fix(q)
			}
		})
		_, err := chamfer(t, body, topEdges(topEdge{r3.Vec{X: 30, Y: 10}, r3.Vec{X: 1}}))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
