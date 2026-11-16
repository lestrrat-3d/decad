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

// keyhole is a 20 x 20 plate whose bottom edge opens, through a slot of width 2w, into a bore of radius r about the
// origin. A positive flat is the y of a chord across the bore's top, which makes the bore a D.
type keyhole struct{ r, w, flat float64 }

func (k keyhole) footY() float64 { return math.Sqrt(k.r*k.r - k.w*k.w) }

// body extrudes the plate 5 mm, with every x scaled by sx, so that sx = -1 draws the mirror image.
func (k keyhole) body(t *testing.T, sx float64) *decad.Body {
	t.Helper()
	return capLoopSubsetBody(t, func(s *sketch.Sketch) {
		p := func(x, y float64) *sketch.Point { return s.CreatePoint(sx*x, y) }
		// arc draws the counter-clockwise arc about the origin from a to b, as drawn before mirroring.
		centre := p(0, 0)
		arc := func(a, b *sketch.Point) {
			if sx > 0 {
				s.CreateArc(centre, a, b)
			} else {
				s.CreateArc(centre, b, a)
			}
		}
		y0 := k.footY()
		a, b, c, e, f, g, h, i := p(-10, -10), p(-k.w, -10), p(-k.w, -y0), p(k.w, -y0), p(k.w, -10), p(10, -10), p(10, 10),
			p(-10, 10)
		pts := []*sketch.Point{a, b, c, e, f, g, h, i, centre}
		s.CreateLine(a, b)
		s.CreateLine(b, c)
		if k.flat > 0 {
			qx := math.Sqrt(k.r*k.r - k.flat*k.flat)
			q2, q1 := p(qx, k.flat), p(-qx, k.flat)
			arc(e, q2)
			s.CreateLine(q2, q1)
			arc(q1, c)
			pts = append(pts, q2, q1)
		} else {
			arc(e, c)
		}
		s.CreateLine(e, f)
		s.CreateLine(f, g)
		s.CreateLine(g, h)
		s.CreateLine(h, i)
		s.CreateLine(i, a)
		for _, q := range pts {
			s.Fix(q)
		}
	})
}

// rim selects the top face's edges of the bore's rim: its arcs, and the flat of a D.
func (k keyhole) rim(sx float64) *decad.EdgeQuery {
	y0 := k.footY()
	sel := decad.Edges(decad.EndpointAt(r3.Vec{X: sx * k.w, Y: -y0, Z: 5}), decad.Circular())
	if k.flat == 0 {
		// A mesh Boolean result may hold the bore's circle as two arcs, split where its recorded seam lies.
		return sel.Or(decad.EndpointAt(r3.Vec{X: sx * k.r, Z: 5}), decad.Circular())
	}
	qx := math.Sqrt(k.r*k.r - k.flat*k.flat)
	return sel.Or(decad.EndpointAt(r3.Vec{X: sx * -qx, Y: k.flat, Z: 5}), decad.Circular()).
		Or(decad.EndpointAt(r3.Vec{X: sx * qx, Y: k.flat, Z: 5}), decad.ParallelTo(r3.Vec{X: 1}))
}

// removed is the volume a CAD chamfer of setback d takes from the bore's rim. At setback s = d - z the section of the
// removed region is the disc of radius r + s below y = f + s (the whole disc for a round bore), less the bore and less
// the slot, which the chamfer does not reach. The mitre at the flat's corners follows from the section.
func (k keyhole) removed(d float64) float64 {
	strip := func(rad float64) float64 { // the disc of radius rad inside |x| < w
		return 2 * (k.w*math.Sqrt(rad*rad-k.w*k.w) + rad*rad*math.Asin(k.w/rad))
	}
	below := func(rad, h float64) float64 { // the disc of radius rad below y = h
		return rad*rad*(math.Pi-math.Acos(h/rad)) + h*math.Sqrt(rad*rad-h*h)
	}
	section := func(s float64) float64 {
		slot := (strip(k.r+s) - strip(k.r)) / 2
		if k.flat == 0 {
			return math.Pi*((k.r+s)*(k.r+s)-k.r*k.r) - slot
		}
		return below(k.r+s, k.flat+s) - below(k.r, k.flat) - slot
	}
	// Simpson's rule on a smooth integrand.
	const n = 2000
	sum := section(0) + section(d)
	for i := 1; i < n; i++ {
		w := 2.0
		if i%2 == 1 {
			w = 4
		}
		sum += w * section(d*float64(i)/n)
	}
	return sum * d / n / 3
}

// checkSections reads the chamfered keyhole at depths 0.1, 0.5 and 0.9 of the setback below the top: near the bore every
// point lies on the bevel's section, and the bevel meets each slot wall where the cone crosses the wall's plane.
func (k keyhole) checkSections(t *testing.T, out *decad.Body, sx, d float64) {
	t.Helper()
	for _, frac := range []float64{0.1, 0.5, 0.9} {
		z := frac * d
		o := sectionAt(t, out, z, 0.002)
		off, err := o.OffSurface.In(units.Millimeter)
		require.NoError(t, err)
		tol := off + 1e-3
		s := d - z
		rad := k.r + s
		h := math.Inf(1)
		if k.flat > 0 {
			h = k.flat + s
		}
		near := 0
		for _, loop := range o.Loops {
			for _, p := range loop {
				x, y := sx*p.X, p.Y
				if math.Hypot(x, y) > k.r+d+0.4 || math.Abs(x) < k.w+1e-3 {
					continue
				}
				near++
				// A point of the bore's bevel lies on the circle of radius r + s, or the flat at f + s, whichever binds.
				require.InDelta(t, 0, max(math.Hypot(x, y)-rad, y-h), tol, "depth %v at %v,%v", z, x, y)
			}
		}
		require.Greater(t, near, 8)
		// The slot is void: no outline point lies inside it, so the bevel runs out on the walls and nowhere else.
		for _, loop := range o.Loops {
			for _, p := range loop {
				if math.Hypot(p.X, p.Y) < k.r+d+0.4 && p.Y < 0 {
					require.GreaterOrEqual(t, math.Abs(p.X), k.w-tol, "depth %v: outline point %v,%v inside the slot", z, p.X, p.Y)
				}
			}
		}
		wall := -math.Sqrt(rad*rad - k.w*k.w)
		for _, side := range []float64{-1, 1} {
			require.Less(t, nearestLoopPoint(o, sx*side*k.w, wall), tol, "depth %v, wall %v", z, side)
		}
	}
}

// TestChamferKeyholeRimRun chamfers the run of a bore's rim that ends on the walls of a slot opening out of it: the
// bevel runs out onto each wall where the cone crosses the wall's plane.
func TestChamferKeyholeRimRun(t *testing.T) {
	t.Parallel()
	for _, k := range []keyhole{{4, 0.5, 0}, {2.55, 0.5, 0}, {4, 0.5, 3}, {4.05, 0.5, 3.5}, {2.55, 0.6, 1.5}} {
		for _, d := range []float64{0.3, 0.5} {
			for _, sx := range []float64{1, -1} {
				t.Run(fmt.Sprintf("r %v w %v flat %v d %v mirror %v", k.r, k.w, k.flat, d, sx), func(t *testing.T) {
					t.Parallel()
					body := k.body(t, sx)
					out, err := decad.Chamfer(t.Context(), body, k.rim(sx), units.Millimeters(d))
					require.NoError(t, err)
					require.True(t, out.IsSolid())
					m, err := out.Volume()
					require.NoError(t, err)
					bound, err := m.Bound.In(units.CubicMillimeter)
					require.NoError(t, err)
					// The chamfer holds the bore's wall as chords, which the bound covers.
					require.InDelta(t, k.removed(d), volumeOf(t, body)-volumeOf(t, out), bound+1e-6)
					// The cutter's departure from the bevel is charged in the bound: it is a small fraction of a cubic
					// millimetre here, not a free pass.
					require.Less(t, bound, 0.1)
					k.checkSections(t, out, sx, d)
				})
			}
		}
	}
}

// A slot narrower than the arc's run-out lets one end's cutter reach the material across the slot.
func TestChamferKeyholeRimRunRefusesNarrowSlot(t *testing.T) {
	t.Parallel()
	k := keyhole{4, 0.1, 0}
	body := k.body(t, 1)
	_, err := decad.Chamfer(t.Context(), body, k.rim(1), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestChamferConvexArcRunOutOnNextWall chamfers an arc that two sharp corners turn by 70 degrees each: the bevel runs
// out onto both neighbouring walls where the cone crosses their planes, which a tangent extension misses.
func TestChamferConvexArcRunOutOnNextWall(t *testing.T) {
	t.Parallel()
	const (
		d    = 0.5
		r    = 6.0
		turn = 70.0
	)
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	// The arc leaves (20,0) turned 70 degrees from the bottom edge, sweeps 60 degrees and meets a line turned 70 degrees
	// more, leaving at 200 degrees.
	p1 := [2]float64{20, 0}
	cx, cy := p1[0]-r*math.Sin(rad(turn)), p1[1]+r*math.Cos(rad(turn))
	at := func(deg float64) [2]float64 { return [2]float64{cx + r*math.Cos(rad(deg)), cy + r*math.Sin(rad(deg))} }
	a0, a1 := turn-90, turn-90+60
	p2 := at(a1)
	dir := [2]float64{math.Cos(rad(a1 + 90 + turn)), math.Sin(rad(a1 + 90 + turn))}
	p3 := [2]float64{p2[0] + 8*dir[0], p2[1] + 8*dir[1]}
	for _, sx := range []float64{1, -1} {
		t.Run(fmt.Sprintf("mirror %v", sx), func(t *testing.T) {
			t.Parallel()
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(q [2]float64) *sketch.Point { return s.CreatePoint(sx*q[0], q[1]) }
				o, b, e, f, g, c := p([2]float64{0, 0}), p(p1), p(p2), p(p3), p([2]float64{0, 12}), p([2]float64{cx, cy})
				s.CreateLine(o, b)
				if sx > 0 {
					s.CreateArc(c, b, e)
				} else {
					s.CreateArc(c, e, b)
				}
				s.CreateLine(e, f)
				s.CreateLine(f, g)
				s.CreateLine(g, o)
				for _, q := range []*sketch.Point{o, b, e, f, g, c} {
					s.Fix(q)
				}
			})
			sel := decad.Edges(decad.EndpointAt(r3.Vec{X: sx * p1[0], Y: p1[1], Z: 5}), decad.Circular())
			out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			for _, z := range []float64{0.1, 0.25, 0.4} {
				o := sectionAt(t, out, z, 0.002)
				off, err := o.OffSurface.In(units.Millimeter)
				require.NoError(t, err)
				tol := off + 1e-3
				rho := r - (d - z)
				// The bevel's contour is the arc's circle moved in by d - z.
				mid := rad((a0 + a1) / 2) // the middle of the arc
				require.Less(t, nearestLoopPoint(o, sx*(cx+rho*math.Cos(mid)), cy+rho*math.Sin(mid)), tol)
				// It meets the bottom wall (y = 0) where the moved circle crosses it, past the arc's end at (20,0).
				xb := cx + math.Sqrt(rho*rho-cy*cy)
				require.Less(t, nearestLoopPoint(o, sx*xb, 0), tol, "depth %v", z)
				// And it meets the next wall, the line through p2 along dir, where the moved circle crosses that line.
				f0 := [2]float64{p2[0] - cx, p2[1] - cy}
				bq := f0[0]*dir[0] + f0[1]*dir[1]
				disc := bq*bq - (f0[0]*f0[0] + f0[1]*f0[1] - rho*rho)
				require.Greater(t, disc, 0.0)
				tt := -bq - math.Sqrt(disc) // the line heads into the circle, so both roots are ahead of p2
				require.Greater(t, tt, 0.0)
				require.Less(t, nearestLoopPoint(o, sx*(p2[0]+tt*dir[0]), p2[1]+tt*dir[1]), tol, "depth %v", z)
			}
		})
	}
}

// A convex arc small enough that its bevel's circle at the cap never reaches the next wall has no run-out: the cutter
// would carve a sector from the wall's material and leave a step. The layout is TestChamferConvexArcRunOutOnNextWall's.
func TestChamferConvexArcRunOutRefusesASmallArc(t *testing.T) {
	t.Parallel()
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	// The bevel's circle at the cap has radius r - d, and the next wall lies r cos(turn) from the arc's centre.
	for _, c := range []struct{ r, turn, d float64 }{{1.2, 45, 0.5}, {0.9, 60, 0.5}} {
		require.Less(t, c.r-c.d, c.r*math.Cos(rad(c.turn)))
		t.Run(fmt.Sprintf("r %v turn %v", c.r, c.turn), func(t *testing.T) {
			t.Parallel()
			p1 := [2]float64{20, 0}
			cx, cy := p1[0]-c.r*math.Sin(rad(c.turn)), p1[1]+c.r*math.Cos(rad(c.turn))
			a1 := c.turn - 90 + 60
			p2 := [2]float64{cx + c.r*math.Cos(rad(a1)), cy + c.r*math.Sin(rad(a1))}
			dir := [2]float64{math.Cos(rad(a1 + 90 + c.turn)), math.Sin(rad(a1 + 90 + c.turn))}
			p3 := [2]float64{p2[0] + 8*dir[0], p2[1] + 8*dir[1]}
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(q [2]float64) *sketch.Point { return s.CreatePoint(q[0], q[1]) }
				o, b, e, f, g, cc := p([2]float64{0, 0}), p(p1), p(p2), p(p3), p([2]float64{0, 12}), p([2]float64{cx, cy})
				s.CreateLine(o, b)
				s.CreateArc(cc, b, e)
				s.CreateLine(e, f)
				s.CreateLine(f, g)
				s.CreateLine(g, o)
				for _, q := range []*sketch.Point{o, b, e, f, g, cc} {
					s.Fix(q)
				}
			})
			sel := decad.Edges(decad.EndpointAt(r3.Vec{X: p1[0], Y: p1[1], Z: 5}), decad.Circular())
			_, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(c.d))
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

// The keyhole's slot holds a thin peninsula of material, 0.02 mm wide, rising from its floor to 0.3 mm below the bore,
// inside the cutter's end and between the samples a grid of points would take. A CAD chamfer of the rim leaves it alone,
// and the route refuses a chamfer whose cutter would cut it.
func TestChamferKeyholeRimRunRefusesAThinPeninsula(t *testing.T) {
	t.Parallel()
	const r, w, d = 4.0, 0.5, 0.5
	y0 := math.Sqrt(r*r - w*w)
	for _, pen := range [][2]float64{{0.31, 0.33}} {
		t.Run(fmt.Sprintf("peninsula x %v to %v", pen[0], pen[1]), func(t *testing.T) {
			t.Parallel()
			const top = -4.3
			body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
				p := func(x, y float64) *sketch.Point { return s.CreatePoint(x, y) }
				centre := p(0, 0)
				pts := []*sketch.Point{
					p(-10, -10), p(pen[1], -10), p(pen[1], top), p(pen[0], top), p(pen[0], -8), p(-w, -8),
					p(-w, -y0), p(w, -y0), p(w, -10), p(10, -10), p(10, 10), p(-10, 10),
				}
				for i := range pts {
					if i == 6 {
						// Counter-clockwise from the right wall to the left one, the long way over the top.
						s.CreateArc(centre, pts[7], pts[6])
						continue
					}
					s.CreateLine(pts[i], pts[(i+1)%len(pts)])
				}
				for _, q := range append(pts, centre) {
					s.Fix(q)
				}
			})
			sel := decad.Edges(decad.EndpointAt(r3.Vec{X: w, Y: -y0, Z: 5}), decad.Circular())
			_, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

// A tab 0.03 mm thick stands off the wall past the end of a line run, between the strip's old sample rows.
func TestChamferLineRunRefusesAThinTabPastItsEnd(t *testing.T) {
	t.Parallel()
	body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
		p := func(x, y float64) *sketch.Point { return s.CreatePoint(x, y) }
		pts := []*sketch.Point{p(0, 0), p(30, 0), p(30, 0.26), p(31, 0.26), p(31, 0.29), p(30, 0.29), p(30, 10), p(0, 10)}
		for i, q := range pts {
			s.CreateLine(q, pts[(i+1)%len(pts)])
			s.Fix(q)
		}
	})
	_, err := decad.Chamfer(t.Context(), body, topEdges(topEdge{r3.Vec{}, r3.Vec{X: 1}}), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// The two run-out ends of one arc, uncapped, would reach across the slot and overlap: each stops the margin past its
// wall instead, so setbacks above the slot's width build as the CAD chamfer does.
func TestChamferKeyholeRimRunWithAWideSetback(t *testing.T) {
	t.Parallel()
	for _, k := range []keyhole{{2.55, 0.5, 0}, {4, 0.5, 0}, {2.55, 0.5, 1.5}} {
		for _, d := range []float64{0.55, 0.8} {
			t.Run(fmt.Sprintf("r %v flat %v d %v", k.r, k.flat, d), func(t *testing.T) {
				t.Parallel()
				body := k.body(t, 1)
				out, err := decad.Chamfer(t.Context(), body, k.rim(1), units.Millimeters(d))
				require.NoError(t, err)
				m, err := out.Volume()
				require.NoError(t, err)
				bound, err := m.Bound.In(units.CubicMillimeter)
				require.NoError(t, err)
				require.InDelta(t, k.removed(d), volumeOf(t, body)-volumeOf(t, out), bound+1e-6)
				k.checkSections(t, out, 1, d)
			})
		}
	}
}
