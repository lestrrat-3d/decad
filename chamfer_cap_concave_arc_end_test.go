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

// TestChamferConcaveArcEndHoldsItsBound chamfers, on both caps, a chain of a convex arc and a line that starts at a
// concave corner turned a few degrees beside a line, the next wall, and turns a sharp 90 degree corner from the arc
// into the line. The end clip stops the bevel short of the concave corner, so the cutter never lies flush with the next
// wall, and the Cut needs no gap shift. A gap shift would move the clip face across the arc's wall at the corner's
// turn, where the result's bound divides the wall's chord bound by the sine of the turn. On the outline of radius 14.5
// turned 15 degrees, the layered cutter's fewest layers at a setback of 0.19 mm are seven, which put a ring on the
// wall's level; the cutter takes the next count that does not, so the bottom cap's cells beside the corner cross the
// wall instead of touching it. It is the smallest outline found that takes seven layers and, without the rule, a gap
// shift.
func TestChamferConcaveArcEndHoldsItsBound(t *testing.T) {
	t.Parallel()
	for _, c := range []struct {
		r, turn float64 // the arc's radius about the origin, and the concave corner's turn in degrees
		ds      []float64
	}{
		// The next wall's line passes 14 cos(10 degrees) = 13.79 from the centre, so it caps the bevel up to a setback
		// of 0.21 mm.
		{14, 10, []float64{0.2121, 0.2, 0.19, 0.15, 0.12}},
		{14.5, 15, []float64{0.19}},
	} {
		for _, d := range c.ds {
			t.Run(fmt.Sprintf("radius %v turn %v setback %v", c.r, c.turn, d), func(t *testing.T) {
				t.Parallel()
				chamferConcaveArcEnd(t, c.r, c.turn, d)
			})
		}
	}
}

func chamferConcaveArcEnd(t *testing.T, r, turn, d float64) {
	t.Helper()
	const (
		from  = 200.0 // the arc's start, the concave corner, in degrees
		to    = 300.0 // the arc's end, where the line turns inward
		inner = 11.0  // where the inward line ends
	)
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	polar := func(rho, deg float64) [2]float64 {
		return [2]float64{rho * math.Cos(rad(deg)), rho * math.Sin(rad(deg))}
	}
	v, w, p := polar(r, from), polar(r, to), polar(inner, to)
	// The next wall arrives at v along the arc's tangent turned toward the centre.
	u := [2]float64{math.Cos(rad(from + 90 + turn)), math.Sin(rad(from + 90 + turn))}
	a := [2]float64{v[0] - 3*u[0], v[1] - 3*u[1]}
	// The chain's other end turns 60 degrees, from the inward line onto a line along -X. The outline then reaches up to
	// y = 20, so the pair's chord tolerance, a fixed share of its diameter, passes the cutter's bound.
	q, f := [2]float64{p[0] - 10, p[1]}, [2]float64{p[0] - 10, 20}
	body := capLoopSubsetBody(t, func(s *sketch.Sketch) {
		pt := func(c [2]float64) *sketch.Point { return s.CreatePoint(c[0], c[1]) }
		ap, vp, wp, pp, qp, fp, o := pt(a), pt(v), pt(w), pt(p), pt(q), pt(f), pt([2]float64{})
		s.CreateLine(ap, vp)
		s.CreateArc(o, vp, wp)
		s.CreateLine(wp, pp)
		s.CreateLine(pp, qp)
		s.CreateLine(qp, fp)
		s.CreateLine(fp, ap)
		for _, x := range []*sketch.Point{ap, vp, wp, pp, qp, fp, o} {
			s.Fix(x)
		}
	})
	inward := r3.Vec{X: p[0] - w[0], Y: p[1] - w[1]}
	sel := topEdges(topEdge{r3.Vec{X: w[0], Y: w[1]}, inward}).
		Or(decad.EndpointAt(r3.Vec{X: v[0], Y: v[1], Z: 5}), decad.Circular()).
		Or(decad.EndpointAt(r3.Vec{X: w[0], Y: w[1]}), decad.ParallelTo(inward)).
		Or(decad.EndpointAt(r3.Vec{X: v[0], Y: v[1]}), decad.Circular())
	out, err := decad.Chamfer(t.Context(), body, sel, units.Millimeters(d))
	require.NoError(t, err)
	require.True(t, out.IsSolid())
	for _, diag := range out.OperationDiagnostics() {
		require.NotEqual(t, decad.OpDiagGapShiftApplied, diag.Code)
	}
	// The box's bound reads about 0.006 mm here, and about 0.02 mm after a gap shift on this outline.
	box, err := out.Bounds()
	require.NoError(t, err)
	require.Less(t, box.Bound.Base(), 0.01)

	// Below the top by h the outline follows the next wall's line from the corner to the bevel's top circle, radius
	// r - d + h, at the line's first crossing lambda1 = cu - sqrt((r - d + h)^2 - D^2), D and cu the distances from
	// the centre to the line and from the foot of the centre to the corner along it.
	D, cu := r*math.Cos(rad(turn)), r*math.Sin(rad(turn))
	for _, h := range []float64{d / 4, d / 2} {
		o := sectionAt(t, out, h, 0.01)
		off, err := o.OffSurface.In(units.Millimeter)
		require.NoError(t, err)
		lam := cu - math.Sqrt((r-d+h)*(r-d+h)-D*D)
		require.Less(t, nearestLoopPoint(o, v[0]+lam*u[0], v[1]+lam*u[1]), off+1e-3, "next wall at depth %v", h)
	}
}

// TestChamferConcaveEndKeepsItsRelief chamfers a line run that ends at a concave corner, turned 90 or 45 degrees, in
// documents whose gap tolerance does and does not reach the corner relief. With the relief the result is the relieved
// chamfer: within its volume bound, its volume is the volume of the chamfer whose bevel stops on the next wall's plane
// plus between zero and 0.004^2 turn d / 2, the material the relief keeps. The chamfer's flush Union of exact faces is
// classified exactly, so no gap shift is applied under any tolerance.
func TestChamferConcaveEndKeepsItsRelief(t *testing.T) {
	t.Parallel()
	const d = 0.5
	cot := func(deg float64) float64 { return 1 / math.Tan(deg*math.Pi/180) }
	ell := [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {10, 30}, {0, 30}}
	for _, c := range []struct {
		name  string
		pts   [][2]float64
		phi   float64
		gap   float64
		shift bool
	}{
		{"right angle", ell, 90, 0.01, false},
		{"45 degrees", [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {0, 20}}, 45, 0.01, false},
		{"right angle under a fine gap tolerance", ell, 90, 0.003, false},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			ctx := t.Context()
			w := sketch.NewWorld()
			f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
			require.NoError(t, err)
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			ps := make([]*sketch.Point, len(c.pts))
			for i, p := range c.pts {
				ps[i] = s.CreatePoint(p[0], p[1])
				s.Fix(ps[i])
			}
			for i, p := range ps {
				s.CreateLine(p, ps[(i+1)%len(ps)])
			}
			_, err = s.Solve(ctx)
			require.NoError(t, err)
			var pr *sketch.Profile
			for _, p := range s.Profiles() {
				if p.Valid {
					pr = p
				}
			}
			require.NotNil(t, pr)
			doc := decad.New(decad.WithGapTolerance(units.Millimeters(c.gap)))
			body, err := decad.Extrude(ctx, doc, s, pr, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
			require.NoError(t, err)
			before := volumeOf(t, body)

			out, err := decad.Chamfer(ctx, body, topEdges(lineEdge(1, c.pts[2], c.pts[3])), units.Millimeters(d))
			require.NoError(t, err)
			shifted := false
			for _, diag := range out.OperationDiagnostics() {
				shifted = shifted || diag.Code == decad.OpDiagGapShiftApplied
			}
			require.Equal(t, c.shift, shifted)
			v, err := out.Volume()
			require.NoError(t, err)
			want := before - 20*d*d/2 + d*d*d*cot(c.phi)/6
			kept := 0.0
			if !c.shift {
				kept = 0.004 * 0.004 * (c.phi * math.Pi / 180) * d / 2
			}
			require.GreaterOrEqual(t, v.Value.Base(), want-v.Bound.Base())
			require.LessOrEqual(t, v.Value.Base(), want+kept+v.Bound.Base())
			require.Less(t, v.Bound.Base(), 1e-3)
		})
	}
}
