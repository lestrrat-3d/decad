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
// concave corner turned 10 degrees beside a line, the next wall, and turns a sharp 90 degree corner from the arc into
// the line. The end clip stops the bevel short of the concave corner, so the cutter never lies flush with the next
// wall, and the Cut needs no gap shift. A gap shift would move the clip face across the arc's wall at the corner's 10
// degree turn, where the result's bound divides the wall's chord bound by sin(10 degrees). The relief is charged to
// the vertices near the corner instead.
func TestChamferConcaveArcEndHoldsItsBound(t *testing.T) {
	t.Parallel()
	const (
		r     = 14.0  // the arc's radius, about the origin
		from  = 200.0 // the arc's start, the concave corner, in degrees
		to    = 300.0 // the arc's end, where the line turns inward
		inner = 11.0  // where the inward line ends
		turn  = 10.0  // the concave corner's turn
	)
	rad := func(deg float64) float64 { return deg * math.Pi / 180 }
	polar := func(rho, deg float64) [2]float64 {
		return [2]float64{rho * math.Cos(rad(deg)), rho * math.Sin(rad(deg))}
	}
	v, w, p := polar(r, from), polar(r, to), polar(inner, to)
	// The next wall arrives at v along the arc's tangent turned 10 degrees toward the centre.
	u := [2]float64{math.Cos(rad(from + 90 + turn)), math.Sin(rad(from + 90 + turn))}
	a := [2]float64{v[0] - 3*u[0], v[1] - 3*u[1]}
	// The chain's other end turns 60 degrees, from the inward line onto a line along -X. The outline then reaches up to
	// y = 20, so the pair's chord tolerance, a fixed share of its diameter, passes the cutter's bound.
	q, f := [2]float64{p[0] - 10, p[1]}, [2]float64{p[0] - 10, 20}
	// Chamfer retires its receiver from the document, so each setback draws its own body.
	draw := func(s *sketch.Sketch) {
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
	}
	inward := r3.Vec{X: p[0] - w[0], Y: p[1] - w[1]}
	sel := topEdges(topEdge{r3.Vec{X: w[0], Y: w[1]}, inward}).
		Or(decad.EndpointAt(r3.Vec{X: v[0], Y: v[1], Z: 5}), decad.Circular()).
		Or(decad.EndpointAt(r3.Vec{X: w[0], Y: w[1]}), decad.ParallelTo(inward)).
		Or(decad.EndpointAt(r3.Vec{X: v[0], Y: v[1]}), decad.Circular())
	// The next wall's line passes r cos(10 degrees) = 13.79 from the centre, so it caps the bevel up to a setback of
	// 0.21 mm.
	for _, d := range []float64{0.2121, 0.2, 0.19, 0.15, 0.12} {
		t.Run(fmt.Sprintf("setback %v", d), func(t *testing.T) {
			t.Parallel()
			out, err := decad.Chamfer(t.Context(), capLoopSubsetBody(t, draw), sel, units.Millimeters(d))
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			for _, diag := range out.OperationDiagnostics() {
				require.NotEqual(t, decad.OpDiagGapShiftApplied, diag.Code)
			}
			// The box's bound reads about 0.0125 mm here, the relief's charge included, and about 0.02 mm after a gap
			// shift on this outline.
			box, err := out.Bounds()
			require.NoError(t, err)
			require.Less(t, box.Bound.Base(), 0.016)
			// The vertices beside the top corner carry the relief.
			corner := 0.0
			for _, vx := range out.Vertices() {
				pos := vx.Position()
				if math.Hypot(pos.Value.X-v[0], pos.Value.Y-v[1]) < 0.01 && pos.Value.Z > 5-d {
					corner = max(corner, pos.Bound.Base())
				}
			}
			require.GreaterOrEqual(t, corner, 0.004)

			// Below the top by h the outline follows the next wall's line from the corner to the bevel's top circle,
			// radius r - d + h, at the line's first crossing lambda1 = cu - sqrt((r - d + h)^2 - D^2), D and cu the
			// distances from the centre to the line and from the foot of the centre to the corner along it.
			D, cu := r*math.Cos(rad(turn)), r*math.Sin(rad(turn))
			for _, h := range []float64{d / 4, d / 2} {
				o := sectionAt(t, out, h, 0.01)
				off, err := o.OffSurface.In(units.Millimeter)
				require.NoError(t, err)
				lam := cu - math.Sqrt((r-d+h)*(r-d+h)-D*D)
				require.Less(t, nearestLoopPoint(o, v[0]+lam*u[0], v[1]+lam*u[1]), off+1e-3, "next wall at depth %v", h)
			}
		})
	}
}

// TestChamferConcaveEndChargesItsRelief chamfers a line run that ends at a concave corner, turned 90 or 45 degrees, in
// documents whose gap tolerance does and does not reach the corner relief. Either way the result's volume lies within
// its bound of the chamfer whose bevel stops on the next wall's plane, without the relief: the relief keeps about
// 6e-6 mm3 at the right angle, which its charge covers, and without it the gap shift's charge covers the shift. A
// document whose gap tolerance is under the relief takes the shift.
func TestChamferConcaveEndChargesItsRelief(t *testing.T) {
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
		{"right angle under a fine gap tolerance", ell, 90, 0.003, true},
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
			require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base())
			require.Less(t, v.Bound.Base(), 1e-3)
		})
	}
}
