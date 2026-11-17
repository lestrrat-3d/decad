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
// degree turn, where the result's bound divides the wall's chord bound by sin(10 degrees).
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
			// The box's bound reads about 0.006 mm here, and about 0.02 mm after a gap shift.
			box, err := out.Bounds()
			require.NoError(t, err)
			require.Less(t, box.Bound.Base(), 0.01)

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
