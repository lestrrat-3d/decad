package filletband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// polygon is a loop of straight walks through pts, counter-clockwise for an
// outer loop and clockwise for a hole, with each corner classified by its
// exact turn: a left turn LF4, a right turn LF6.
func polygon(t *testing.T, pts [][2]float64) filletband.Loop {
	t.Helper()
	var l filletband.Loop
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		l.Walks = append(l.Walks, filletband.Walk{Start: filletband.Point{U: p[0], V: p[1]}, End: filletband.Point{U: q[0], V: q[1]}})
	}
	n := len(l.Walks)
	for k := range l.Walks {
		turn, err := filletband.Turn(l.Walks[(k+n-1)%n], l.Walks[k])
		require.NoError(t, err)
		require.NotZero(t, turn)
		c := filletband.Miter
		if turn < 0 {
			c = filletband.Reflex
		}
		l.Corners = append(l.Corners, c)
	}
	return l
}

// requireEncloses asserts iv contains every value of [lo, hi].
func requireEncloses(t *testing.T, iv filletband.Interval, lo, hi *big.Rat, what string) {
	t.Helper()
	require.LessOrEqual(t, iv.Lo.Cmp(lo), 0, "%s: [%s, %s] misses %s", what, iv.Lo.FloatString(18), iv.Hi.FloatString(18), lo.FloatString(18))
	require.GreaterOrEqual(t, iv.Hi.Cmp(hi), 0, "%s: [%s, %s] misses %s", what, iv.Lo.FloatString(18), iv.Hi.FloatString(18), hi.FloatString(18))
}

// requirePoint asserts iv is exactly the rational want.
func requirePoint(t *testing.T, iv filletband.Interval, want *big.Rat, what string) {
	t.Helper()
	require.Zero(t, iv.Lo.Cmp(want), "%s: low end %s, want %s", what, iv.Lo.RatString(), want.RatString())
	require.Zero(t, iv.Hi.Cmp(want), "%s: high end %s, want %s", what, iv.Hi.RatString(), want.RatString())
}

// piPoly encloses a + b·π + c·π² over the proven π bracket.
func piPoly(a, b, c *big.Rat) (*big.Rat, *big.Rat) {
	at := func(p *big.Rat) *big.Rat {
		out := new(big.Rat).Add(a, new(big.Rat).Mul(b, p))
		return out.Add(out, new(big.Rat).Mul(c, new(big.Rat).Mul(p, p)))
	}
	lo, hi := at(proofbound.PiLower), at(proofbound.PiUpper)
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	// a + bπ + cπ² is monotone over a bracket far narrower than any of its
	// stationary points here, so its two ends enclose it.
	return lo, hi
}

func rat(n, d int64) *big.Rat { return big.NewRat(n, d) }

// TestCoefficientsOfARectangle checks Table CF on P1's 40×20 top loop: a₁ is
// the perimeter, a₂ = −Σκ = −4 for four right-angle miters, and by symmetry
// the strip's first moment is its area times the centre (20, 10), so
// m₁ = (2400, 1200), m₂ = (−80, −40) and m₃ = 0, all exact.
func TestCoefficientsOfARectangle(t *testing.T) {
	t.Parallel()
	c, err := polygon(t, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {0, 20}}).Coefficients()
	require.NoError(t, err)
	requirePoint(t, c.A1, rat(120, 1), "a₁")
	requirePoint(t, c.A2, rat(-4, 1), "a₂")
	requirePoint(t, c.M1[0], rat(2400, 1), "m₁u")
	requirePoint(t, c.M1[1], rat(1200, 1), "m₁v")
	requirePoint(t, c.M2[0], rat(-80, 1), "m₂u")
	requirePoint(t, c.M2[1], rat(-40, 1), "m₂v")
	requirePoint(t, c.M3[0], rat(0, 1), "m₃u")
	requirePoint(t, c.M3[1], rat(0, 1), "m₃v")
}

// Two adjacent top edges of the 40×20 box move inward by t while the other
// two stay put. The removed planar strip is 800−(40−t)(20−t) = 60t−t².
func TestSelectedCornerStripOfARectangle(t *testing.T) {
	t.Parallel()
	l := polygon(t, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {0, 20}})
	pieces, err := l.PiecesSelected([]bool{true, false, false, true})
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	c := filletband.SumCoefficients(pieces)
	requirePoint(t, c.A1, rat(60, 1), "selected strip a₁")
	requirePoint(t, c.A2, rat(-1, 1), "selected strip a₂")
}

// TestCoefficientsOfAPocketMouth checks CF1 with CF4 on P2's 20×10 mouth, a
// hole walked clockwise whose four corners are reflex: a₁ = 60, a₂ = ψ/2
// summed over four quarter turns = π, and by symmetry the first moment is
// the area times the centre (20, 20): m₁ = (1200, 1200), m₂ = (20π, 20π),
// m₃ = 0.
func TestCoefficientsOfAPocketMouth(t *testing.T) {
	t.Parallel()
	l := polygon(t, [][2]float64{{10, 15}, {10, 25}, {30, 25}, {30, 15}})
	for _, c := range l.Corners {
		require.Equal(t, filletband.Reflex, c)
	}
	c, err := l.Coefficients()
	require.NoError(t, err)
	requirePoint(t, c.A1, rat(60, 1), "a₁")
	lo, hi := piPoly(rat(0, 1), rat(1, 1), rat(0, 1))
	requireEncloses(t, c.A2, lo, hi, "a₂")
	requirePoint(t, c.M1[0], rat(1200, 1), "m₁u")
	lo, hi = piPoly(rat(0, 1), rat(20, 1), rat(0, 1))
	requireEncloses(t, c.M2[0], lo, hi, "m₂u")
	requireEncloses(t, c.M2[1], lo, hi, "m₂v")
	requireEncloses(t, c.M3[0], rat(0, 1), rat(0, 1), "m₃u")
	requireEncloses(t, c.M3[1], rat(0, 1), rat(0, 1), "m₃v")
}

// polygonMoment is the exact area and first moment of a simple polygon by the
// shoelace sums.
func polygonMoment(pts [][2]*big.Rat) (*big.Rat, [2]*big.Rat) {
	area := new(big.Rat)
	mx, my := new(big.Rat), new(big.Rat)
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		cr := new(big.Rat).Sub(new(big.Rat).Mul(p[0], q[1]), new(big.Rat).Mul(q[0], p[1]))
		area.Add(area, cr)
		mx.Add(mx, new(big.Rat).Mul(cr, new(big.Rat).Add(p[0], q[0])))
		my.Add(my, new(big.Rat).Mul(cr, new(big.Rat).Add(p[1], q[1])))
	}
	area.Mul(area, rat(1, 2))
	mx.Mul(mx, rat(1, 6))
	my.Mul(my, rat(1, 6))
	return area, [2]*big.Rat{mx, my}
}

// TestCoefficientsOfTheTrapezoid checks CF1 at oblique LF4 corners on the
// bound fixture's trapezoid (0, 0), (100, 0), (72, 45), (28, 45): its slanted
// sides have length 53, so κ = 9/5 at the base corners and 5/9 at the top,
// a₁ = 250 and a₂ = −212/45. The strip S(t) is the trapezoid less its offset
// by t, whose corners are the exact meets of the offset lines, so its area
// and first moment are exact rationals at t = 1 and t = 3, and the
// polynomials must reproduce both.
func TestCoefficientsOfTheTrapezoid(t *testing.T) {
	t.Parallel()
	c, err := polygon(t, [][2]float64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}).Coefficients()
	require.NoError(t, err)
	requirePoint(t, c.A1, rat(250, 1), "a₁")
	requirePoint(t, c.A2, rat(-212, 45), "a₂")

	pts := [][2]int64{{0, 0}, {100, 0}, {72, 45}, {28, 45}}
	lengths := []int64{100, 53, 44, 53}
	outer := make([][2]*big.Rat, len(pts))
	for i, p := range pts {
		outer[i] = [2]*big.Rat{rat(p[0], 1), rat(p[1], 1)}
	}
	a0, m0 := polygonMoment(outer)
	for _, tt := range []int64{1, 3} {
		// Side i's offset line −dy·x + dx·y = −dy·px + dx·py + t·length.
		type line struct{ a, b, c *big.Rat }
		lines := make([]line, len(pts))
		for i, p := range pts {
			q := pts[(i+1)%len(pts)]
			dx, dy := q[0]-p[0], q[1]-p[1]
			lines[i] = line{rat(-dy, 1), rat(dx, 1), rat(-dy*p[0]+dx*p[1]+tt*lengths[i], 1)}
		}
		inner := make([][2]*big.Rat, len(pts))
		for i := range lines {
			l0, l1 := lines[(i+len(lines)-1)%len(lines)], lines[i]
			det := new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.b), new(big.Rat).Mul(l0.b, l1.a))
			x := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.c, l1.b), new(big.Rat).Mul(l0.b, l1.c)), det)
			y := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(l0.a, l1.c), new(big.Rat).Mul(l0.c, l1.a)), det)
			inner[i] = [2]*big.Rat{x, y}
		}
		a1, m1 := polygonMoment(inner)
		tr := rat(tt, 1)
		poly := func(c1, c2, c3 filletband.Interval) filletband.Interval {
			t2 := new(big.Rat).Mul(tr, tr)
			t3 := new(big.Rat).Mul(t2, tr)
			return proofbound.IntervalAdd(proofbound.IntervalAdd(proofbound.IntervalScale(c1, tr), proofbound.IntervalScale(c2, t2)), proofbound.IntervalScale(c3, t3))
		}
		area := new(big.Rat).Sub(a0, a1)
		requireEncloses(t, poly(c.A1, c.A2, proofbound.PointInterval(new(big.Rat))), area, area, "A(t)")
		for k := range 2 {
			want := new(big.Rat).Sub(m0[k], m1[k])
			requireEncloses(t, poly(c.M1[k], c.M2[k], c.M3[k]), want, want, "M(t)")
		}
	}
}

// TestCoefficientsOfCircles checks CF2 and CF3 on whole circles (E = 0): a
// boss rim of radius 5 has a₁ = 10π and a₂ = −π, a hole rim of radius 3
// a₁ = 6π and a₂ = +π, and each strip's moment is its area times the centre.
func TestCoefficientsOfCircles(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		ccw    bool
		radius float64
		a1, a2 int64
	}{{true, 5, 10, -1}, {false, 3, 6, 1}} {
		l := filletband.Loop{Walks: []filletband.Walk{{Circular: true, Closed: true, CCW: tc.ccw,
			Start: filletband.Point{U: 20 + tc.radius, V: 20}, End: filletband.Point{U: 20 + tc.radius, V: 20},
			Center: filletband.Point{U: 20, V: 20}, Radius: tc.radius}}}
		c, err := l.Coefficients()
		require.NoError(t, err)
		lo, hi := piPoly(rat(0, 1), rat(tc.a1, 1), rat(0, 1))
		requireEncloses(t, c.A1, lo, hi, "a₁")
		lo, hi = piPoly(rat(0, 1), rat(tc.a2, 1), rat(0, 1))
		requireEncloses(t, c.A2, lo, hi, "a₂")
		lo, hi = piPoly(rat(0, 1), rat(20*tc.a1, 1), rat(0, 1))
		requireEncloses(t, c.M1[0], lo, hi, "m₁u")
		lo, hi = piPoly(rat(0, 1), rat(20*tc.a2, 1), rat(0, 1))
		requireEncloses(t, c.M2[1], lo, hi, "m₂v")
	}
}

// TestCoefficientsOfAnArc checks CF2 and CF3's first moment on a partial arc
// against a direct quadrature of the annular sector: the arc about (3, 4) of
// radius 5 from (8, 4) to (6, 8), walked both ways, at t = 1.5.
func TestCoefficientsOfAnArc(t *testing.T) {
	t.Parallel()
	for _, ccw := range []bool{true, false} {
		w := filletband.Walk{Circular: true, CCW: ccw, Start: filletband.Point{U: 8, V: 4}, End: filletband.Point{U: 6, V: 8}, Center: filletband.Point{U: 3, V: 4}}
		if !ccw {
			w.Start, w.End = w.End, w.Start
		}
		pieces, err := (filletband.Loop{Walks: []filletband.Walk{w}, Corners: []filletband.Corner{filletband.Tangent}}).Pieces()
		require.NoError(t, err)
		c := pieces[0].Coefficients
		const tt = 1.5
		r0, r1 := 5-tt, 5.0
		if !ccw {
			r0, r1 = 5, 5+tt
		}
		a0, a1 := 0.0, math.Atan2(4, 3)
		const n = 2000
		var mu, mv, area float64
		for i := range n {
			for j := range n {
				rho := r0 + (r1-r0)*(float64(i)+0.5)/n
				a := a0 + (a1-a0)*(float64(j)+0.5)/n
				dA := rho * (r1 - r0) / n * (a1 - a0) / n
				area += dA
				mu += (3 + rho*math.Cos(a)) * dA
				mv += (4 + rho*math.Sin(a)) * dA
			}
		}
		eval := func(c1, c2, c3 filletband.Interval) float64 {
			f := func(iv filletband.Interval) float64 { v, _ := iv.Lo.Float64(); return v }
			return f(c1)*tt + f(c2)*tt*tt + f(c3)*tt*tt*tt
		}
		zero := proofbound.PointInterval(new(big.Rat))
		require.InDelta(t, area, eval(c.A1, c.A2, zero), 1e-5, "A(t), ccw %v", ccw)
		require.InDelta(t, mu, eval(c.M1[0], c.M2[0], c.M3[0]), 1e-4, "Mu(t), ccw %v", ccw)
		require.InDelta(t, mv, eval(c.M1[1], c.M2[1], c.M3[1]), 1e-4, "Mv(t), ccw %v", ccw)
	}
}

// TestHeightsMatchQuadrature checks §5.2's closed forms at r = 2 against a
// midpoint quadrature of t(h)^k and h·t(h)^k.
func TestHeightsMatchQuadrature(t *testing.T) {
	t.Parallel()
	h, err := filletband.HeightsOf(proofbound.PointInterval(rat(2, 1)))
	require.NoError(t, err)
	const r, n = 2.0, 200000
	var j [4]float64
	var hk [3]float64
	for i := range n {
		x := r * (float64(i) + 0.5) / n
		tv := r - math.Sqrt(r*r-x*x)
		for k := 1; k <= 3; k++ {
			j[k] += math.Pow(tv, float64(k)) * r / n
		}
		for k := 1; k <= 2; k++ {
			hk[k] += x * math.Pow(tv, float64(k)) * r / n
		}
	}
	mid := func(iv filletband.Interval) float64 {
		s := new(big.Rat).Add(iv.Lo, iv.Hi)
		v, _ := s.Mul(s, rat(1, 2)).Float64()
		return v
	}
	require.InDelta(t, j[1], mid(h.J1), 1e-6)
	require.InDelta(t, j[2], mid(h.J2), 1e-6)
	require.InDelta(t, j[3], mid(h.J3), 1e-6)
	require.InDelta(t, hk[1], mid(h.H1), 1e-6)
	require.InDelta(t, hk[2], mid(h.H2), 1e-6)
	lo, hi := piPoly(rat(4, 1), rat(-1, 1), rat(0, 1))
	requireEncloses(t, h.J1, lo, hi, "J₁ = r²(1 − π/4)")
}

// TestQuarterEllipseLengthEnclosesTheArc checks the LF4 edge's length
// enclosure against a fine quadrature of the quarter ellipse with semi-axes
// 2√2 and 2 (P1's corner at r = 2, κ = 1), and that its width is the Riemann
// gap the design states.
func TestQuarterEllipseLengthEnclosesTheArc(t *testing.T) {
	t.Parallel()
	iv, err := filletband.QuarterEllipseLength(proofbound.PointInterval(rat(2, 1)), proofbound.PointInterval(rat(1, 1)))
	require.NoError(t, err)
	const n = 1000000
	var sum float64
	for i := range n {
		phi := math.Pi / 2 * (float64(i) + 0.5) / n
		s := math.Sin(phi)
		sum += 2 * math.Sqrt(1+s*s) * math.Pi / 2 / n
	}
	lo, _ := iv.Lo.Float64()
	hi, _ := iv.Hi.Float64()
	require.Less(t, lo, sum)
	require.Greater(t, hi, sum)
	require.Less(t, hi-lo, 2*(math.Sqrt2-1)*math.Pi/2/64*1.01)
}

// TestExtremeOfARectangleBand checks §5.4 on P1's top-loop band at r = 2
// (sideZ = 18, m = −1): along every axis the band reaches no farther than its
// directrices, exactly, and along (1, 0, 1) the quarter cylinder on the
// x = 40 wall bulges past both directrices to its interior stationary point.
func TestExtremeOfARectangleBand(t *testing.T) {
	t.Parallel()
	l := polygon(t, [][2]float64{{0, 0}, {40, 0}, {40, 20}, {0, 20}})
	pieces, err := l.Pieces()
	require.NoError(t, err)
	r := proofbound.PointInterval(rat(2, 1))
	sideZ := proofbound.PointInterval(rat(18, 1))
	for _, tc := range []struct {
		g    [3]int64
		want int64
	}{{[3]int64{1, 0, 0}, 40}, {[3]int64{-1, 0, 0}, 0}, {[3]int64{0, 1, 0}, 20}, {[3]int64{0, 0, 1}, 20}, {[3]int64{0, 0, -1}, -18}} {
		g := [3]*big.Rat{rat(tc.g[0], 1), rat(tc.g[1], 1), rat(tc.g[2], 1)}
		e, err := filletband.Extreme(pieces, r, sideZ, -1, g)
		require.NoError(t, err)
		requirePoint(t, e, rat(tc.want, 1), "axis extreme")
	}
	// Along (1, 0, 1): the x = 40 patch's ball centres run at u = 38, z = 18,
	// so its extreme is 38 + 18 + 2√2.
	e, err := filletband.Extreme(pieces, r, sideZ, -1, [3]*big.Rat{rat(1, 1), rat(0, 1), rat(1, 1)})
	require.NoError(t, err)
	want := 56 + 2*math.Sqrt2
	lo, _ := e.Lo.Float64()
	hi, _ := e.Hi.Float64()
	require.LessOrEqual(t, lo, want)
	require.GreaterOrEqual(t, hi, want)
	require.Less(t, hi-lo, 1e-12)
}
