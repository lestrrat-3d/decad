package circularbounds

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// driftedArcFixture is an ArcSeg whose End sits one ulp of its radius off
// Start's circle: Start is on the unit circle about (5, −3), End on the
// circle of radius nextafter(1, +Inf) at angle atan2(0.8, 0.6). The exact
// squared radii differ, so every circular bracket has to read End's radial
// ratio rather than trust it to be 1.
func driftedArcFixture(t *testing.T) (ArcSeg, *big.Rat, *big.Rat) {
	t.Helper()
	d := math.Nextafter(1, math.Inf(1))
	seg := ArcSeg{
		Center: Point2{U: 5, V: -3},
		Start:  Point2{U: 6, V: -3},
		End:    Point2{U: 5 + 0.6*d, V: -3 + 0.8*d},
		TStart: 0,
		TEnd:   1,
	}
	dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
	dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
	r2 := proofbound.RatAdd(proofbound.RatMul(dx0, dx0), proofbound.RatMul(dy0, dy0))
	endR2 := proofbound.RatAdd(proofbound.RatMul(dx1, dx1), proofbound.RatMul(dy1, dy1))
	require.NotEqual(t, 0, endR2.Cmp(r2), `the fixture's two exact squared radii must differ`)
	q := new(big.Rat).Quo(r2, endR2)
	require.NotEqual(t, proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q), `the radial ratio's bracket must have width`)
	return seg, r2, endR2
}

func requireIntervalWidthAtMost(t *testing.T, name string, iv proofbound.RatInterval, ceiling float64) {
	t.Helper()
	width, _ := new(big.Rat).Sub(iv.Hi, iv.Lo).Float64()
	require.GreaterOrEqual(t, width, 0.0, "%s: an interval's hi must not sit below its lo", name)
	require.LessOrEqual(t, width, ceiling, "%s: interval width %g exceeds %g", name, width, ceiling)
}

func requireIntervalNegates(t *testing.T, name string, fwd, rev proofbound.RatInterval) {
	t.Helper()
	require.Zero(t, rev.Lo.Cmp(new(big.Rat).Neg(fwd.Hi)), "%s: reversed lo must be the forward hi negated", name)
	require.Zero(t, rev.Hi.Cmp(new(big.Rat).Neg(fwd.Lo)), "%s: reversed hi must be the forward lo negated", name)
}

func requirePointInterval(t *testing.T, name string, iv proofbound.RatInterval) {
	t.Helper()
	require.Zero(t, iv.Lo.Cmp(iv.Hi), "%s: expected a point interval, got [%s, %s]",
		name, iv.Lo.FloatString(20), iv.Hi.FloatString(20))
}

// Shown-to-fail: swapping the proofbound.RatSqrtDown/proofbound.RatSqrtUp calls in
// arcEndRadialRatio inverts the bracket, and the containment leg goes red.
func TestArcEndRadialRatioBracketsTheRatio(t *testing.T) {
	t.Parallel()
	_, r2, endR2 := driftedArcFixture(t)
	q := new(big.Rat).Quo(r2, endR2)

	rho, ok := arcEndRadialRatio(r2, endR2)
	require.True(t, ok)
	require.LessOrEqual(t, new(big.Rat).Mul(rho.Lo, rho.Lo).Cmp(q), 0, `lo² must not exceed r²/endR²`)
	require.GreaterOrEqual(t, new(big.Rat).Mul(rho.Hi, rho.Hi).Cmp(q), 0, `hi² must not fall below r²/endR²`)
	requireIntervalWidthAtMost(t, "rho", rho, math.Ldexp(1, -48))

	equal, ok := arcEndRadialRatio(r2, r2)
	require.True(t, ok)
	require.Zero(t, equal.Lo.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)
	require.Zero(t, equal.Hi.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)

	_, ok = arcEndRadialRatio(r2, new(big.Rat))
	require.False(t, ok, `an End on the Center has no radial ratio`)
}

// Shown-to-fail: restoring `if endR2.Cmp(r2) != 0 { return …, false }` in
// circularFirstMomentInterval's ArcSeg arm answers ok == false for the drifted
// fixture, and the first leg goes red.
func TestCircularFirstMomentIntervalChargesEndpointRadiusDrift(t *testing.T) {
	t.Parallel()
	seg, _, _ := driftedArcFixture(t)
	anchor := Point2{}

	mu, mv, ok := circularFirstMomentInterval(seg, anchor)
	require.True(t, ok, `a drifted End is charged into the bracket, not refused`)
	requireIntervalWidthAtMost(t, "mu", mu, 1e-12)
	requireIntervalWidthAtMost(t, "mv", mv, 1e-12)

	reversed := seg
	reversed.TStart, reversed.TEnd = 1, 0
	revMU, revMV, ok := circularFirstMomentInterval(reversed, anchor)
	require.True(t, ok)
	requireIntervalNegates(t, "mu", mu, revMU)
	requireIntervalNegates(t, "mv", mv, revMV)
}

// Shown-to-fail: restoring `if endR2.Cmp(r2) != 0 { return …, false }` in
// circularSecondMomentInterval's ArcSeg arm answers ok == false for the
// drifted fixture, and the first leg goes red.
func TestCircularSecondMomentIntervalChargesEndpointRadiusDrift(t *testing.T) {
	t.Parallel()
	seg, _, _ := driftedArcFixture(t)
	anchor := Point2{}

	muu, muv, mvv, ok := circularSecondMomentInterval(seg, anchor)
	require.True(t, ok, `a drifted End is charged into the bracket, not refused`)
	requireIntervalWidthAtMost(t, "muu", muu, 1e-12)
	requireIntervalWidthAtMost(t, "muv", muv, 1e-12)
	requireIntervalWidthAtMost(t, "mvv", mvv, 1e-12)

	reversed := seg
	reversed.TStart, reversed.TEnd = 1, 0
	revMUU, revMUV, revMVV, ok := circularSecondMomentInterval(reversed, anchor)
	require.True(t, ok)
	requireIntervalNegates(t, "muu", muu, revMUU)
	requireIntervalNegates(t, "muv", muv, revMUV)
	requireIntervalNegates(t, "mvv", mvv, revMVV)
}

// With the centre on the anchor every swept-angle coefficient of mu, mv and
// muv is zero and every other term is rational, so any width these intervals
// carry could only come from End's radial ratio. Equal radii (End = (12, 16)
// on the radius-20 circle, both of its deltas nonzero so every leg reads ρ)
// must keep it a point.
//
// Shown-to-fail: replacing the equal-radii fast path's proofbound.PointInterval(1) in
// arcEndRadialRatio by proofbound.Interval(1, 1 + 2^-52) widens mu, mv and muv, and each
// point-interval leg goes red.
func TestCircularMomentIntervalsExactArcKeepPointRadialRatio(t *testing.T) {
	t.Parallel()
	seg := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 20, V: 0},
		End:    Point2{U: 12, V: 16},
		TStart: 0,
		TEnd:   1,
	}
	anchor := Point2{}

	mu, mv, ok := circularFirstMomentInterval(seg, anchor)
	require.True(t, ok)
	requirePointInterval(t, "mu", mu)
	requirePointInterval(t, "mv", mv)

	_, muv, _, ok := circularSecondMomentInterval(seg, anchor)
	require.True(t, ok)
	requirePointInterval(t, "muv", muv)
}

func requireIntervalsOverlap(t *testing.T, name string, a, b proofbound.RatInterval) {
	t.Helper()
	require.LessOrEqual(t, a.Lo.Cmp(b.Hi), 0, "%s: [%s, %s] lies above [%s, %s]", name,
		a.Lo.FloatString(20), a.Hi.FloatString(20), b.Lo.FloatString(20), b.Hi.FloatString(20))
	require.LessOrEqual(t, b.Lo.Cmp(a.Hi), 0, "%s: [%s, %s] lies below [%s, %s]", name,
		a.Lo.FloatString(20), a.Hi.FloatString(20), b.Lo.FloatString(20), b.Hi.FloatString(20))
}

// TestCircularMonomialsAgreeWithClosedForms cross-checks circularMonomials'
// generic trig-power reduction against the hand-expanded first- and
// second-moment enclosures, over the forms both evaluate the same way
// (∫u dA = ½∮u²dv, ∫u² dA = ⅓∮u³dv, ∫uv dA = ½∮u²v dv): a fractional
// CircleSeg, a whole one, and the drifted ArcSeg walked both ways. Two sound
// enclosures of one value must overlap.
//
// Shown-to-fail: dropping the a ≥ 2 reduction's (a−1)·r² lower term
// separates every case. These forms never reach the b ≥ 2 reduction;
// TestThirdOrderMomentsOfASector covers it.
func TestCircularMonomialsAgreeWithClosedForms(t *testing.T) {
	t.Parallel()
	drifted, _, _ := driftedArcFixture(t)
	reversed := drifted
	reversed.TStart, reversed.TEnd = 1, 0
	for name, seg := range map[string]CurveSegment{
		"fractional circle": CircleSeg{Center: Point2{U: 5, V: -3}, Radius: units.Millimeters(2),
			TStart: 0.125, TEnd: 0.4375, CCW: true},
		"whole circle": CircleSeg{Center: Point2{U: 5, V: -3}, Radius: units.Millimeters(2),
			TStart: 0, TEnd: 1, CCW: true},
		"forward arc": drifted,
		"reverse arc": reversed,
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			walk, ok := circularMomentWalkOf(seg)
			require.True(t, ok)
			j := circularMonomials(walk, 5)
			mu, _, ok := circularFirstMomentInterval(seg, Point2{})
			require.True(t, ok)
			muu, muv, _, ok := circularSecondMomentInterval(seg, Point2{})
			require.True(t, ok)
			for _, check := range []struct {
				name string
				p, q int
				want proofbound.RatInterval
			}{{"mu", 1, 0, mu}, {"muu", 2, 0, muu}, {"muv", 1, 1, muv}} {
				got := circularGreenMoment(walk, j, check.p, check.q)
				requireIntervalsOverlap(t, check.name, got, check.want)
				requireIntervalWidthAtMost(t, check.name, got, 1e-9)
			}
		})
	}
}

// TestCircularThirdMomentWholeCircle checks a whole CCW circle's third-order
// contributions against the disc's own: about its centre ∫x² dA = πr⁴/4 and
// every odd moment vanishes, so ∫u³ = π(cU³r² + 3cU·r⁴/4),
// ∫u²v = π(cU²cV·r² + cV·r⁴/4), ∫uv² = π(cU·cV²·r² + cU·r⁴/4) and
// ∫v³ = π(cV³r² + 3cV·r⁴/4). The turn starting at 1/8 has endpoint sine and
// cosine enclosures of series width; the whole-turn closure is what keeps
// them out of the result, leaving only 2π's enclosure.
//
// Shown-to-fail: always clearing circularMomentWalk.closed widens the
// 1/8-turn walk's intervals past the width ceiling; dropping J(0,0)'s sweep
// separates all four intervals from the closed form.
func TestCircularThirdMomentWholeCircle(t *testing.T) {
	t.Parallel()
	cU, cV := big.NewRat(3, 1), big.NewRat(-2, 1)
	r2, r4 := big.NewRat(4, 1), big.NewRat(16, 1)
	quarterR4 := ratScale(r4, 1, 4)
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	want := [4]*big.Rat{
		proofbound.RatAdd(proofbound.RatMul(cU, cU, cU, r2), proofbound.RatMul(big.NewRat(3, 1), cU, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cU, cU, cV, r2), proofbound.RatMul(cV, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cU, cV, cV, r2), proofbound.RatMul(cU, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cV, cV, cV, r2), proofbound.RatMul(big.NewRat(3, 1), cV, quarterR4)),
	}
	for _, start := range []float64{0, 0.125} {
		seg := CircleSeg{Center: Point2{U: 3, V: -2}, Radius: units.Millimeters(2),
			TStart: start, TEnd: start + 1, CCW: true}
		got, ok := circularThirdMomentInterval(seg)
		require.True(t, ok)
		for i, coefficient := range want {
			name := []string{"u³", "u²v", "uv²", "v³"}[i]
			requireIntervalsOverlap(t, name, got[i], proofbound.IntervalScale(pi, coefficient))
			requireIntervalWidthAtMost(t, name, got[i], 1e-65)
		}
	}
}

// fragmentForm is a + b·√2 + c·π over rationals: every moment of a walk on
// the radius-5 circle between multiples of π/4 lands in it, since its
// endpoint sines and cosines are 0, ±1 or ±√2/2 and its swept angle is a
// rational multiple of π that only ever multiplies rationals.
type fragmentForm struct{ a, b, c *big.Rat }

var (
	fragmentSqrt2Lo = mustFragmentDecimal("1.414213562373095048801688724209698078569671875376948073176679737")
	fragmentSqrt2Hi = mustFragmentDecimal("1.414213562373095048801688724209698078569671875376948073176679738")
	fragmentPiLo    = mustFragmentDecimal("3.141592653589793238462643383279502884197169399375105820974944592")
	fragmentPiHi    = mustFragmentDecimal("3.141592653589793238462643383279502884197169399375105820974944593")
)

func mustFragmentDecimal(s string) *big.Rat {
	r, ok := new(big.Rat).SetString(s)
	if !ok {
		panic("invalid decimal " + s)
	}
	return r
}

func ffRat(r *big.Rat) fragmentForm {
	return fragmentForm{new(big.Rat).Set(r), new(big.Rat), new(big.Rat)}
}

func ffInt(n int64) fragmentForm { return ffRat(big.NewRat(n, 1)) }

func (f fragmentForm) add(g fragmentForm) fragmentForm {
	return fragmentForm{
		new(big.Rat).Add(f.a, g.a), new(big.Rat).Add(f.b, g.b), new(big.Rat).Add(f.c, g.c),
	}
}

func (f fragmentForm) sub(g fragmentForm) fragmentForm { return f.add(g.scale(big.NewRat(-1, 1))) }

func (f fragmentForm) scale(s *big.Rat) fragmentForm {
	return fragmentForm{new(big.Rat).Mul(f.a, s), new(big.Rat).Mul(f.b, s), new(big.Rat).Mul(f.c, s)}
}

// mul multiplies two forms; a product that would need π·√2 or π² is outside
// the form, and no moment below produces one.
func (f fragmentForm) mul(t *testing.T, g fragmentForm) fragmentForm {
	t.Helper()
	cross := new(big.Rat).Add(new(big.Rat).Mul(f.b, g.c), new(big.Rat).Mul(f.c, g.b))
	require.Zero(t, cross.Sign(), "a π·√2 term is outside the form")
	require.Zero(t, new(big.Rat).Mul(f.c, g.c).Sign(), "a π² term is outside the form")
	a := new(big.Rat).Add(new(big.Rat).Mul(f.a, g.a), new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(f.b, g.b)))
	b := new(big.Rat).Add(new(big.Rat).Mul(f.a, g.b), new(big.Rat).Mul(f.b, g.a))
	c := new(big.Rat).Add(new(big.Rat).Mul(f.a, g.c), new(big.Rat).Mul(f.c, g.a))
	return fragmentForm{a, b, c}
}

func (f fragmentForm) pow(t *testing.T, n int) fragmentForm {
	out := ffInt(1)
	for range n {
		out = out.mul(t, f)
	}
	return out
}

func (f fragmentForm) bracket() (*big.Rat, *big.Rat) {
	term := func(c, lo, hi *big.Rat) (*big.Rat, *big.Rat) {
		a, b := new(big.Rat).Mul(c, lo), new(big.Rat).Mul(c, hi)
		if a.Cmp(b) > 0 {
			a, b = b, a
		}
		return a, b
	}
	l1, h1 := term(f.b, fragmentSqrt2Lo, fragmentSqrt2Hi)
	l2, h2 := term(f.c, fragmentPiLo, fragmentPiHi)
	lo := new(big.Rat).Add(f.a, new(big.Rat).Add(l1, l2))
	hi := new(big.Rat).Add(f.a, new(big.Rat).Add(h1, h2))
	return lo, hi
}

// quarterPiSinCos is sin and cos of k·π/4 for k in [−2, 2].
func quarterPiSinCos(k int) (fragmentForm, fragmentForm) {
	half := fragmentForm{new(big.Rat), big.NewRat(1, 2), new(big.Rat)}
	neg := func(f fragmentForm) fragmentForm { return f.scale(big.NewRat(-1, 1)) }
	switch k {
	case -2:
		return ffInt(-1), ffInt(0)
	case -1:
		return neg(half), half
	case 0:
		return ffInt(0), ffInt(1)
	case 1:
		return half, half
	default:
		return ffInt(1), ffInt(0)
	}
}

func requireIntervalEncloses(t *testing.T, name string, got proofbound.RatInterval, want fragmentForm) {
	t.Helper()
	lo, hi := want.bracket()
	require.LessOrEqual(t, got.Lo.Cmp(lo), 0, "%s: [%s, %s] starts above the closed form %s", name,
		got.Lo.FloatString(30), got.Hi.FloatString(30), lo.FloatString(30))
	require.GreaterOrEqual(t, got.Hi.Cmp(hi), 0, "%s: [%s, %s] ends below the closed form %s", name,
		got.Lo.FloatString(30), got.Hi.FloatString(30), hi.FloatString(30))
	requireIntervalWidthAtMost(t, name, got, 1e-20)
}

// TestNarrowedArcIntervalsEncloseClosedForms holds every circular bracket of
// an ArcSeg walked over a narrowed range to the walk's own closed form. The
// arc is the radius-5 semicircle from (0, −5) through (5, 0) to (0, 5), so
// θ(t) = −π/2 + t·π, and each range ends on a multiple of π/4. About the
// anchor (−3, 2) the centre sits at (cU, cV) = (3, −2); with X = r·cos θ,
// Y = r·sin θ, [g] = g(θ1) − g(θ0) and Δθ the signed sweep:
//
//	A    = ½(r²Δθ + cU[Y] − cV[X])
//	∫u   = ½∫(cU + X)²·X dθ,    ∫v  = ½∫(cV + Y)²·Y dθ
//	∫u²  = ⅓∫(cU + X)³·X dθ,    ∫v² = ⅓∫(cV + Y)³·Y dθ
//	∫uv  = ½∫(cU + X)²(cV + Y)·X dθ,  ∫u³ = ¼∫(cU + X)⁴·X dθ
//
// each expanded into the power integrals ∫cosⁿ, ∫sinⁿ, ∫cosᵃsinᵇ written out
// below. The axial moment about the v-parallel axis through the anchor is
// r·(3·|Δθ| + r·(sin hi − sin lo)).
//
// Shown-to-fail: replacing arcOffsetAt's proofbound.RadSinCosSpan by the
// point enclosure of math.Sincos at the angle's lower end separates every
// region moment (area through ∫u³) of all eight ranges from its closed form.
// Dropping the tLo shift from circularAxisMomentInterval's ArcSeg arm
// separates the axial moment of the interior and middle ranges, both ways;
// the two ranges ending at t == 1 mirror their unshifted walks about θ = 0,
// so their sine differences agree and that leg cannot see them.
func TestNarrowedArcIntervalsEncloseClosedForms(t *testing.T) {
	t.Parallel()
	const r = 5
	anchor := Point2{U: -3, V: 2}
	cU, cV := ffInt(3), ffInt(-2)
	ax := NewAxisFrame(anchor.U, anchor.V, 0, 0, 0, -1, 0, 0)
	for _, tc := range []struct {
		name   string
		t0, t1 float64
	}{
		{"interior", 0.25, 0.75},
		{"interior reversed", 0.75, 0.25},
		{"to the natural end", 0.75, 1},
		{"from the natural start", 0, 0.25},
		{"to the natural start, reversed", 0.25, 0},
		{"through the middle", 0.25, 1},
		{"from the middle", 0.5, 0.75},
		{"to the middle, reversed", 0.75, 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			seg := ArcSeg{
				Center: Point2{U: 0, V: 0}, Start: Point2{U: 0, V: -5}, End: Point2{U: 0, V: 5},
				TStart: tc.t0, TEnd: tc.t1,
			}
			k0, k1 := int(4*tc.t0)-2, int(4*tc.t1)-2
			s0, c0 := quarterPiSinCos(k0)
			s1, c1 := quarterPiSinCos(k1)
			rr := func(n int) *big.Rat { return new(big.Rat).SetInt64(int64(math.Pow(r, float64(n)))) }
			dth := fragmentForm{new(big.Rat), new(big.Rat), new(big.Rat).SetFloat64(tc.t1 - tc.t0)}
			diff := func(g func(s, c fragmentForm) fragmentForm) fragmentForm { return g(s1, c1).sub(g(s0, c0)) }
			sin2 := func(s, c fragmentForm) fragmentForm { return s.mul(t, c).scale(big.NewRat(2, 1)) }
			sin4 := func(s, c fragmentForm) fragmentForm {
				return sin2(s, c).mul(t, c.mul(t, c).sub(s.mul(t, s))).scale(big.NewRat(2, 1))
			}
			// ∫Xⁿ dθ and ∫Yⁿ dθ, each the trig-power antiderivative times rⁿ.
			intX := map[int]fragmentForm{
				1: diff(func(s, _ fragmentForm) fragmentForm { return s }),
				2: dth.scale(big.NewRat(1, 2)).add(diff(sin2).scale(big.NewRat(1, 4))),
				3: diff(func(s, _ fragmentForm) fragmentForm { return s.sub(s.pow(t, 3).scale(big.NewRat(1, 3))) }),
				4: dth.scale(big.NewRat(3, 8)).add(diff(sin2).scale(big.NewRat(1, 4))).add(diff(sin4).scale(big.NewRat(1, 32))),
				5: diff(func(s, _ fragmentForm) fragmentForm {
					return s.sub(s.pow(t, 3).scale(big.NewRat(2, 3))).add(s.pow(t, 5).scale(big.NewRat(1, 5)))
				}),
			}
			intY := map[int]fragmentForm{
				1: diff(func(_, c fragmentForm) fragmentForm { return c }).scale(big.NewRat(-1, 1)),
				2: dth.scale(big.NewRat(1, 2)).sub(diff(sin2).scale(big.NewRat(1, 4))),
				3: diff(func(_, c fragmentForm) fragmentForm { return c.pow(t, 3).scale(big.NewRat(1, 3)).sub(c) }),
				4: dth.scale(big.NewRat(3, 8)).sub(diff(sin2).scale(big.NewRat(1, 4))).add(diff(sin4).scale(big.NewRat(1, 32))),
			}
			for n, f := range intX {
				intX[n] = f.scale(rr(n))
			}
			for n, f := range intY {
				intY[n] = f.scale(rr(n))
			}
			// ∫XY, ∫X²Y, ∫X³Y: r^(a+b)·[sin²/2], [−cos³/3], [−cos⁴/4].
			intXY := diff(func(s, _ fragmentForm) fragmentForm { return s.pow(t, 2).scale(big.NewRat(1, 2)) }).scale(rr(2))
			intX2Y := diff(func(_, c fragmentForm) fragmentForm { return c.pow(t, 3).scale(big.NewRat(-1, 3)) }).scale(rr(3))
			intX3Y := diff(func(_, c fragmentForm) fragmentForm { return c.pow(t, 4).scale(big.NewRat(-1, 4)) }).scale(rr(4))
			// [Y] = ∫X dθ and [X] = −∫Y dθ.
			area := dth.scale(rr(2)).add(cU.mul(t, intX[1])).add(cV.mul(t, intY[1])).scale(big.NewRat(1, 2))
			mu := cU.pow(t, 2).mul(t, intX[1]).add(cU.mul(t, intX[2]).scale(big.NewRat(2, 1))).add(intX[3]).
				scale(big.NewRat(1, 2))
			mv := cV.pow(t, 2).mul(t, intY[1]).add(cV.mul(t, intY[2]).scale(big.NewRat(2, 1))).add(intY[3]).
				scale(big.NewRat(1, 2))
			muu := cU.pow(t, 3).mul(t, intX[1]).add(cU.pow(t, 2).mul(t, intX[2]).scale(big.NewRat(3, 1))).
				add(cU.mul(t, intX[3]).scale(big.NewRat(3, 1))).add(intX[4]).scale(big.NewRat(1, 3))
			mvv := cV.pow(t, 3).mul(t, intY[1]).add(cV.pow(t, 2).mul(t, intY[2]).scale(big.NewRat(3, 1))).
				add(cV.mul(t, intY[3]).scale(big.NewRat(3, 1))).add(intY[4]).scale(big.NewRat(1, 3))
			// (cU² + 2cU·X + X²)(cV + Y)·X, term by term.
			muv := cU.pow(t, 2).mul(t, cV).mul(t, intX[1]).
				add(cU.pow(t, 2).mul(t, intXY)).
				add(cU.mul(t, cV).mul(t, intX[2]).scale(big.NewRat(2, 1))).
				add(cU.mul(t, intX2Y).scale(big.NewRat(2, 1))).
				add(cV.mul(t, intX[3])).
				add(intX3Y).
				scale(big.NewRat(1, 2))
			// ∫u³ dA about the plane origin, where the centre is (0, 0).
			u3 := intX[5].scale(big.NewRat(1, 4))

			sLo, sHi := s0, s1
			absDth := dth
			if tc.t1 < tc.t0 {
				sLo, sHi = s1, s0
				absDth = dth.scale(big.NewRat(-1, 1))
			}
			axial := absDth.scale(big.NewRat(3, 1)).add(sHi.sub(sLo).scale(rr(1))).scale(rr(1))

			gotArea, ok := circularAreaInterval(seg, anchor)
			require.True(t, ok)
			requireIntervalEncloses(t, "area", gotArea, area)
			gotMU, gotMV, ok := circularFirstMomentInterval(seg, anchor)
			require.True(t, ok)
			requireIntervalEncloses(t, "mu", gotMU, mu)
			requireIntervalEncloses(t, "mv", gotMV, mv)
			gotMUU, gotMUV, gotMVV, ok := circularSecondMomentInterval(seg, anchor)
			require.True(t, ok)
			requireIntervalEncloses(t, "muu", gotMUU, muu)
			requireIntervalEncloses(t, "muv", gotMUV, muv)
			requireIntervalEncloses(t, "mvv", gotMVV, mvv)
			third, ok := circularThirdMomentInterval(seg)
			require.True(t, ok)
			requireIntervalEncloses(t, "u³", third[0], u3)
			gotAxial, ok := circularAxisMomentInterval(seg, ax)
			require.True(t, ok)
			requireIntervalEncloses(t, "axial", gotAxial, axial)
		})
	}
}
