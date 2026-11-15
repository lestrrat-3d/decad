package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// tangentArcLoop is the counter-clockwise outline of a 20 mm plate whose
// sides turn into R4.5 corners joined by an R22 end arc, as a sketch solve
// records it: each corner arc's tangent point with the end arc carries the
// solver's rounding, so the left corner's End and the right corner's Start
// sit off radius 4.5 by about 5e-16 mm, and the right corner's End (10, −10)
// sits off the circle its Start states.
func tangentArcLoop() sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	pl := p(-6.914285714285715, -14.27197798664358)
	pr := p(6.914285714285715, -14.27197798664358)
	segs := []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(10, -10), End: p(10, 10), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(10, 10), End: p(-10, 10), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(-10, 10), End: p(-10, -10), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(-5.5, -10), Start: p(-10, -10), End: pl, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(0, 6.61324772583615), Start: pl, End: pr, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(5.5, -10), Start: pr, End: p(10, -10), TStart: 0, TEnd: 1},
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// chamferAdmission runs the occupied-volume admission over loop as a cap-loop
// chamfer moved by amount on both caps; sharp and restate pick the class.
func chamferAdmission(t *testing.T, loop sectionrecord.LoopRecord, amount float64, sharp, restate bool) error {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	var amounts capband.SharpAmounts
	if sharp {
		amounts = func(_ int, walks []survey2d.SideWalk) ([]float64, float64) {
			out := make([]float64, len(walks))
			for i := range out {
				out[i] = amount
			}
			return out, 0
		}
	}
	refusal, err := capband.OccupiedVolumeAdmission(budget, "cap-loop chamfer", []sectionrecord.LoopRecord{loop},
		map[int]bool{0: true}, map[int]bool{0: true},
		func(int, sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.Walks, func() ([]bool, error) { return make([]bool, len(cl.Walks)), nil }, nil
		}, amounts, restate)
	require.NoError(t, err)
	return refusal
}

// TestChamferAdmissionRestatesOffRadiusArcs pins agent-docs/tessellation-reach-design.md
// §7.1 on tangentArcLoop: a cap-loop chamfer, which restates and charges
// near-tangent joins, admits the loop; the exact class refuses its first
// off-radius arc, and a draft band, which charges near-tangent joins but
// restates nothing, refuses it too.
func TestChamferAdmissionRestatesOffRadiusArcs(t *testing.T) {
	t.Parallel()
	loop := tangentArcLoop()
	require.NoError(t, chamferAdmission(t, loop, 0.5, true, true))
	for _, tc := range []struct {
		name           string
		sharp, restate bool
	}{
		{"exact class", false, false},
		{"near-tangent class without restatement", true, false},
	} {
		refusal := chamferAdmission(t, loop, 0.5, tc.sharp, tc.restate)
		require.ErrorContains(t, refusal, "an arc whose recorded end is not on the circle its start states", tc.name)
		require.ErrorContains(t, refusal, "recorded segment", tc.name)
	}
}

// TestChamferAdmissionReadsRestatedCorners pins that a restating band reads
// every corner about the restated centre: a plate with one R5 corner whose
// End sits 2⁻²⁶ mm off its Start's circle joins both its lines exactly
// tangent about the recorded centre, but its restated centre moves about
// 1e-8 mm across the corner's chord, turning each join by about 1.5e-9 rad,
// past the near-tangent class, so the chamfer refuses the first such corner
// by name. Reading corners about the recorded centres (readCorner ignoring
// restate) admits the plate and turns this test red.
func TestChamferAdmissionReadsRestatedCorners(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	right := 20 + math.Ldexp(1, -26)
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(-15, -5), End: p(15, -5), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(15, 0), Start: p(15, -5), End: p(right, 0), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(right, 0), End: p(right, 10), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(right, 10), End: p(-15, 10), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(-15, 10), End: p(-15, -5), TStart: 0, TEnd: 1},
	}}
	refusal := chamferAdmission(t, loop, 0.5, true, true)
	require.ErrorContains(t, refusal, "exact turn exceeds the admitted")
	require.ErrorContains(t, refusal, "between recorded segments 0 and 1")
}

// TestOffRadiusShiftRefusesShortArc pins OffRadiusShift's window gate: an
// arc whose ends sit 5·10⁻⁹ mm apart with its End 2⁻⁴⁰ mm off its Start's
// circle needs a bisector centre about 2·10⁻⁵ mm away, whose turn π·c/r′
// exceeds the least window that chord subtends, so the two windows could run
// different ways round and the bound refuses.
func TestOffRadiusShiftRefusesShortArc(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	arc := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(5, 0), End: p(5+math.Ldexp(1, -40), 5e-9), TStart: 0, TEnd: 1}
	_, ok := capband.OffRadiusShift(arc, 0.5)
	require.False(t, ok)
}

// offRadiusReference is the largest distance, in 256-bit arithmetic over
// samples of φ and τ, between the point arc denotes, C + (r − τ)·e(θ_S + φ·W),
// and the point its restatement on C′ (the ends' bisector point nearest C)
// denotes, C′ + (r′ − τ)·e(θ′_S + φ·W′).
func offRadiusReference(arc sectionrecord.ArcSeg, reach float64) float64 {
	const prec = 256
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	cu, cv := rat(arc.Center.U), rat(arc.Center.V)
	su, sv := rat(arc.Start.U), rat(arc.Start.V)
	eu, ev := rat(arc.End.U), rat(arc.End.V)
	sq := func(u, v *big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(u, u), new(big.Rat).Mul(v, v))
	}
	du, dv := new(big.Rat).Sub(eu, su), new(big.Rat).Sub(ev, sv)
	mu := new(big.Rat).Sub(sq(new(big.Rat).Sub(eu, cu), new(big.Rat).Sub(ev, cv)), sq(new(big.Rat).Sub(su, cu), new(big.Rat).Sub(sv, cv)))
	mu.Quo(mu, new(big.Rat).Mul(big.NewRat(2, 1), sq(du, dv)))
	ru, rv := new(big.Rat).Add(cu, new(big.Rat).Mul(mu, du)), new(big.Rat).Add(cv, new(big.Rat).Mul(mu, dv))
	f := func(q *big.Rat) float64 { v, _ := q.Float64(); return v }
	bf := func(q *big.Rat) *big.Float { return new(big.Float).SetPrec(prec).SetRat(q) }
	radius := func(pu, pv, ou, ov *big.Rat) float64 {
		v, _ := new(big.Float).SetPrec(prec).Sqrt(bf(sq(new(big.Rat).Sub(pu, ou), new(big.Rat).Sub(pv, ov)))).Float64()
		return v
	}
	r, rr := radius(su, sv, cu, cv), radius(su, sv, ru, rv)
	angle := func(pu, pv, ou, ov *big.Rat) float64 {
		return math.Atan2(f(new(big.Rat).Sub(pv, ov)), f(new(big.Rat).Sub(pu, ou)))
	}
	sweep := func(a0, a1 float64) float64 {
		w := a1 - a0
		for w <= 0 {
			w += 2 * math.Pi
		}
		return w
	}
	a0, a1 := angle(su, sv, cu, cv), angle(eu, ev, cu, cv)
	b0, b1 := angle(su, sv, ru, rv), angle(eu, ev, ru, rv)
	w, ww := sweep(a0, a1), sweep(b0, b1)
	worst := 0.0
	for k := range 65 {
		phi := float64(k) / 64
		for _, tau := range []float64{-reach, 0, reach} {
			th, th2 := a0+phi*w, b0+phi*ww
			// The centre offset is exact; the angles and radii are float64
			// reads of 256-bit values, far finer than the shift sampled.
			gu := f(new(big.Rat).Sub(cu, ru)) + (r-tau)*math.Cos(th) - (rr-tau)*math.Cos(th2)
			gv := f(new(big.Rat).Sub(cv, rv)) + (r-tau)*math.Sin(th) - (rr-tau)*math.Sin(th2)
			worst = math.Max(worst, math.Hypot(gu, gv))
		}
	}
	return worst
}

// TestOffRadiusShiftEnclosesTheRestatement pins OffRadiusShift against the
// sampled reference on an arc whose End sits 2⁻³⁰ mm off its Start's circle,
// large enough for float64 sampling to see: the bound covers every sample and
// stays within a small factor of it (about 2.6 times the sampled 9.3e-10 mm
// here). It is zero for an arc whose ends lie on one circle.
func TestOffRadiusShiftEnclosesTheRestatement(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	arc := sectionrecord.ArcSeg{Center: p(-5.5, -10), Start: p(-10, -10), End: p(-5.5, -14.5-math.Ldexp(1, -30)), TStart: 0, TEnd: 1}
	const reach = 0.5
	shift, ok := capband.OffRadiusShift(arc, reach)
	require.True(t, ok)
	ref := offRadiusReference(arc, reach)
	require.Positive(t, ref)
	require.LessOrEqual(t, ref, shift)
	require.Less(t, shift, 8*ref, "the bound stays within a small factor of the sampled shift")

	exact := sectionrecord.ArcSeg{Center: p(-5.5, -10), Start: p(-10, -10), End: p(-5.5, -14.5), TStart: 0, TEnd: 1}
	zero, ok := capband.OffRadiusShift(exact, reach)
	require.True(t, ok)
	require.Zero(t, zero)
}

// TestOffRadiusLoopTerms pins the per-loop terms on tangentArcLoop: two of
// its arcs are restated (the end arc's ends lie on one circle), the shift is
// positive and far below a micrometre, the largest restated radius is the
// corners' 4.5 mm, and the perimeter covers the outline at the reach.
func TestOffRadiusLoopTerms(t *testing.T) {
	t.Parallel()
	terms, ok := capband.OffRadiusLoop(tangentArcLoop(), 0.5)
	require.True(t, ok)
	require.Equal(t, 2, terms.Arcs)
	require.Positive(t, terms.Shift)
	require.Less(t, terms.Shift, 1e-12)
	require.InDelta(t, 4.5, terms.RadiusHi, 1e-12)
	// Three lines of 20 mm and three arcs at 2π·(R + 0.5) at the least.
	require.Greater(t, terms.Perimeter, 60+2*math.Pi*(5+5+22.5))
}
