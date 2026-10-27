package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// This file pins agent-docs/draft-design.md §9.1.1 at a corner of two arcs:
// Lemma 1′ against a 256-bit reference in each of its four cases, the
// refusals of condition 6 for two arcs, and the survival gate at such a
// corner.

// arcPair is one near-tangent corner of two arcs at v = (5, 0): prev arrives
// there and cur leaves it, the tangents turning by sin θ ≈ shift/3.
type arcPair struct {
	name      string
	prev, cur sectionrecord.ArcSeg
}

// nearTangentArcPairs lists Lemma 1′'s four cases. The radius-5 circle about
// the origin meets a radius-√(9 + shift²) circle whose centre sits shift off
// the axis through v: inside it (internal tangency) walked both
// counter-clockwise or both clockwise, or beside it about (8, ±shift)
// (external tangency, an S bend) walked counter-clockwise into clockwise or
// clockwise into counter-clockwise. Each recorded end lies exactly on its
// circle.
func nearTangentArcPairs(shift float64) []arcPair {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	big5 := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(0, -5), End: p(5, 0), TStart: 0, TEnd: 1}
	inner := sectionrecord.ArcSeg{Center: p(2, shift), Start: p(5, 0), End: p(2-shift, 3+shift), TStart: 0, TEnd: 1}
	reverse := func(a sectionrecord.ArcSeg) sectionrecord.ArcSeg {
		a.TStart, a.TEnd = 1-a.TStart, 1-a.TEnd
		return a
	}
	return []arcPair{
		{"internal, both counter-clockwise", big5, inner},
		{"internal, both clockwise", reverse(inner), reverse(big5)},
		{"external, counter-clockwise into clockwise", big5,
			sectionrecord.ArcSeg{Center: p(8, shift), Start: p(8+shift, 3+shift), End: p(5, 0), TStart: 1, TEnd: 0}},
		{"external, clockwise into counter-clockwise",
			sectionrecord.ArcSeg{Center: p(0, 0), Start: p(5, 0), End: p(0, 5), TStart: 1, TEnd: 0},
			sectionrecord.ArcSeg{Center: p(8, -shift), Start: p(5, 0), End: p(8+shift, -3-shift), TStart: 0, TEnd: 1}},
	}
}

// twoArcGaps is the reference for Lemma 1′ at amount t in 256-bit
// arithmetic: the true corner m(t), where the two moved circles meet nearest
// the leaving arc's radial foot f₂(t), and its distances from f₂(t) and from
// the arriving arc's radial foot f₁(t), with the distance between the feet.
// A counter-clockwise arc's circle moves to R − t, a clockwise one's to R + t.
func twoArcGaps(pair arcPair, t float64) (float64, float64, float64) {
	const prec = 256
	n := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(a, b) }
	sub := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sub(a, b) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	quo := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Quo(a, b) }
	sqrt := func(a *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sqrt(a) }
	type pt struct{ u, v *big.Float }
	dist := func(a, b pt) float64 {
		du, dv := sub(a.u, b.u), sub(a.v, b.v)
		out, _ := sqrt(add(mul(du, du), mul(dv, dv))).Float64()
		return out
	}
	v := pt{n(5), n(0)}
	type circle struct {
		c    pt
		r, m *big.Float // recorded and moved radius
	}
	moved := func(a sectionrecord.ArcSeg) circle {
		c := pt{n(a.Center.U), n(a.Center.V)}
		du, dv := sub(v.u, c.u), sub(v.v, c.v)
		r := sqrt(add(mul(du, du), mul(dv, dv)))
		if a.TStart == 1 {
			return circle{c, r, add(r, n(t))}
		}
		return circle{c, r, sub(r, n(t))}
	}
	foot := func(k circle) pt {
		s := quo(k.m, k.r)
		return pt{add(k.c.u, mul(s, sub(v.u, k.c.u))), add(k.c.v, mul(s, sub(v.v, k.c.v)))}
	}
	c1, c2 := moved(pair.prev), moved(pair.cur)
	f1, f2 := foot(c1), foot(c2)
	wu, wv := sub(c2.c.u, c1.c.u), sub(c2.c.v, c1.c.v)
	d := sqrt(add(mul(wu, wu), mul(wv, wv)))
	a := quo(add(mul(d, d), sub(mul(c1.m, c1.m), mul(c2.m, c2.m))), mul(n(2), d))
	h := sqrt(sub(mul(c1.m, c1.m), mul(a, a)))
	base := pt{add(c1.c.u, quo(mul(a, wu), d)), add(c1.c.v, quo(mul(a, wv), d))}
	off := pt{quo(mul(h, wv), d), quo(mul(h, wu), d)}
	m := pt{sub(base.u, off.u), add(base.v, off.v)}
	if other := (pt{add(base.u, off.u), sub(base.v, off.v)}); dist(other, f2) < dist(m, f2) {
		m = other
	}
	return dist(m, f2), dist(m, f1), dist(f1, f2)
}

// TestNearTangentArcsEncloseTheMiter pins Lemma 1′ of
// agent-docs/draft-design.md §9.1.1 against the 256-bit reference in each of
// its four cases: at every amount the sweep reaches, inward and outward, the
// two moved circles meet within ρ of the leaving arc's radial foot, which B1
// holds, and of the arriving arc's own foot, and the two feet lie within ρ of
// each other. At the full 0.42 mm amount ρ (4.07e-11 mm) is about twice the
// true corner's gap and equal to the feet's, |t|·2 sin(θ/2). Shown to fail
// first: with ρ quartered in nearTangentReach, every case went red at
// 0.1575 mm on the feet, and the corner's gap passes the quartered ρ from
// 0.2625 mm on.
func TestNearTangentArcsEncloseTheMiter(t *testing.T) {
	t.Parallel()
	for _, pair := range nearTangentArcPairs(5 * math.Ldexp(1, -34)) {
		t.Run(pair.name, func(t *testing.T) {
			t.Parallel()
			for _, d := range []float64{0.42, -0.42} {
				const delta = 1e-16
				charge, why, _ := capband.NearTangentCorner(pair.prev, pair.cur, d, d, delta)
				require.Empty(t, why)
				require.True(t, charge.TwoArcs)
				require.Positive(t, charge.Rho)
				require.Zero(t, charge.Eps, "no straight wall meets a corner of two arcs")
				for k := 1; k <= 8; k++ {
					amount := d * float64(k) / 8
					gap2, gap1, feet := twoArcGaps(pair, amount)
					require.Positive(t, gap2)
					require.LessOrEqual(t, gap2, charge.Rho, "amount %g: the true corner lies past ρ from B1's foot", amount)
					require.LessOrEqual(t, gap1, charge.Rho, "amount %g: the true corner lies past ρ from the arriving arc's foot", amount)
					require.LessOrEqual(t, feet, charge.Rho, "amount %g: the two radial feet lie past ρ apart", amount)
				}
			}
		})
	}
}

// TestNearTangentArcsRefuse pins §9.1.1's refusals at a corner of two arcs.
// Past sin θ = 2⁻³² the turn refuses; two amounts refuse; and condition 6
// refuses where either moved circle comes within ρ of the corner: the inner
// counter-clockwise circle moved inward to it, the clockwise circle of the S
// bend moved outward to it, and the arriving clockwise inner circle moved
// outward to it. Shown to fail first: reading every arc as counter-clockwise
// in movedRadius admits both clockwise rows, and checking only the leaving
// circle admits the arriving one's.
func TestNearTangentArcsRefuse(t *testing.T) {
	t.Parallel()
	pairs := nearTangentArcPairs(5 * math.Ldexp(1, -34))
	wide := nearTangentArcPairs(5 * math.Ldexp(1, -31))[0]
	inner := math.Sqrt(9 + 25*math.Ldexp(1, -68))
	for _, tc := range []struct {
		name   string
		pair   arcPair
		prev   float64
		cur    float64
		want   string
		admits bool
	}{
		{name: "turn past the limit", pair: wide, prev: 0.4, cur: 0.4, want: "exceeds the admitted sin²θ ≤ 2⁻⁶⁴"},
		{name: "different amounts", pair: pairs[0], prev: 0.4, cur: 0, want: "move by different amounts"},
		{name: "inner circle moved inward to the corner", pair: pairs[0], prev: inner - 1e-12, cur: inner - 1e-12, want: "do not both clear"},
		{name: "clockwise circle moved outward to the corner", pair: pairs[2], prev: -(inner - 1e-12), cur: -(inner - 1e-12), want: "do not both clear"},
		{name: "arriving circle moved outward to the corner", pair: pairs[1], prev: -(inner - 1e-12), cur: -(inner - 1e-12), want: "do not both clear"},
		{name: "inner circle moved outward past its radius", pair: pairs[0], prev: -4, cur: -4, admits: true},
		{name: "clockwise circle moved inward past its radius", pair: pairs[2], prev: 4, cur: 4, admits: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, why, _ := capband.NearTangentCorner(tc.pair.prev, tc.pair.cur, tc.prev, tc.cur, 0)
			if tc.admits {
				require.Empty(t, why)
				return
			}
			require.Contains(t, why, tc.want)
		})
	}
}

// tinyArcPairLoop is a CCW loop of two arcs: a 2⁻⁴⁰ rad arc about the origin
// from (1 + 2⁻⁴⁰, 1) to (1, 1 + 2⁻⁴⁰), and the near-full arc about
// (0.5, 0.5) back round, which meets it near tangent (sin θ ≈ 2⁻⁴¹) at both
// ends from inside.
func tinyArcPairLoop() sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	e40 := math.Ldexp(1, -40)
	p1, p2 := p(1+e40, 1), p(1, 1+e40)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{Center: p(0, 0), Start: p1, End: p2, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(0.5, 0.5), Start: p2, End: p1, TStart: 0, TEnd: 1},
	}}
}

// TestNearTangentArcsWallMustSurvive pins the survival gate at corners of two
// arcs. The 2⁻⁴⁰ rad arc of tinyArcPairLoop is admitted drafted 0.5 mm
// inward, where the turn its two ends' ρ allow stays inside its window, and
// refused at 0.6 mm, where condition 6 still holds for both circles but that
// turn outruns the window. Shown to fail first: with the gate removed from the
// admission, the 0.6 mm draft admits.
func TestNearTangentArcsWallMustSurvive(t *testing.T) {
	t.Parallel()
	require.NoError(t, draftAdmission(t, tinyArcPairLoop(), 0.5))
	refusal := draftAdmission(t, tinyArcPairLoop(), 0.6)
	require.ErrorIs(t, refusal, decaderr.ErrUnsupported)
	require.ErrorContains(t, refusal, "a circular wall at a near-tangent join")
	require.ErrorContains(t, refusal, "the wall over recorded segments [0]")
	require.ErrorContains(t, refusal, "no proof of the volume")
}
