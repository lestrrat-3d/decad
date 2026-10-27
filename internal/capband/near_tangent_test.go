package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// nearTangentPair is the bottom line and right arc of a 30 × 10 slot whose arc
// centre sits shift to the right of the line's end b = (15, −5): the arc
// still passes through b and through c = (15, 5), but its tangent at b turns
// by θ, sin θ = shift/√(25 + shift²), off the line it meets.
func nearTangentPair(shift float64) (sectionrecord.LineSeg, sectionrecord.ArcSeg) {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line := sectionrecord.LineSeg{Start: p(-15, -5), End: p(15, -5), TStart: 0, TEnd: 1}
	arc := sectionrecord.ArcSeg{Center: p(15+shift, 0), Start: p(15, -5), End: p(15, 5), TStart: 0, TEnd: 1}
	return line, arc
}

// nearTangentGap is the reference for Lemma 1 of agent-docs/draft-design.md
// §9.1.1 on nearTangentPair at amount t, in 256-bit arithmetic: the distance
// from the radial foot f(t) = b + t·(C − b)/R to the true corner m(t), where
// the moved line v = −5 + t meets the moved circle of radius R − t on the
// branch through b, and the distance from f(t) to that moved line.
func nearTangentGap(shift, t float64) (float64, float64) {
	const prec = 256
	n := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	add := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Add(a, b) }
	sub := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sub(a, b) }
	mul := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Mul(a, b) }
	quo := func(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Quo(a, b) }
	sqrt := func(a *big.Float) *big.Float { return new(big.Float).SetPrec(prec).Sqrt(a) }
	s, tt := n(shift), n(t)
	radius := sqrt(add(n(25), mul(s, s)))
	moved := sub(radius, tt)
	half := sub(n(5), tt)
	mu := sub(add(n(15), s), sqrt(sub(mul(moved, moved), mul(half, half))))
	mv := sub(tt, n(5))
	fu := add(n(15), quo(mul(tt, s), radius))
	fv := add(n(-5), quo(mul(tt, n(5)), radius))
	du, dv := sub(mu, fu), sub(mv, fv)
	gap, _ := sqrt(add(mul(du, du), mul(dv, dv))).Float64()
	off, _ := new(big.Float).Abs(sub(fv, mv)).Float64()
	return gap, off
}

// TestNearTangentCornerEnclosesTheMiter pins Lemma 1 of
// agent-docs/draft-design.md §9.1.1 against the 256-bit reference: at every
// amount the sweep reaches, inward and outward, the true corner lies within ρ
// of the radial foot B1 holds, and that foot within ε of the moved line.
// The bounds sit about twice the reference (ρ = 2.4e-11 against a gap of
// 1.2e-11 at the inward end, ε = 1.4e-21 against 7.1e-22). Shown to fail
// first: with ρ or ε quartered in nearTangentCharge, the matching enclosure
// went red at an amount of 0.26 mm, and zeroed, the positivity check did.
func TestNearTangentCornerEnclosesTheMiter(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -34)
	line, arc := nearTangentPair(shift)
	for _, d := range []float64{0.42, -0.42} {
		const delta = 1e-16
		charge, why, _ := capband.NearTangentCorner(line, arc, d, d, delta)
		require.Empty(t, why)
		require.Positive(t, charge.Rho)
		require.Positive(t, charge.Eps)
		for k := 1; k <= 8; k++ {
			amount := (d + math.Copysign(delta, d)) * float64(k) / 8
			gap, off := nearTangentGap(shift, amount)
			require.LessOrEqual(t, gap, charge.Rho, "amount %g: the true corner lies past ρ", amount)
			require.LessOrEqual(t, off, charge.Eps, "amount %g: the radial foot lies past ε from the moved line", amount)
			require.Positive(t, gap)
		}
	}
}

// TestNearTangentCornerRefuses pins each of §9.1.1's refusals by its reason,
// and the exact-tangent refusal for a corner whose ends differ or whose
// tangents oppose. Deleting the matching gate in NearTangentCorner turns its
// row red.
func TestNearTangentCornerRefuses(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line, arc := nearTangentPair(5 * math.Ldexp(1, -34))
	wide, wideArc := nearTangentPair(5 * math.Ldexp(1, -31))
	shift := 5 * math.Ldexp(1, -34)
	first := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(0, -5), End: p(5, 0), TStart: 0, TEnd: 1}
	second := sectionrecord.ArcSeg{Center: p(2, shift), Start: p(5, 0), End: p(2-shift, 3+shift), TStart: 0, TEnd: 1}
	offEnd := sectionrecord.LineSeg{Start: p(-15, -5), End: p(15, -5+math.Ldexp(1, -40)), TStart: 0, TEnd: 1}
	reversed := sectionrecord.LineSeg{Start: p(15, -5), End: p(-15, -5), TStart: 0, TEnd: 1}
	for _, tc := range []struct {
		name       string
		prev, cur  sectionrecord.CurveSegment
		prevAmount float64
		curAmount  float64
		want       string
	}{
		{"two circular walls", first, second, 0.4, 0.4, "between two circular walls"},
		{"turn past the limit", wide, wideArc, 0.4, 0.4, "exceeds the admitted sin²θ ≤ 2⁻⁶⁴"},
		{"different amounts", line, arc, 0.4, 0, "move by different amounts"},
		{"moved circle at the corner", line, arc, 5 - 1e-12, 5 - 1e-12, "moved circle comes within"},
		{"recorded ends differ", offEnd, arc, 0.4, 0.4, "exactly tangent join"},
		{"tangents oppose", reversed, arc, 0.4, 0.4, "exactly tangent join"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, why, _ := capband.NearTangentCorner(tc.prev, tc.cur, tc.prevAmount, tc.curAmount, 0)
			require.Contains(t, why, tc.want)
		})
	}
	t.Run("an outward amount past the radius stays clear", func(t *testing.T) {
		t.Parallel()
		_, why, _ := capband.NearTangentCorner(line, arc, -6, -6, 0)
		require.Empty(t, why)
	})
}
