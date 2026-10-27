package capband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
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
// row red, and reading every arc as counter-clockwise turns both clockwise
// rows red.
func TestNearTangentCornerRefuses(t *testing.T) {
	t.Parallel()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line, arc := nearTangentPair(5 * math.Ldexp(1, -34))
	wide, wideArc := nearTangentPair(5 * math.Ldexp(1, -31))
	shift := 5 * math.Ldexp(1, -34)
	first := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(0, -5), End: p(5, 0), TStart: 0, TEnd: 1}
	second := sectionrecord.ArcSeg{Center: p(2, shift), Start: p(5, 0), End: p(2-shift, 3+shift), TStart: 0, TEnd: 1}
	// The slot's top line and right arc, walked clockwise as a hole wall: the
	// arc's material lies outside its circle, so an outward amount shrinks it.
	cwLine := sectionrecord.LineSeg{Start: p(-15, 5), End: p(15, 5), TStart: 0, TEnd: 1}
	cwArc := sectionrecord.ArcSeg{Center: p(15+shift, 0), Start: p(15, -5), End: p(15, 5), TStart: 1, TEnd: 0}
	cwRadius := math.Sqrt(25 + shift*shift)
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
		{"clockwise circle moved outward to the corner", cwLine, cwArc, -(cwRadius - 1e-12), -(cwRadius - 1e-12), "moved circle comes within"},
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
	t.Run("a clockwise circle moved inward past its radius stays clear", func(t *testing.T) {
		t.Parallel()
		_, why, _ := capband.NearTangentCorner(cwLine, cwArc, 6, 6, 0)
		require.Empty(t, why)
	})
}

// wedgeLoop is a CCW loop whose bottom wall (0, 0) → (length, 0), recorded
// segment 0, meets a mitered corner at its start and a near-tangent right arc
// at its end: the arc's centre (length + shift, 5) sits shift past the wall's
// end. The top wall (length, 10) → (−1, 10) meets the arc near-tangent too,
// and the side (−1, 10) → (0, 0) closes it with two convex miters.
func wedgeLoop(length, shift float64) sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	a, b, c, d := p(0, 0), p(length, 0), p(length, 10), p(-1, 10)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(length+shift, 5), Start: b, End: c, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}
}

// wedgeAdmission runs the occupied-volume admission over wedgeLoop.
func wedgeAdmission(t *testing.T, length, shift, amount float64) error {
	t.Helper()
	return draftAdmission(t, wedgeLoop(length, shift), amount)
}

// draftAdmission runs the occupied-volume admission over loop as a draft band
// moved by amount on every walk.
func draftAdmission(t *testing.T, loop sectionrecord.LoopRecord, amount float64) error {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	refusal, err := capband.OccupiedVolumeAdmission(budget, "tapered extrude", []sectionrecord.LoopRecord{loop},
		map[int]bool{}, map[int]bool{0: true},
		func(int, sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.Walks, func() ([]bool, error) { return make([]bool, len(cl.Walks)), nil }, nil
		},
		func(_ int, walks []survey2d.SideWalk) ([]float64, float64) {
			amounts := make([]float64, len(walks))
			for i := range amounts {
				amounts[i] = amount
			}
			return amounts, 1e-16
		})
	require.NoError(t, err)
	return refusal
}

// TestNearTangentWallMustSurvive pins Lemma 2's survival gate
// (NearTangentWallRefusal): a wall at a near-tangent join must advance by more
// than its corners' ρ at the near and the far section. The wedge drafted
// inward 0.5 mm keeps its unit bottom wall and is admitted. Drafted inward
// 2 mm, the start miter overruns the wall at the far section. With the bottom
// wall 2⁻³⁶ mm long and an outward draft, the far wall grows but the near one
// is shorter than the near-tangent corner's ρ (about 5.8e-11 mm), while a
// 2⁻³⁰ mm wall outruns it. Removing the gate from the admission admits both
// refusals.
func TestNearTangentWallMustSurvive(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -33)
	require.NoError(t, wedgeAdmission(t, 1, shift, 0.5))
	require.NoError(t, wedgeAdmission(t, math.Ldexp(1, -30), shift, -0.5), "a 2⁻³⁰ mm wall outruns ρ")
	for _, tc := range []struct {
		name           string
		length, amount float64
	}{
		{"far wall overrun", 1, 2},
		{"near wall within the charge", math.Ldexp(1, -36), -0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			refusal := wedgeAdmission(t, tc.length, shift, tc.amount)
			require.ErrorIs(t, refusal, decaderr.ErrUnsupported)
			require.ErrorContains(t, refusal, "advance at the near or far section does not exceed the near-tangent corner charge")
			require.ErrorContains(t, refusal, "no proof of the volume")
		})
	}
}

// riserLoop is a CCW loop whose short riser, recorded segment 2, leaves a
// near-tangent quarter arc at its start and ends at a mitered corner. The
// bottom wall (0, 0) → (10, 0) meets the arc, centre (10 + shift, 5), near
// tangent; the arc ends at (15 + shift, 5 − shift), off the bottom wall's
// axis, and the riser climbs length mm from there before turning to (0, 10)
// and back down to the origin.
func riserLoop(length, shift float64) sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	a, b := p(0, 0), p(10, 0)
	c := p(15+shift, 5-shift)
	d := p(15+shift, 5-shift+length)
	e := p(0, 10)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(10+shift, 5), Start: b, End: c, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: d, End: e, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: e, End: a, TStart: 0, TEnd: 1},
	}}
}

// tinyArcLoop is a CCW loop whose arc about the origin spans 2⁻⁴⁰ rad, from
// (1 + 2⁻⁴⁰, 1) to (1, 1 + 2⁻⁴⁰), between two long walls that meet it near
// tangent, sin θ ≈ 7.3e-12, at both ends: moved 1 mm inward, the true corners
// slide along the arc past each other and its window reverses, while both
// straight walls survive.
func tinyArcLoop() sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	e40, e36 := math.Ldexp(1, -40), math.Ldexp(1, -36)
	p0 := p(2+e40, -e40-e36)
	p1 := p(1+e40, 1)
	p2 := p(1, 1+e40)
	p3 := p(-e40, 2+e40-e36)
	q := p(-2, -2)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p0, End: p1, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(0, 0), Start: p1, End: p2, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p2, End: p3, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p3, End: q, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: q, End: p0, TStart: 0, TEnd: 1},
	}}
}

// nearTangentSlotLoop closes nearTangentPair into the 30 × 10 slot: its right
// arc, a little past a half turn, meets both lines near tangent, and its left
// arc meets them exactly tangent.
func nearTangentSlotLoop(shift float64) sectionrecord.LoopRecord {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	line, arc := nearTangentPair(shift)
	return sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		line, arc,
		sectionrecord.LineSeg{Start: p(15, 5), End: p(-15, 5), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(-15, 0), Start: p(-15, 5), End: p(-15, -5), TStart: 0, TEnd: 1},
	}}
}

// TestNearTangentWallSurvivalBothEnds pins the survival gate where the
// near-tangent join sits at a wall's start and on a circular wall. The riser
// leaves its arc near tangent: a unit riser drafted 0.5 mm inward is admitted;
// drafted 3 mm inward, its end miter overruns it at the far section, which
// reading the arc's start foot for its end foot would hide; a 2⁻³⁶ mm riser
// drafted outward is shorter than its start corner's ρ, which reading that ρ
// as zero would hide. tinyArcLoop's 2⁻⁴⁰ rad window is shorter than the turn
// its two ends' ρ allow at a 1 mm draft. The slot's right arc passes a half
// turn: drafted to within 6.2e-10 mm of its radius it is admitted, and to
// within 6.0e-10 mm, where condition 6 still holds (ρ ≈ 5.8e-10 mm), the
// half-turn branch refuses it. Removing each mutation's target turns its row
// red, and removing the gate admits every refusal.
func TestNearTangentWallSurvivalBothEnds(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -33)
	require.NoError(t, draftAdmission(t, riserLoop(1, shift), 0.5))
	require.NoError(t, draftAdmission(t, nearTangentSlotLoop(shift), 5-6.2e-10))
	for _, tc := range []struct {
		name   string
		loop   sectionrecord.LoopRecord
		amount float64
		want   string
		wall   string
	}{
		{"riser overrun at the far section", riserLoop(1, shift), 3, "a straight wall at a near-tangent join", "[2]"},
		{"riser within its start charge", riserLoop(math.Ldexp(1, -36), shift), -0.5, "a straight wall at a near-tangent join", "[2]"},
		{"arc window within its ends' charge", tinyArcLoop(), 1, "a circular wall at a near-tangent join", "[1]"},
		{"arc past a half turn within its ends' charge", nearTangentSlotLoop(shift), 5 - 6e-10, "a circular wall at a near-tangent join", "[1]"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			refusal := draftAdmission(t, tc.loop, tc.amount)
			require.ErrorIs(t, refusal, decaderr.ErrUnsupported)
			require.ErrorContains(t, refusal, tc.want)
			require.ErrorContains(t, refusal, "the wall over recorded segments "+tc.wall)
		})
	}
}
