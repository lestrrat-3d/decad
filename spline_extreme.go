package decad

import (
	"context"
	"math"
	"math/big"
)

// This file is docs/spline-design.md §6.2's directional-extreme row, Tier A
// (polynomial Bézier) column only: a proven bracket on the min and max of the
// linear functional g(u, v) = gu·u + gv·v over one converted free-form span.
// It reduces to clearance_poly.go's certified root engine exactly as the row
// prescribes — reuse it, never fork a second one.
//
// The reduction: P(t) = g·C(t) is itself a polynomial Bézier of the same
// degree as the span (a linear functional of a polynomial curve is a
// polynomial), so its Bernstein coefficients are gu·p.u + gv·p.v, one per
// control point. P's extreme over [0, 1] is attained either at an endpoint —
// exact, a Bézier interpolates its ends — or at an interior root of P'.
// Isolating those roots with rpIsolateRootsContext and bracketing P's value
// over each isolating interval via the convex-hull property (bernsteinHull
// of the Bernstein form RESTRICTED to that interval, bernsteinRestrict) is
// the Lipschitz step the row asks for, made exact rather than estimated: the
// restricted hull encloses P over the WHOLE isolating interval, so it
// encloses the critical value wherever inside it the root actually sits.
//
// Folding every endpoint's exact value and every isolated interval's hull
// together, the minimum of the LOWER ends of every candidate's own enclosure
// is a proven lower bound on P's true minimum (every candidate's true value
// is at least its own lower end, and the true minimum equals some candidate's
// true value), and the minimum of the UPPER ends is a proven upper bound on
// it (every candidate's true value is at most its own upper end, including
// the one attaining the minimum) — symmetrically for the maximum. That is
// exactly the shipped capBlendPayload.extentBoundedAlong fold, reused
// verbatim at the per-span level here.

// spanDirectionalValues returns the Bernstein coefficients of
// P(t) = gu·u(t) + gv·v(t), one per control point of span. A linear
// functional of a Bézier curve is the same-degree Bézier of the functional
// applied to each control point — no conversion, no rounding.
func spanDirectionalValues(span bezierSpan, gu, gv *big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(span))
	for i, p := range span {
		v := new(big.Rat).Mul(gu, p.u)
		v.Add(v, new(big.Rat).Mul(gv, p.v))
		out[i] = v
	}
	return out
}

// bernsteinSplit is one exact de Casteljau reduction of a Bernstein form at
// parameter t: left holds the diagonal (the restriction to [0, t]) and right
// the anti-diagonal (the restriction to [t, 1]), each re-expressed over its
// own local [0, 1] — the same triangular scheme spline_length.go's
// dyadicSpan.split uses, generalized from a fixed t = 1/2 to an arbitrary
// rational t so it can land on wherever root isolation puts it.
func bernsteinSplit(values []*big.Rat, t *big.Rat) (left, right []*big.Rat) {
	n := len(values)
	work := make([]*big.Rat, n)
	copy(work, values)
	left = make([]*big.Rat, 0, n)
	right = make([]*big.Rat, n)
	left = append(left, work[0])
	right[n-1] = work[n-1]
	oneMinusT := new(big.Rat).Sub(big.NewRat(1, 1), t)
	for round := n - 1; round > 0; round-- {
		for i := range round {
			blend := new(big.Rat).Mul(oneMinusT, work[i])
			blend.Add(blend, new(big.Rat).Mul(t, work[i+1]))
			work[i] = blend
		}
		left = append(left, work[0])
		right[round-1] = work[round-1]
	}
	return left, right
}

// bernsteinRestrict returns the exact Bernstein control values of the same
// polynomial restricted to [a, b] ⊆ [0, 1], by two de Casteljau splits: the
// first isolates [a, 1] (taking its right branch), the second isolates
// [a, b] within it at the reparametrized point (b−a)/(1−a) (taking its left
// branch). a = 1 forces b = 1 too (a ≤ b ≤ 1) and is handled directly — the
// restriction to the single point t = 1 is the polynomial's own endpoint
// value, repeated, and computing it through the general path would divide by
// the zero 1−a.
func bernsteinRestrict(values []*big.Rat, a, b *big.Rat) []*big.Rat {
	one := big.NewRat(1, 1)
	if a.Cmp(one) == 0 {
		out := make([]*big.Rat, len(values))
		last := values[len(values)-1]
		for i := range out {
			out[i] = new(big.Rat).Set(last)
		}
		return out
	}
	_, right := bernsteinSplit(values, a)
	span := new(big.Rat).Sub(b, a)
	span.Quo(span, new(big.Rat).Sub(one, a))
	left, _ := bernsteinSplit(right, span)
	return left
}

// bernsteinHull returns the min and max of a Bernstein form's own
// coefficients. The convex-hull property of the Bernstein basis makes this an
// enclosure of the polynomial's values over [0, 1] — the proof spline design
// §6.2 asks for, not an estimate of it.
func bernsteinHull(values []*big.Rat) (*big.Rat, *big.Rat) {
	lo, hi := new(big.Rat).Set(values[0]), new(big.Rat).Set(values[0])
	for _, v := range values[1:] {
		if v.Cmp(lo) < 0 {
			lo.Set(v)
		}
		if v.Cmp(hi) > 0 {
			hi.Set(v)
		}
	}
	return lo, hi
}

// freeformExtremeCost is the conservative preflight of one span's directional
// extreme bracket, in the shape freeformSpanCost and freeformBracketCost
// already use: read off the control count alone, before a single big.Rat
// allocates. Building the monomial derivative, its square-free reduction and
// its Sturm chain each run a further Euclidean-style reduction over the
// degree freeformSpanCost's own Bernstein-to-monomial conversion already
// charges for, so that conversion's own cost is the base unit and this scales
// it by a constant factor to cover them.
func freeformExtremeCost(controls int) uint64 {
	return costMul(freeformSpanCost(controls), 8)
}

// ratFloatDown and ratFloatUp are proven outward roundings of an exact
// rational to float64: the largest float64 at or below q, and the smallest
// at or above it. big.Rat.Float64() already rounds to nearest, which can
// land on EITHER side of q, so each checks its own direction against the
// float converted back to a rational and only steps once via Nextafter when
// that check fails — an exactly representable q comes back UNCHANGED, which
// is what keeps a candidate whose true value is exact from picking up a
// spurious width it never earned.
func ratFloatDown(q *big.Rat) float64 {
	f, _ := q.Float64()
	if isNonFinite(f) {
		return f
	}
	if fr, ok := ratOf(f); ok && fr.Cmp(q) <= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(-1))
}

func ratFloatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if isNonFinite(f) {
		return f
	}
	if fr, ok := ratOf(f); ok && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

// spanExtremeEnclosureContext is one span's own reading: a proven enclosure
// of its minimum value and a proven enclosure of its maximum value, over the
// linear functional gu·u + gv·v. work is the record's free-form work counter
// (docs/spline-design.md §5.2); the charge is levied first, before the
// Bernstein coefficients that name P are built, so an unaffordable span
// refuses (R7) before any rational allocates.
func spanExtremeEnclosureContext(ctx context.Context, span bezierSpan, gu, gv *big.Rat, work *freeformWork) (ratIv, ratIv, error) {
	if err := work.step(freeformExtremeCost(len(span))); err != nil {
		return ratIv{}, ratIv{}, err
	}
	if err := ctx.Err(); err != nil {
		return ratIv{}, ratIv{}, err
	}

	values := spanDirectionalValues(span, gu, gv)
	first, last := values[0], values[len(values)-1]
	minLo, minHi := new(big.Rat).Set(first), new(big.Rat).Set(first)
	maxLo, maxHi := new(big.Rat).Set(first), new(big.Rat).Set(first)
	fold := func(lo, hi *big.Rat) {
		if lo.Cmp(minLo) < 0 {
			minLo = lo
		}
		if hi.Cmp(minHi) < 0 {
			minHi = hi
		}
		if lo.Cmp(maxLo) > 0 {
			maxLo = lo
		}
		if hi.Cmp(maxHi) > 0 {
			maxHi = hi
		}
	}
	fold(last, last)

	stationary := rpSquareFree(rpDeriv(rpFromBernstein(values)))
	ivs, err := rpIsolateRootsContext(ctx, stationary)
	if err != nil {
		return ratIv{}, ratIv{}, err
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	for _, iv := range ivs {
		a, b := iv.lo, iv.hi
		if a.Cmp(zero) < 0 {
			a = zero
		}
		if b.Cmp(one) > 0 {
			b = one
		}
		if a.Cmp(b) > 0 {
			// The isolated interval lies entirely outside [0, 1] — a root of
			// the extended polynomial the recorded span never reaches.
			continue
		}
		lo, hi := bernsteinHull(bernsteinRestrict(values, a, b))
		fold(lo, hi)
	}
	return ratIv{lo: minLo, hi: minHi}, ratIv{lo: maxLo, hi: maxHi}, nil
}
