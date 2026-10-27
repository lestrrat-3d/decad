package freeform

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file is agent-docs/spline-design.md §6.2's directional-extreme row, Tier A
// (polynomial Bézier) column only: a proven bracket on the min and max of the
// linear functional g(u, v) = gu·u + gv·v over one converted free-form span.
// It reduces to internal/polynomial's certified root engine exactly as the row
// prescribes — reuse it, never fork a second one.
//
// The reduction: P(t) = g·C(t) is itself a polynomial Bézier of the same
// degree as the span (a linear functional of a polynomial curve is a
// polynomial), so its Bernstein coefficients are gu·p.u + gv·p.v, one per
// control point. P's extreme over [0, 1] is attained either at an endpoint —
// exact, a Bézier interpolates its ends — or at an interior root of P'.
// Isolating those roots with polynomial.RpIsolateRootsContext and bracketing P's value
// over each isolating interval via the convex-hull property (BernsteinHull
// of the Bernstein form RESTRICTED to that interval, BernsteinRestrict) is
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

// SpanDirectionalValues returns the Bernstein coefficients of
// P(t) = gu·u(t) + gv·v(t), one per control point of span. A linear
// functional of a Bézier curve is the same-degree Bézier of the functional
// applied to each control point — no conversion, no rounding.
func SpanDirectionalValues(span BezierSpan, gu, gv *big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(span))
	for i, p := range span {
		v := new(big.Rat).Mul(gu, p.U)
		v.Add(v, new(big.Rat).Mul(gv, p.V))
		out[i] = v
	}
	return out
}

// BernsteinSplit is one exact de Casteljau reduction of a Bernstein form at
// parameter t: left holds the diagonal (the restriction to [0, t]) and right
// the anti-diagonal (the restriction to [t, 1]), each re-expressed over its
// own local [0, 1] — the same triangular scheme spline_length.go's
// DyadicSpan.split uses, generalized from a fixed t = 1/2 to an arbitrary
// rational t so it can land on wherever root isolation puts it.
//
// The triangle runs over integers (bernsteinIntSplit) and each output is
// reduced once. Every big.Rat is held in lowest terms with a positive
// denominator, so the exact value fixes its numerator and denominator: the
// outputs equal, as canonical big.Rat values, those of the rational triangle
// work[i] ← (1−t)·work[i] + t·work[i+1].
//
// The outputs share pointers the way that triangle shares them: left[0] is
// values[0], right[n-1] is values[n-1], and right[0] is left[n-1] (the value
// at t). Every other output is a fresh big.Rat.
func BernsteinSplit(values []*big.Rat, t *big.Rat) (left, right []*big.Rat) {
	n := len(values)
	left, right = make([]*big.Rat, n), make([]*big.Rat, n)
	left[0], right[n-1] = values[0], values[n-1]
	work, den := bernsteinIntRow(values)
	split := newBernsteinIntSplit(t)
	for level := 1; level < n; level++ {
		split.level(work[:n-level+1], den)
		left[level] = proofarith.RatFrac(new(big.Rat), work[0], den)
		if level < n-1 {
			right[n-1-level] = proofarith.RatFrac(new(big.Rat), work[n-1-level], den)
		}
	}
	right[0] = left[n-1]
	return left, right
}

// BernsteinRestrict returns the exact Bernstein control values of the same
// polynomial restricted to [a, b] ⊆ [0, 1], by two de Casteljau splits: the
// first isolates [a, 1] (taking its right branch), the second isolates
// [a, b] within it at the reparametrized point (b−a)/(1−a) (taking its left
// branch). a = 1 forces b = 1 too (a ≤ b ≤ 1) and is handled directly — the
// restriction to the single point t = 1 is the polynomial's own endpoint
// value, repeated, and computing it through the general path would divide by
// the zero 1−a.
//
// Both splits run over integers. The right branch of the first split stays
// unreduced: for a row of degree d, entry j leaves the triangle over
// den·q^(d−j) and is scaled by q^j onto the common denominator den·q^d,
// where q is a's denominator. Only the kept branch of the second split is
// reduced, once per output, so the outputs are the canonical big.Rat values
// of the two rational splits. [0, 1] is the identity restriction and returns
// copies of values. A single value is its own restriction and is returned as
// it stands, as the rational splits return it. Every other output is a fresh
// big.Rat. An empty row panics on every path, as the rational splits do.
func BernsteinRestrict(values []*big.Rat, a, b *big.Rat) []*big.Rat {
	n := len(values)
	last := values[n-1]
	one := big.NewRat(1, 1)
	if a.Cmp(one) == 0 {
		out := make([]*big.Rat, n)
		for i := range out {
			out[i] = new(big.Rat).Set(last)
		}
		return out
	}
	if n == 1 {
		return []*big.Rat{values[0]}
	}
	if a.Sign() == 0 && b.Cmp(one) == 0 {
		out := make([]*big.Rat, n)
		for i, v := range values {
			out[i] = new(big.Rat).Set(v)
		}
		return out
	}
	work, den := bernsteinIntRow(values)
	// The right branch of the split at a = 0 is values itself:
	// (1−0)·x + 0·y = x.
	if a.Sign() != 0 {
		first := newBernsteinIntSplit(a)
		for level := 1; level < n; level++ {
			first.level(work[:n-level+1], den)
		}
		// Entry j was last written at level n−1−j, over den·q^(n−1−j)
		// before den reached den·q^(n−1); scaling it by q^j puts it there.
		qPow := new(big.Int).Set(first.q)
		for j := 1; j < n; j++ {
			work[j].Mul(work[j], qPow)
			if j < n-1 {
				qPow.Mul(qPow, first.q)
			}
		}
	}
	span := new(big.Rat).Sub(b, a)
	span.Quo(span, new(big.Rat).Sub(one, a))
	second := newBernsteinIntSplit(span)
	out := make([]*big.Rat, n)
	out[0] = proofarith.RatFrac(new(big.Rat), work[0], den)
	for level := 1; level < n; level++ {
		second.level(work[:n-level+1], den)
		out[level] = proofarith.RatFrac(new(big.Rat), work[0], den)
	}
	return out
}

// bernsteinIntRow returns the coefficients as integer numerators over their
// least common denominator: values[i] = nums[i]/den, den > 0. The numerators
// are fresh, so the split triangles may overwrite them.
func bernsteinIntRow(values []*big.Rat) ([]*big.Int, *big.Int) {
	den := big.NewInt(1)
	var gcd big.Int
	for _, v := range values {
		if den.Cmp(v.Denom()) == 0 {
			continue
		}
		gcd.GCD(nil, nil, den, v.Denom())
		den.Quo(den, &gcd)
		den.Mul(den, v.Denom())
	}
	nums := make([]*big.Int, len(values))
	var scale big.Int
	for i, v := range values {
		scale.Quo(den, v.Denom())
		nums[i] = new(big.Int).Mul(v.Num(), &scale)
	}
	return nums, den
}

// bernsteinIntSplit is one de Casteljau parameter t = p/q, q > 0, as the
// integer weights of a level: (1−t)·x + t·y = ((q−p)·x + p·y)/q.
type bernsteinIntSplit struct {
	q, p, qp *big.Int
	// pWeight and qpWeight are 0 for a zero weight, 1 for a unit weight and
	// 2 otherwise; the level skips a product by 0 or 1 since its exact
	// result is known.
	pWeight, qpWeight int
	scratch           *big.Int
	term              big.Int
}

func newBernsteinIntSplit(t *big.Rat) *bernsteinIntSplit {
	s := &bernsteinIntSplit{
		q:       new(big.Int).Set(t.Denom()),
		p:       new(big.Int).Set(t.Num()),
		scratch: new(big.Int),
	}
	s.qp = new(big.Int).Sub(s.q, s.p)
	s.pWeight, s.qpWeight = intWeight(s.p), intWeight(s.qp)
	return s
}

func intWeight(x *big.Int) int {
	switch {
	case x.Sign() == 0:
		return 0
	case x.IsInt64() && x.Int64() == 1:
		return 1
	default:
		return 2
	}
}

// level runs one level of the triangle over row, whose entries are integer
// numerators over den: row[i] ← (q−p)·row[i] + p·row[i+1] for every i but
// the last, and den ← den·q. The exact value row[i]/den after the level is
// (1−t)·x + t·y for the values x, y at i and i+1 before it. The last entry
// is not updated and so no longer lies over den: callers read it before the
// level, or rescale it as BernsteinRestrict does. The entries are the
// triangle's own integers, never a caller's, so a product may swap an entry
// with the scratch integer instead of copying it back.
func (s *bernsteinIntSplit) level(row []*big.Int, den *big.Int) {
	for i := range len(row) - 1 {
		x, y := row[i], row[i+1]
		switch s.qpWeight {
		case 0:
			x.SetInt64(0)
		case 2:
			s.scratch.Mul(s.qp, x)
			row[i], s.scratch = s.scratch, x
			x = row[i]
		}
		switch s.pWeight {
		case 1:
			x.Add(x, y)
		case 2:
			s.term.Mul(s.p, y)
			x.Add(x, &s.term)
		}
	}
	den.Mul(den, s.q)
}

// BernsteinHull returns the min and max of a Bernstein form's own
// coefficients. The convex-hull property of the Bernstein basis makes this an
// enclosure of the polynomial's values over [0, 1] — the proof spline design
// §6.2 asks for, not an estimate of it.
func BernsteinHull(values []*big.Rat) (*big.Rat, *big.Rat) {
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

// FreeformExtremeCost is the conservative preflight of one span's directional
// extreme bracket, in the shape FreeformSpanCost and FreeformBracketCost
// already use: read off the control count alone, before a single big.Rat
// allocates. Building the monomial derivative, its square-free reduction and
// its Sturm chain each run a further Euclidean-style reduction over the
// degree FreeformSpanCost's own Bernstein-to-monomial conversion already
// charges for, so that conversion's own cost is the base unit and this scales
// it by a constant factor to cover them.
func FreeformExtremeCost(controls int) uint64 {
	return CostMul(FreeformSpanCost(controls), 8)
}

// ErrFreeformExtremeUnrepresentable is agent-docs/spline-design.md §6.2's own
// counterpart to §6.1's R15 (spline_length.go's
// ErrFreeformLengthUnrepresentable): the enclosure is still PROVEN, but no
// float64 interval holds it, so there is nothing to fold and nothing to
// publish. The sentinel is ErrUnsupported — every coordinate reaching the
// bracket is finite, so the curve EXISTS and only this evaluator's float64
// reading of its directional extreme does not — never ErrNotFinite, whose
// subject is a non-finite INPUT, and never ErrDegenerate, which claims no such
// body exists at all.
var ErrFreeformExtremeUnrepresentable = fmt.Errorf(
	`%w: a free-form segment's directional extreme runs past the representable float64 range`, decaderr.ErrUnsupported,
)

// FreeformExtremeFloats is the ONE conversion from a span's exact rational
// enclosure into the float64 interval prismextent.ProfileBoundaryExtremes folds,
// and it is where an enclosure the float64 range cannot state refuses.
//
// Rounding outward is not enough on its own: an enclosure past MaxFloat64
// saturates to an infinity, and an infinity folded into that accumulator is
// read as a candidate nothing contributed — the scan then reports the empty
// region's own ErrDegenerate, inverting the existence claim an agent branches
// on, or publishes an infinite Box bound. The width is checked for the same
// reason the endpoints are: an interval with two finite ends can still be
// wider than the range, and the fold reports a midpoint and a half width, so
// an unrepresentable width has no reading either. Refusing here keeps every
// caller's own free-form refusal intact — prismrecord.Payload.ExtentAlongWork and its
// revolve twin answer ErrUnsupported, exactly as they do for a bracket carrying
// an ordinary nonzero bound, and a through-all stop reading a bounded extent
// gets the same sentinel here rather than an interval it could charge.
func FreeformExtremeFloats(iv polynomial.RatIv) (float64, float64, error) {
	lo, hi := proofbound.RatFloatDown(iv.Lo), proofbound.RatFloatUp(iv.Hi)
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || proofbound.IsNonFinite(hi-lo) {
		return 0, 0, ErrFreeformExtremeUnrepresentable
	}
	return lo, hi, nil
}

// RequireFiniteDirection refuses a non-finite direction component, reading the
// two floats themselves so nothing allocates: it is the ONE owner of both
// messages, and it is what lets every caller gate a direction ahead of the R7
// preflight that must precede the work (agent-docs/spline-design.md §5.2).
func RequireFiniteDirection(gu, gv float64) error {
	if proofbound.IsNonFinite(gu) {
		return fmt.Errorf(`%w: a directional extreme's gu component must be finite`, decaderr.ErrNotFinite)
	}
	if proofbound.IsNonFinite(gv) {
		return fmt.Errorf(`%w: a directional extreme's gv component must be finite`, decaderr.ErrNotFinite)
	}
	return nil
}

// DirectionRats lifts the two direction components into the exact rationals
// SpanDirectionalValues multiplies by. proofbound.RatOf allocates, so this runs only from
// BEHIND a span's own R7 charge; proofbound.RatOf fails exactly on a non-finite float, so
// RequireFiniteDirection names which component it was.
func DirectionRats(gu, gv float64) (*big.Rat, *big.Rat, error) {
	guR, ok := proofbound.RatOf(gu)
	if !ok {
		return nil, nil, RequireFiniteDirection(gu, gv)
	}
	gvR, ok := proofbound.RatOf(gv)
	if !ok {
		return nil, nil, RequireFiniteDirection(gu, gv)
	}
	return guR, gvR, nil
}

// SpanExtremeEnclosureContext is one span's own reading: a proven enclosure
// of its minimum value and a proven enclosure of its maximum value, over the
// linear functional gu·u + gv·v. work is the record's free-form work counter
// (agent-docs/spline-design.md §5.2); the charge is levied first — before the
// direction is lifted into rationals and before the Bernstein coefficients
// that name P are built — so an unaffordable span refuses (R7) before any
// rational allocates. It therefore takes the direction as the two FLOATS its
// caller holds, never as rationals a caller would have had to allocate ahead
// of this charge.
func SpanExtremeEnclosureContext(ctx context.Context, span BezierSpan, gu, gv float64, work *FreeformWork) (polynomial.RatIv, polynomial.RatIv, error) {
	if err := work.Step(FreeformExtremeCost(len(span))); err != nil {
		return polynomial.RatIv{}, polynomial.RatIv{}, err
	}
	if err := ctx.Err(); err != nil {
		return polynomial.RatIv{}, polynomial.RatIv{}, err
	}
	guR, gvR, err := DirectionRats(gu, gv)
	if err != nil {
		return polynomial.RatIv{}, polynomial.RatIv{}, err
	}

	values := SpanDirectionalValues(span, guR, gvR)
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

	stationary := polynomial.RpSquareFree(polynomial.RpDeriv(RpFromBernstein(values)))
	chain, err := polynomial.SturmChainIntContext(ctx, polynomial.RpTrim(stationary))
	if err != nil {
		return polynomial.RatIv{}, polynomial.RatIv{}, err
	}
	ivs, err := polynomial.RpIsolateRootsContext(ctx, stationary, chain)
	if err != nil {
		return polynomial.RatIv{}, polynomial.RatIv{}, err
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	for _, iv := range ivs {
		a, b := iv.Lo, iv.Hi
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
		lo, hi := BernsteinHull(BernsteinRestrict(values, a, b))
		fold(lo, hi)
	}
	return polynomial.RatIv{Lo: minLo, Hi: minHi}, polynomial.RatIv{Lo: maxLo, Hi: maxHi}, nil
}
