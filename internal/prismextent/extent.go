package prismextent

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/r3"
)

// PlacementCoeffAllow bounds how far base/gu/gv/gz — the four scalar
// coefficients extentBoundedAlong lifts the boundary and sweep extremes
// through — can sit from the value the SAME frame-and-placement chain's exact
// arithmetic would give. An identity placement read along a world axis uses
// only exact zero-or-one products and additions for these coefficients; other
// cases use proofbound.ExactIsometryDotRound's rational check (internal/proofbound/bounds.go). Each
// coefficient's own displacement moves the published
// extreme at the rate of the coordinate it multiplies —
// proofbound.DirectionalPerturbationAllow's own Lipschitz shape, coordUpper for gu/gv and
// zUpper for gz — while base's displaces the extreme directly, at both ends
// alike, since it is the section's own constant offset under this direction.
// capBlendPayload.extentBoundedAlong reuses this unchanged through
// prismLike's shared frame and placement. A second, independent term
// (DecompositionRoundAllow) covers the rounding this reading's own
// DECOMPOSITION gu*u + gv*v + gz*z commits even when every coefficient is
// itself exactly right: multiplying an EXACT but non-trivial coefficient by a
// recorded coordinate still rounds, and grouping the sum this way (the
// boundary scan's own gu*u+gv*v first) is a DIFFERENT float computation than
// applying the frame and placement to the point directly, even though the two
// are equal in exact arithmetic. The recombination with base that FOLLOWS is a
// third term, charged exactly at the call site rather than here
// (proofbound.ExactSumRound), because it rounds for placements this function's own check
// proves exact — a translation is committed there and nowhere else.
func PlacementCoeffAllow(xform r3.Transform, frame r3.Frame, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	var baseRound, guRound, gvRound, gzRound float64
	if xform != r3.Identity() || !worldAxis(g) {
		baseRound = proofbound.ExactIsometryDotRound(xform, frame.Origin(), g, true, base)
		guRound = proofbound.ExactIsometryDotRound(xform, frame.U(), g, false, gu)
		gvRound = proofbound.ExactIsometryDotRound(xform, frame.V(), g, false, gv)
		gzRound = proofbound.ExactIsometryDotRound(xform, frame.N(), g, false, gz)
	}
	return proofbound.AbsSumUpper(
		baseRound,
		proofbound.DirectionalPerturbationAllow(guRound, coordUpper),
		proofbound.DirectionalPerturbationAllow(gvRound, coordUpper),
		proofbound.DirectionalPerturbationAllow(gzRound, zUpper),
		DecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper),
	)
}

// With an identity placement and a world axis, Apply and Dot multiply each
// frame component only by zero or one. Their additions include only zero, so
// every coefficient equals the exact rational result of those operations.
func worldAxis(g r3.Vec) bool {
	return g == r3.NewVec(1, 0, 0) || g == r3.NewVec(0, 1, 0) || g == r3.NewVec(0, 0, 1)
}

// DecompositionRoundAllow bounds the rounding the MULTIPLY-AND-SUM
// combination base + gu*u + gv*v + gz*z commits, given base/gu/gv/gz
// themselves proven exact against the isometry that produced them
// (PlacementCoeffAllow's own proofbound.ExactIsometryDotRound check, above). IEEE
// 754 multiplies exactly by 0, 1 or -1 for ANY operand — those three values
// are the only ones that never round a multiply — so a coefficient outside
// that set can round when it multiplies a recorded coordinate, and this
// reading's own left-to-right summation order (the boundary-extreme scan's
// gu*u+gv*v first, then +base, then +gz*z — a DIFFERENT grouping than
// applying the frame and placement to the point directly) can round again on
// top of that even where every individual multiply happens not to. Both are
// genuinely new roundings a non-axis-permuting frame commits, so the term is
// zero where every coefficient is trivial — and otherwise reuses
// proofbound.AnalyticRoundBound's own established "a bounded number of basic ops at a
// magnitude" contract: at most 3 multiplies and 3 additions here, far under
// its 128-operation budget. |base| stays in that envelope because the same
// left-to-right evaluation folds base in, and charging it in both arms is only
// ever wider than the non-trivial arm owes.
//
// The trivial arm's zero speaks for the DECOMPOSITION alone, never for the
// whole endpoint. g is a unit world axis and the frame orthonormal, so
// gu²+gv²+gz² = 1 and an all-trivial reading has exactly one coefficient at
// ±1 with the other two at 0: every multiply is exact and so is the scan's own
// gu*u+gv*v, whatever the coordinates are. What that argument does NOT reach
// is the recombination with base and the sweep level, which rounds for a
// coefficient set this arm calls trivial — a pure translation is exactly that
// case — so extentBoundedAlong charges it separately and exactly through
// proofbound.ExactSumRound (internal/proofbound/bounds.go), and this term must never be read as covering it.
func DecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	trivial := func(c float64) bool { return c == 0 || c == 1 || c == -1 }
	if trivial(gu) && trivial(gv) && trivial(gz) {
		return 0
	}
	// The non-trivial arm is loose, and it is loose in the safe direction: one
	// fixture measures both halves of that. A 5x5 mm section swept 1 mm on the
	// UNPLACED frame U=(0.6,0.8,0), V=(-0.8,0.6,0) reports Min=(-4,0,0),
	// Max=(3,7,1), Approximate, bound 3.9790393202565666e-13. That whole figure
	// is this term: divide it by proofbound.AnalyticRoundBound's own 256*proofbound.UnitRoundoff and
	// it comes to 14 up to that helper's outward rounding, which is the scale
	// below — |gu|*coordUpper + |gv|*coordUpper = 0.6*10 + 0.8*10, the walk's
	// coordUpper being 10 for that section — while every sibling term answers
	// zero: proofbound.ExactIsometryDotRound on base/gu/gv/gz under the identity xform,
	// proofbound.ExactSumRound on base 0 with levels 0 and 1, and both displacement terms
	// prismBoundsContext composes. A zero bound on that fixture would be
	// unsound rather than tighter, because the extremes it would call exact are
	// not: summing this frame's OWN held entries over the rationals against the
	// section corners {0,5}^2 gives X-min = -5*float64(0.8), X-max =
	// 5*float64(0.6) and Y-max = 5*float64(0.6) + 5*float64(0.8), none of them
	// representable, since float64(0.8) sits above 4/5 and 5*float64(0.8) needs
	// 55 significand bits — it is exactly 4 + 2^-52, a QUARTER of the 2^-50 ulp
	// above 4, which ordinary round-to-nearest returns as 4 with no tie. Each of
	// the three published coordinates therefore misses its true extreme by a
	// representable amount (2.22e-16 in X-min, 1.11e-16 in X-max and Y-max),
	// X-min and Y-max landing INSIDE the true extreme and X-max landing outward
	// of it, a miss this term covers with wide margin.
	scale := proofbound.AbsSumUpper(
		proofbound.ProductUpper(math.Abs(gu), coordUpper),
		proofbound.ProductUpper(math.Abs(gv), coordUpper),
		proofbound.ProductUpper(math.Abs(gz), zUpper),
		math.Abs(base),
	)
	return proofbound.AnalyticRoundBound(scale)
}

// circularExtremeInterval encloses the two EXACT extremes of the functional
// g(u, v) = gu·u + gv·v over the whole circle a circular walk lies on. Writing
// the walk as c + r·(cos θ, sin θ) gives g(θ) = (gu·cU + gv·cV) + r·|(gu, gv)|·
// cos(θ − θ*), so the circle's own minimum and maximum are P ∓ r·|(gu, gv)| —
// an identity in which the angle does not appear at all.
//
// That is why the scan's circular candidate is bounded from here rather than
// from a certified sine and cosine at the candidate's own angle: the angle is a
// SELECTION (does the walk sweep the apex?), while the VALUE the fold publishes
// is this closed form, and reading it this way charges no π-rounding guard for
// an angle that never enters the answer. The three inputs that are not exact
// leaves each enter through the bounded arithmetic: the walk's radius under its
// own proven bound (survey2d.SegmentWalk.radiusBound — an ArcSeg states Start and Center,
// so its radius is a math.Hypot), the direction's magnitude through
// proofbound.BoundedNorm2's certified square-root brackets, and every product and sum
// through proofbound.BoundedMul/proofbound.BoundedAdd's own exact rounding terms. An exactly
// representable apex therefore still reports a zero bound, which is what lets a
// recorded circle's box stay Exact along an axis whose reading the apex holds —
// the walk's own endpoints answer for themselves there
// (survey2d.SegmentWalk.startBound/endBound).
func circularExtremeInterval(w survey2d.SegmentWalk, gu, gv float64) (proofbound.BoundedScalar, proofbound.BoundedScalar) {
	gmag := proofbound.BoundedNorm2(proofbound.ExactScalar(gu), proofbound.ExactScalar(gv))
	centre := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.ExactScalar(gu), proofbound.ExactScalar(w.CU)),
		proofbound.BoundedMul(proofbound.ExactScalar(gv), proofbound.ExactScalar(w.CV)),
	)
	amplitude := proofbound.BoundedMul(proofbound.MeasuredScalar(w.Radius, w.RadiusBound), gmag)
	return proofbound.BoundedSub(centre, amplitude), proofbound.BoundedAdd(centre, amplitude)
}

// BoundaryExtremesBoundedContext is the one scan, total over survey2d.WalkKind
// (docs/spline-design.md §6.2): the min and max of g(u, v) = gu·u + gv·v over
// the recorded region's boundary, AND the proven half-width every CANDIDATE's
// own position contributes to that interval.
//
// That half-width is the candidates' POSITIONAL displacement alone. The scan
// evaluates each candidate as the float gu·u + gv·v, and the rounding of that
// multiply-and-sum is the CALLER's to charge, at the coordinate envelope the
// caller's own geometry states: a prism reads it through
// DecompositionRoundAllow, a revolve through
// proofbound.PlaneDotDecompositionRoundAllow (internal/proofbound/bounds.go), and a caller that charges
// neither publishes a candidate the record states verbatim — a zero-width one,
// on which this scan reports zero — as if the arithmetic reading it had
// committed nothing.
//
// An ENDPOINT candidate is the walk's own endpoint read through the direction
// the caller holds, which this evaluator reads as an exact leaf throughout (the
// convention internal/survey2d/survey2d.go's own file comment states). The endpoint itself is an
// exact leaf only where the record STATES it — a line's or an arc's natural
// bounds — and there the candidate has zero width, so an all-straight section's
// reading stays exact. Every other endpoint is one this evaluator computed, and
// the walk states what it is worth (survey2d.SegmentWalk.startBound/endBound);
// proofbound.PointPerturbationAllow carries that displacement through the functional so
// the candidate enters at the width its own construction owes, never at zero.
// An endpoint whose bound no arithmetic could state refuses the whole scan
// rather than folding an infinity into the accumulators.
//
// An interior CIRCULAR candidate is a second computed reading: its position is
// the walk's radius times a cosine and a sine, and the radius itself is a
// math.Hypot for every ArcSeg, so it enters the fold under the proven enclosure
// circularExtremeInterval derives — the single owner of that term, charged where
// the candidate is produced rather than beside the fold by whichever consumer
// noticed. A free-form walk folds each of its converted spans' own proven
// enclosure (freeform.SpanExtremeEnclosureContext) and takes no endpoint candidate of its
// own: a span enclosure already covers the span's whole parameter range,
// endpoints included.
//
// The fold is the shipped capBlendPayload.extentBoundedAlong idiom: track the
// lower and upper ends of every candidate contributing to the region minimum
// (loLower/loUpper) and to the region maximum (hiLower/hiUpper) separately, so
// a candidate that loses the extremization contributes nothing to the reported
// bound, and report the midpoint of each composed interval with the larger of
// the two half widths, rounded up — the same convention freeform.FreeformArcLength
// already uses. The fold is sound because every candidate interval encloses a
// value the true boundary actually attains: the reported minimum's lower end is
// the least of the candidates' lower ends and so never exceeds the truth, and
// its upper end is the least of their upper ends, which the candidate attaining
// the true minimum keeps at or above it.
//
// A span enclosure that convention cannot state in float64 refuses at the
// conversion rather than entering the fold (internal/freeform/spline_extreme.go's
// freeform.FreeformExtremeFloats, Table R row R18), so every number these accumulators
// hold is finite and the only reading left to the empty-region check below is
// a region that genuinely contributed no candidate.
//
// The direction is carried through this scan as the two FLOATS the caller
// holds, and it is gated by freeform.RequireFiniteDirection, which reads them and
// allocates nothing. The rational lift each span's Bernstein coefficients need
// happens inside freeform.SpanExtremeEnclosureContext, behind that span's own R7 charge
// — §5.2's rule is that every charge is levied before the work allocates, and
// a rational built here would allocate ahead of every charge this scan makes.
//
// counts gives each recorded loop's segment count. read returns that segment's
// walk in loop and segment order. The root adapter validates that any supplied
// pre-resolved walks belong to the same recorded profile before calling here.
func BoundaryExtremesBoundedContext(ctx context.Context, gu, gv float64, work *freeform.FreeformWork, counts []int, read func(li, si int) (survey2d.SegmentWalk, error)) (float64, float64, float64, error) {
	loLower, loUpper := math.Inf(1), math.Inf(1)
	hiLower, hiUpper := math.Inf(-1), math.Inf(-1)
	takeLo := func(l, h float64) {
		loLower = math.Min(loLower, l)
		loUpper = math.Min(loUpper, h)
	}
	takeHi := func(l, h float64) {
		hiLower = math.Max(hiLower, l)
		hiUpper = math.Max(hiUpper, h)
	}
	// take folds one candidate's held value under the proven bound its own
	// generator derived. A zero bound enters as the held value twice: widening
	// an exact candidate by a directed rounding would mint an error the
	// arithmetic provably did not commit. A nonzero one is stepped outward with
	// math.Nextafter rather than proofbound.UpRound/freeform.DownRound, since a directional value
	// can be negative and those two only move a POSITIVE bound toward zero.
	take := func(g, allow float64) {
		if allow == 0 {
			takeLo(g, g)
			takeHi(g, g)
			return
		}
		lo := math.Nextafter(g-allow, math.Inf(-1))
		hi := math.Nextafter(g+allow, math.Inf(1))
		takeLo(lo, hi)
		takeHi(lo, hi)
	}
	takeVertex := func(u, v float64, bound proofbound.WalkEndBound) {
		take(gu*u+gv*v, proofbound.PointPerturbationAllow(bound, gu, gv))
	}
	// Every span enclosure enters the fold through freeform.FreeformExtremeFloats
	// (internal/freeform/spline_extreme.go), which rounds outward through proofbound.RatFloatDown/proofbound.RatFloatUp
	// — never freeform.DownRound/proofbound.UpRound: a directional value can be negative, and those
	// only ever move a POSITIVE bound toward zero (internal/freeform/spline_length.go's
	// arc-length-only convention), the wrong direction for a negative
	// candidate and a spurious one-ulp widening of an exactly representable
	// value either way — and refuses decaderr.ErrUnsupported for an enclosure the
	// float64 range cannot state, so no infinity ever reaches these
	// accumulators.

	for li, count := range counts {
		for si := range count {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
			w, err := read(li, si)
			if err != nil {
				return 0, 0, 0, err
			}
			if w.Kind == survey2d.WalkFreeform {
				for _, span := range w.Spans {
					minIv, maxIv, err := freeform.SpanExtremeEnclosureContext(ctx, span, gu, gv, work)
					if err != nil {
						return 0, 0, 0, err
					}
					minLo, minHi, err := freeform.FreeformExtremeFloats(minIv)
					if err != nil {
						return 0, 0, 0, err
					}
					maxLo, maxHi, err := freeform.FreeformExtremeFloats(maxIv)
					if err != nil {
						return 0, 0, 0, err
					}
					takeLo(minLo, minHi)
					takeHi(maxLo, maxHi)
				}
				continue
			}
			if !w.StartBound.Derivable() || !w.EndBound.Derivable() {
				return 0, 0, 0, fmt.Errorf(`%w: a boundary segment's walked endpoint states no proven displacement, so this scan cannot bound the region's extremes`, decaderr.ErrUnsupported)
			}
			takeVertex(w.StartU, w.StartV, w.StartBound)
			takeVertex(w.EndU, w.EndV, w.EndBound)
			if !w.IsCircular() {
				continue
			}
			// Interior extremes at θ* where the functional's gradient
			// aligns with the radius: θ* = atan2(gv, gu) (+π). The angle
			// SELECTS which of the circle's two extremes the walk sweeps;
			// circularExtremeInterval states what that extreme is worth.
			gmag := math.Hypot(gu, gv)
			if gmag == 0 {
				continue
			}
			minIv, maxIv := circularExtremeInterval(w, gu, gv)
			star := math.Atan2(gv, gu)
			tlo, thi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
			for ci, cand := range [2]float64{star, star + math.Pi} {
				apex := maxIv
				if ci == 1 {
					apex = minIv
				}
				for k := math.Floor((tlo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= thi+1e-12; k += 2 * math.Pi {
					th := cand + k
					if th < tlo-1e-12 {
						continue
					}
					held := gu*(w.CU+w.Radius*math.Cos(th)) + gv*(w.CV+w.Radius*math.Sin(th))
					take(held, proofbound.BoundedFloatError(apex, held))
				}
			}
		}
	}
	if math.IsInf(loLower, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, decaderr.ErrDegenerate)
	}
	loMid := loLower + (loUpper-loLower)/2
	hiMid := hiLower + (hiUpper-hiLower)/2
	bound := proofbound.UpRound(math.Max(
		math.Max(loMid-loLower, loUpper-loMid),
		math.Max(hiMid-hiLower, hiUpper-hiMid),
	))
	return loMid, hiMid, bound, nil
}
