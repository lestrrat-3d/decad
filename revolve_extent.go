package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file answers the extent questions asked OF a finished revolve: how far
// the solid reaches along a direction, and the axis-aligned box containing it.
//
// A revolve's extreme along a direction is a sweep extreme, not a boundary
// vertex: the meridian's own extremes are swept through the angular interval,
// and sweepExtremeBounds brackets where that sweep turns. Every answer is a
// bounded interval charging the frame's rounding, the angular interval's own
// bound, and the meridian bound the walk carries. See
// docs/evaluator-design.md §6.

// extentAlong is the through-all stop's reading of the revolved solid
// (stops.go): its extent interval along an arbitrary world direction g, beside
// the proven displacement extentBoundedAlong states for its two ends. The
// swept radial factor's range over the angular interval turns the extreme into
// a linear functional over the recorded boundary in (z, ρ) — ρ ≥ 0 over the
// region, so the solid's extreme along g is the extreme of wg·z + m·ρ with m at
// its own extreme, and a linear functional's extreme sits on the boundary. An
// end a sweep extreme or a computed arc radius holds only to a bracket
// publishes that bracket's width, and the stop charges it to the level it
// resolves (docs/evaluator-design.md §5/§6).
func (rp revolvePayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	return rp.extentBoundedAlong(context.Background(), g, newFreeformWork())
}

func (rp revolvePayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return rp.extentAlongWork(ctx, g, newFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, the same shape
// prismPayload.extentAlongWork already takes: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement —
// clearance.go's separating-plane short-circuit, which falls back rather than
// fails. A direction whose extreme is held to a bracket rather than proven
// exactly — by the boundary scan, or by the sweep extreme a partial revolution
// reaches through math.Sin/Cos — refuses here rather than publish a held
// coordinate as the one it denotes. A through-all stop instead consumes the
// bounded reading and charges the displacement to its own level (stops.go).
func (rp revolvePayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := rp.extentBoundedAlong(ctx, g, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: the revolved solid's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the reading itself: the interval AND the proven
// half-width its two ends carry. Both mechanisms that can move an end are
// charged here, because this is the ONE reading every consumer takes — the box
// and the through-all stop alike — and a term charged in only one of them lets
// the same coordinate publish as bounded on one path and exact on the other.
//
// The first mechanism is everything axisExtremeContext returns a bound for: the
// boundary-extreme scan's own — a circular candidate sits at a math.Hypot radius
// and can miss the apex the sweep actually reaches — the rounding that scan's
// own gu·u + gv·v arithmetic commits at the section's coordinate magnitude, and
// that reading's own anchor-shift arithmetic (its doc comment states all three).
// The second is the swept radial coefficient's, sweepBoundAlong below. A section
// whose every candidate is exactly representable, read through a direction whose
// own two products and their sum are exact, swept through an extreme this
// direction's own amplitude holds exactly and shifted by an anchor its own
// products and subtraction hold exactly, carries a zero bound — an all-straight
// or recorded-circle meridian under a full revolution about an axis-aligned
// frame reads exactly.
//
// The two mechanisms displace the SAME end and they COMPOSE, so this reading
// sums them; taking the larger would be sound only if they could not both move
// the end the same way, and nothing makes them exclusive. They are displacements
// of different quantities — the scan bounds how far the computed extreme of
// wg·z + m·ρ sits from the true extreme at the HELD coefficient m, while
// sweepBoundAlong bounds how far that held extreme sits from the extreme at the
// TRUE coefficient — so the triangle inequality is the only composition
// available, and the sum is what the end's total displacement obeys.
//
// The sum is per END. The scan's own bound belongs to the end that scan
// produced, and the sweep term belongs to the end whose coefficient it charges
// (mlo for the low end, mhi for the high end), so the two pairs are composed
// separately and the reading publishes the larger total — it states ONE
// half-width covering both ends, and the larger of two per-end totals covers
// each end's own displacement.
//
// A THIRD mechanism displaces both ends the same way and so composes outward
// with that per-end maximum rather than folding into it:
// revolvePayload.frameRoundAllow, base/wg/c0/c1's own displacement from the
// axis frame's proven direction/anchor uncertainty (axisInPlane's
// dUBound/dVBound/aUBound/aVBound — the fields axisMoments already folds into
// the region's moments) and from the placement's own rounding
// (exactIsometryDotRound, bounds.go). It is zero for an axis-aligned frame
// under an identity placement, which is what keeps an ordinary, unplaced
// revolve's box Exact as before.
//
// A FOURTH is this reading's own recombination of those terms into a published
// endpoint — the base + lo and base + hi below, charged exactly by
// exactSumRound. It covers what none of the other three does: a placement whose
// coefficients are every one of them exactly right, whose sum nonetheless
// rounds. It is zero wherever that addition is exactly representable, so an
// unplaced revolve's box keeps its zero bound.
func (rp revolvePayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, float64, error) {
	b := rp.basis()
	base := rp.xform.Apply(b.a3).Dot(g)
	wg := rp.xform.ApplyDir(b.w).Dot(g)
	c0 := rp.xform.ApplyDir(b.e0).Dot(g)
	c1 := rp.xform.ApplyDir(b.e1).Dot(g)
	mlo, mhi := sweepExtremes(c0, c1, rp.phi0, rp.phi1, rp.full)
	hi, hiBound, err := axisExtremeContext(ctx, rp, wg, mhi, true, work)
	if err != nil {
		return 0, 0, 0, err
	}
	lo, loBound, err := axisExtremeContext(ctx, rp, wg, mlo, false, work)
	if err != nil {
		return 0, 0, 0, err
	}
	sweepLo, sweepHi, err := rp.sweepBoundAlong(c0, c1, mlo, mhi, work)
	if err != nil {
		return 0, 0, 0, err
	}
	// outward is the per-end composition: the outward sum of the two terms,
	// through the same absSumUpper every other composed bound in this package
	// takes. A non-finite term answers +Inf rather than folding into a small
	// bound (bounds.go's own rule), since absSumUpper is an arithmetic on
	// magnitudes and states nothing about an absent one.
	outward := func(boundary, sweep float64) float64 {
		if isNonFinite(boundary) || isNonFinite(sweep) {
			return math.Inf(1)
		}
		return absSumUpper(boundary, sweep)
	}
	bound := math.Max(outward(loBound, sweepLo), outward(hiBound, sweepHi))
	frameAllow, err := rp.frameRoundAllow(g, b, base, wg, c0, c1, work)
	if err != nil {
		return 0, 0, 0, err
	}
	// A FOURTH mechanism is the reading's own final summation base + lo (and
	// base + hi), charged exactly against the same two terms by exactSumRound
	// (bounds.go). frameRoundAllow proves base/wg/c0/c1 each right and says
	// nothing about adding them: a pure translation of an axis-aligned revolve
	// leaves all four exactly right and still rounds here. It is charged per END
	// — the two ends are summed from different terms — and composed outward with
	// the per-end maximum above, through the same triangle inequality every
	// other composition in this reading takes.
	loEnd, hiEnd := base+lo, base+hi
	sumAllow := math.Max(
		exactSumRound(loEnd, base, lo),
		exactSumRound(hiEnd, base, hi),
	)
	bound = absSumUpper(bound, frameAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
}

// frameRoundAllow bounds how far base/wg/c0/c1 — the four scalar coefficients
// extentBoundedAlong lifts the boundary and sweep extremes through — can sit
// from the values the axis's TRUE (unrounded) direction/anchor and the
// placement's own EXACT arithmetic would give. Two independent mechanisms
// compose outward:
//
//   - axisAllow: the axis frame's own direction/anchor uncertainty
//     (dUBound/dVBound/aUBound/aVBound, axisInPlane). Every material point of
//     the swept solid is xform.Apply(a3 + w·z + ρ·(e0·cos φ + e1·sin φ)) for
//     some (z, ρ, φ) the recorded boundary and sweep interval admit, with
//     |z|, |ρ| both bounded by envUpper = ax.radialUpper(coordUpper) — the
//     SAME envelope axisFrame.radialUpper already states for ρ, since
//     z = (p−a)·d and ρ = |cross(d, p−a)| are both bounded by |p−a| for a
//     unit d. Perturbing the anchor by its own proven bound moves a3 by at
//     most anchorAllow (through the frame's own unit U/V); perturbing the
//     direction by its own proven bound moves w and e0 by at most dirAllow
//     each (the same construction), and e1 = w×e0 by at most e1Allow (the
//     cross product's own two-term expansion, worst-cased at unit |w|, |e0|).
//     g is always a unit world axis, so a bound on the pre-transform point's
//     own displacement bounds its g-projection too (Cauchy-Schwarz) — the
//     isometry carries a magnitude bound through unchanged, in exact
//     arithmetic.
//   - placeAllow: the placement's own rounding, through
//     exactIsometryDotRound's exact rational check (bounds.go) on each of
//     base/wg/c0/c1 against the SAME frame+placement chain applied to the
//     ALREADY-HELD a3/w/e0/e1 — zero exactly where the placement's own float
//     arithmetic is exact for this input (an identity placement) regardless
//     of how tilted the axis frame itself is. wg's displacement moves the
//     extreme at the rate of |z| ≤ envUpper; c0's and c1's at the rate of
//     |ρ| ≤ envUpper (the swept radial coefficient multiplies ρ); base's
//     displaces the extreme directly, at both ends alike.
func (rp revolvePayload) frameRoundAllow(g r3.Vec, b revolveBasis, base, wg, c0, c1 float64, work *freeformWork) (float64, error) {
	coordUpper, err := profileCoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return 0, err
	}
	ax := rp.ax
	envUpper := ax.radialUpper(coordUpper)
	dirAllow := absSumUpper(ax.dUBound, ax.dVBound)
	e1Allow := absSumUpper(productUpper(2, dirAllow), productUpper(dirAllow, dirAllow))
	anchorAllow := absSumUpper(ax.aUBound, ax.aVBound)
	axisAllow := absSumUpper(
		anchorAllow,
		productUpper(dirAllow, envUpper),
		productUpper(envUpper, absSumUpper(dirAllow, e1Allow)),
	)

	baseRound := exactIsometryDotRound(rp.xform, b.a3, g, true, base)
	wgRound := exactIsometryDotRound(rp.xform, b.w, g, false, wg)
	c0Round := exactIsometryDotRound(rp.xform, b.e0, g, false, c0)
	c1Round := exactIsometryDotRound(rp.xform, b.e1, g, false, c1)
	placeAllow := absSumUpper(
		baseRound,
		productUpper(wgRound, envUpper),
		productUpper(envUpper, absSumUpper(c0Round, c1Round)),
	)

	return absSumUpper(axisAllow, placeAllow), nil
}

// sweepBoundAlong is the swept radial coefficient's own contribution to the
// extent's half-width along one direction: sweepExtremeBounds proves how far
// the held sweep extreme (mlo, mhi) can sit from the true one, and that
// direction error turns into a position error through the same
// directional-perturbation Lipschitz bound (bounds.go) every directional
// extreme charges.
//
// It returns the LOW end's term and the HIGH end's term separately, never their
// larger. Each end's extreme is evaluated at its own held coefficient — the low
// end at mlo, the high end at mhi — so each carries only its own coefficient's
// displacement, and the caller composes it with that same end's boundary-scan
// bound. Folding the two ends together here would hand the caller one number it
// could no longer attribute to an end, and the composition it owes is per end.
//
// The envelope that Lipschitz step charges is the RADIAL one: the extreme's
// own functional is wg·z + m·ρ, so a perturbation of the swept radial
// coefficient m multiplies ρ, the distance from the RESOLVED AXIS, and not the
// profile's coordinates about the frame origin. axisFrame.radialUpper owns that
// envelope and folds in the axis anchor, which is the whole term an offset axis
// adds; an extent whose radial envelope cannot be proven finite is refused
// rather than published against a bound that omits it.
func (rp revolvePayload) sweepBoundAlong(c0, c1, mlo, mhi float64, work *freeformWork) (float64, float64, error) {
	coordUpper, err := profileCoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return 0, 0, err
	}
	rhoUpper := rp.ax.radialUpper(coordUpper)
	if isNonFinite(rhoUpper) {
		return 0, 0, fmt.Errorf(`%w: the revolved region's radial distance from its own axis has no finite proven bound, so no sweep-extreme bound can be composed`, ErrNotFinite)
	}
	loBound, hiBound := sweepExtremeBounds(c0, c1, rp.phi0, rp.phi1, mlo, mhi, rp.full)
	return directionalPerturbationAllow(loBound, rhoUpper),
		directionalPerturbationAllow(hiBound, rhoUpper),
		nil
}

// revolveBoundsContext computes the axis-aligned bounds of the placed
// revolved solid — the same directional-extreme analysis the prism uses, in
// cylindrical coordinates (docs/evaluator-design.md §6). Bounds is Exact only
// where every axis's extent reads with a zero bound; every other reading (a
// partial sweep, an amplitude no float64 holds exactly, a boundary extreme a
// computed arc radius carries, the boundary scan's own gu·u + gv·v arithmetic
// under a non-trivial direction, the axis frame/placement's own rounding, or the
// reading's own summation of those terms into a published endpoint)
// is Approximate with the PROVEN bound its own arithmetic derives.
//
// The box states no error term of its own. Every mechanism that can move an
// end — the sweep extreme's, the boundary scan's candidate positions and its own
// arithmetic (planeDotDecompositionRoundAllow), the axis frame and
// placement's own rounding (frameRoundAllow), and the endpoint summation's
// (exactSumRound) — belongs to the extent reading
// itself (extentBoundedAlong), which every consumer takes, so the box simply
// maxes the three axes' half-widths. Charging one of them here instead would
// leave the same coordinate bounded on this path and exact on the
// through-all stop path.
func revolveBoundsContext(ctx context.Context, rp revolvePayload, work *freeformWork) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	bound := 0.0
	for i, g := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, extentBound, err := rp.extentBoundedAlong(ctx, g, work)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		if extentBound > bound {
			bound = extentBound
		}
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// sweepExtremes returns the range of m(φ) = c0·cos φ + c1·sin φ over the
// sweep interval: endpoints plus the interior critical angles, or the full
// ±amplitude for a whole turn. The held values it returns are what
// extentBoundedAlong evaluates its interval from, and so what the box and the
// through-all stop both publish; sweepExtremeBounds proves how far each can sit
// from the truth without touching either.
func sweepExtremes(c0, c1, phi0, phi1 float64, full bool) (float64, float64) {
	amp := math.Hypot(c0, c1)
	if full {
		return -amp, amp
	}
	m := func(phi float64) float64 { return c0*math.Cos(phi) + c1*math.Sin(phi) }
	lo := math.Min(m(phi0), m(phi1))
	hi := math.Max(m(phi0), m(phi1))
	if amp == 0 {
		return lo, hi
	}
	star := math.Atan2(c1, c0)
	for _, cand := range []float64{star, star + math.Pi} {
		for k := math.Floor((phi0-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= phi1+1e-12; k += 2 * math.Pi {
			phi := cand + k
			if phi < phi0-1e-12 {
				continue
			}
			lo = math.Min(lo, m(phi))
			hi = math.Max(hi, m(phi))
		}
	}
	return lo, hi
}

// sweepExtremeBounds proves how far sweepExtremes' held (heldLo, heldHi) can
// sit from the TRUE min/max of m(φ) = c0·cos φ + c1·sin φ over [phi0, phi1],
// without ever trusting math.Sin/Cos/Atan2/Hypot's accuracy: c0, c1,
// phi0 and phi1 are read as exact rationals (their own float64 bit patterns —
// the same convention sweepExtremes' own callers already take for a sweep
// angle), sin/cos of the endpoints are enclosed by radSinCosInterval
// (normal_bound.go, the Cone normal's own bracket), and the amplitude
// √(c0²+c1²) by the rational square-root brackets circularLengthInterval
// reads an ArcSeg's radius through (ratSqrtDown/ratSqrtUp).
//
// The true extreme over [phi0, phi1] always sits at phi0, at phi1, or at an
// interior critical angle where m′(φ) = −c0·sin φ + c1·cos φ = 0. m′ is
// itself a sinusoid whose zeros are spaced exactly π apart, so an interval
// shorter than π contains AT MOST one: if m′ is proven the same sign at both
// endpoints (a certified sign, from the same enclosures — never a float
// comparison) and the interval's own width is proven under π, no interior
// critical angle can exist and the extreme is provably an endpoint. Where
// that cannot be certified, the enclosure is widened to the global amplitude
// bound (valid for ANY φ, critical or not — a stationary point's own
// contribution is at most second-order past the endpoint reading, so the
// widening this admits stays small whenever an interior critical angle truly
// is close by).
func sweepExtremeBounds(c0, c1, phi0, phi1, heldLo, heldHi float64, full bool) (float64, float64) {
	c0R, c1R := floatRat(c0), floatRat(c1)
	if c0R == nil || c1R == nil {
		return math.Inf(1), math.Inf(1)
	}
	sq := new(big.Rat).Add(new(big.Rat).Mul(c0R, c0R), new(big.Rat).Mul(c1R, c1R))
	ampLoF, ampHiF := ratSqrtDown(sq), ratSqrtUp(sq)
	if isNonFinite(ampLoF) || isNonFinite(ampHiF) {
		return math.Inf(1), math.Inf(1)
	}
	ampLoR, ampHiR := floatRat(ampLoF), floatRat(ampHiF)
	if ampLoR == nil || ampHiR == nil {
		return math.Inf(1), math.Inf(1)
	}
	if full {
		hiIv := interval(ampLoR, ampHiR)
		loIv := interval(new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR))
		return intervalFloatError(loIv, heldLo), intervalFloatError(hiIv, heldHi)
	}
	p0R, p1R := floatRat(phi0), floatRat(phi1)
	if p0R == nil || p1R == nil {
		return math.Inf(1), math.Inf(1)
	}
	sin0, cos0, ok0 := radSinCosInterval(p0R)
	sin1, cos1, ok1 := radSinCosInterval(p1R)
	if !ok0 || !ok1 {
		return math.Inf(1), math.Inf(1)
	}
	m0 := intervalAdd(intervalScale(cos0, c0R), intervalScale(sin0, c1R))
	m1 := intervalAdd(intervalScale(cos1, c0R), intervalScale(sin1, c1R))
	maxRat := func(a, b *big.Rat) *big.Rat {
		if a.Cmp(b) >= 0 {
			return a
		}
		return b
	}
	minRat := func(a, b *big.Rat) *big.Rat {
		if a.Cmp(b) <= 0 {
			return a
		}
		return b
	}
	// The true max is always >= both endpoints' true values (an endpoint is
	// always a candidate), so the lower end of its enclosure never needs
	// widening; likewise the true min's upper end. Only the "far" end of
	// each — where an unexcluded interior critical angle could push it —
	// widens, and only in the non-monotonic branch below.
	hiLo := maxRat(m0.lo, m1.lo)
	hiHi := maxRat(m0.hi, m1.hi)
	loHi := minRat(m0.hi, m1.hi)
	loLo := minRat(m0.lo, m1.lo)

	// m′(φ) = −c0·sin φ + c1·cos φ shares m's own zeros, spaced exactly π
	// apart, so a closed interval narrower than π contains AT MOST one — and
	// if both endpoints proved the SAME sign (equality admitted, since a
	// critical point sitting exactly at an endpoint is that one zero and
	// changes nothing about the OPEN interval between them), that single
	// zero cannot be interior: m′ cannot cross to the opposite sign and back
	// without a second zero, so it holds that one sign throughout and m is
	// monotone on the whole closed interval.
	negC0R := new(big.Rat).Neg(c0R)
	mp0 := intervalAdd(intervalScale(sin0, negC0R), intervalScale(cos0, c1R))
	mp1 := intervalAdd(intervalScale(sin1, negC0R), intervalScale(cos1, c1R))
	width := new(big.Rat).Sub(p1R, p0R)
	widthLessThanPi := width.Cmp(piLower) < 0
	sameNonPos := mp0.hi.Sign() <= 0 && mp1.hi.Sign() <= 0
	sameNonNeg := mp0.lo.Sign() >= 0 && mp1.lo.Sign() >= 0
	monotonic := widthLessThanPi && (sameNonPos || sameNonNeg)
	if !monotonic {
		hiHi = maxRat(hiHi, ampHiR)
		loLo = minRat(loLo, new(big.Rat).Neg(ampHiR))
	}
	hiIv := interval(hiLo, hiHi)
	loIv := interval(loLo, loHi)
	return intervalFloatError(loIv, heldLo), intervalFloatError(hiIv, heldHi)
}

// axisExtremeContext is one extreme of the linear functional wg·z + k·ρ over the
// recorded boundary, evaluated in axis coordinates through the plane-local
// boundary extremes, beside that extreme's own proven bound.
//
// Three mechanisms compose into that bound. The scan's own is nonzero exactly
// where a candidate's own POSITION is not exactly representable — a circular
// candidate's radius or apex (extrude.go's circularExtremeInterval), a computed
// walk endpoint, a free-form span's enclosure — and it is proven over the
// SECTION's coordinates.
//
// The scan's own ARITHMETIC is the second, and it is independent of the first:
// the scan holds each candidate as the float gu·u + gv·v, and that
// multiply-and-sum rounds at the section's coordinate magnitude even where the
// candidate's position is a value the record states verbatim and the first term
// is therefore zero. planeDotDecompositionRoundAllow (bounds.go) charges it at
// the section's own coordinate envelope, which is the only magnitude in this
// reading it scales with — the prism reading's prismDecompositionRoundAllow is
// the same mechanism one sweep coordinate wider, and neither reading ever reads
// the other's term.
//
// The anchor shift below is the third: this reading's own arithmetic — two
// products, their sum, and the subtraction that carries the scan's extreme into
// axis coordinates — and every one of those rounds at the ANCHOR's magnitude
// rather than the section's, so an axis far from the frame origin rounds here
// while the scan reports zero. exactPlaneDotRound and exactSumRound (bounds.go)
// charge exactly what that arithmetic committed, and the anchor's own proven
// uncertainty (axisInPlane's aUBound/aVBound) rides in beside them through the
// direction it is read against.
func axisExtremeContext(ctx context.Context, rp revolvePayload, wg, k float64, wantMax bool, work *freeformWork) (float64, float64, error) {
	gu, gv := rp.ax.planeDirection(wg, k)
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, rp.profile, gu, gv, work, nil)
	if err != nil {
		return 0, 0, err
	}
	coordUpper, err := profileCoordinateEnvelope(rp.profile, work, nil)
	if err != nil {
		return 0, 0, err
	}
	scanAllow := planeDotDecompositionRoundAllow(gu, gv, coordUpper)
	off := gu*rp.ax.aU + gv*rp.ax.aV
	shiftAllow := absSumUpper(
		exactPlaneDotRound(gu, gv, rp.ax.aU, rp.ax.aV, off),
		productUpper(math.Abs(gu), rp.ax.aUBound),
		productUpper(math.Abs(gv), rp.ax.aVBound),
	)
	scan := hi
	if !wantMax {
		scan = lo
	}
	extreme := scan - off
	return extreme, absSumUpper(bound, scanAllow, shiftAllow, exactSumRound(extreme, scan, -off)), nil
}
