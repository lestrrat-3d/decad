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
	// The FIFTH mechanism: the meridian this reading scanned is the RECORDED
	// one, and a trimmed section's own cut coordinates sit within its
	// sectionDelta of the meridian the record denotes
	// (docs/surface-intersection-design.md §7.1). It is zero for every payload
	// no construction displaced, which is what leaves an ordinary revolve's box
	// on the path it takes today.
	if sectionAllow := rp.sectionExtentAllow(); sectionAllow > 0 {
		bound = absSumUpper(bound, sectionAllow)
	}
	return loEnd, hiEnd, bound, nil
}

// sectionCoordUpper widens the RECORDED meridian's own coordinate envelope
// into one that covers the meridian the record DENOTES
// (docs/surface-intersection-design.md §7.1). coordUpper is an L1 magnitude
// (segment_walk.go's ratL1Upper), so a point whose two components each move by
// at most sectionDelta adds at most twice it. Every term the axis frame and the
// sweep extreme charge at an envelope has to be charged at THIS one, or those
// two terms would be proven against the recorded meridian while the extreme
// they bound sits on the denoted one.
//
// A payload no construction displaced answers its own argument back, untouched:
// absSumUpper up-rounds, so folding a zero would widen an ordinary revolve's
// every published box by an ulp.
func (rp revolvePayload) sectionCoordUpper(coordUpper float64) float64 {
	if rp.sectionDelta <= 0 {
		return coordUpper
	}
	return absSumUpper(coordUpper, productUpper(2, rp.sectionDelta))
}

// sectionExtentAllow is docs/surface-intersection-design.md §7.1's FIFTH
// mechanism in extentBoundedAlong's own enumeration: how far the extreme of the
// linear functional wg·z + m·ρ moves when the meridian it is taken over is the
// recorded one rather than the denoted one.
//
// A recorded endpoint displaced by at most sectionDelta in each plane-local
// component moves its two AXIS coordinates by at most axisCharge's own two
// figures, each bounded by sectionDelta·(|dU| + |dV|) — the same fold §7.1
// derives for the walk, read here for the envelope rather than per endpoint.
// The functional's own coefficients satisfy wg² + c0² + c1² = 1 for a unit g
// over the orthonormal basis, so |wg| ≤ 1 and |m| ≤ 1 and the extreme moves by
// at most δz + δρ. The held basis departs from orthonormal only by the rounding
// frameRoundAllow already charges, at an envelope sectionCoordUpper widens, so
// that departure is accounted for there rather than doubled here.
//
// It displaces both ends the same way, so it composes OUTWARD with the per-end
// maximum exactly as frameRoundAllow does, never folded into one end's own sum.
func (rp revolvePayload) sectionExtentAllow() float64 {
	if rp.sectionDelta <= 0 {
		return 0
	}
	per := productUpper(rp.sectionDelta, absSumUpper(rp.ax.dU, rp.ax.dV))
	return productUpper(2, per)
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
	envUpper := ax.radialUpper(rp.sectionCoordUpper(coordUpper))
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
	rhoUpper := rp.ax.radialUpper(rp.sectionCoordUpper(coordUpper))
	if isNonFinite(rhoUpper) {
		return 0, 0, fmt.Errorf(`%w: the revolved region's radial distance from its own axis has no finite proven bound, so no sweep-extreme bound can be composed`, ErrNotFinite)
	}
	loBound, hiBound := sweepExtremeBounds(c0, c1, rp.phi0, rp.phi1, rp.den, mlo, mhi, rp.full)
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
// sit from the TRUE min/max of m(φ) = c0·cos φ + c1·sin φ over the sweep the
// record DENOTES (docs/evaluator-design.md §6), without ever trusting
// math.Sin/Cos/Atan2/Hypot's accuracy: c0 and c1 are read as exact rationals
// (their own float64 bit patterns — the same convention sweepExtremes' own
// callers already take), each endpoint's own denoted angle is enclosed by
// den.phi0/den.phi1 (revolve_denotation.go) — falling back to the HELD
// phi0/phi1 read as exact rationals wherever the denotation cannot state one,
// which reproduces today's reading exactly — sin/cos of each denoted angle by
// angleDenotation.sinCosFor, which reads a pure-turn end (a degree-stated
// extent) through turnSinCosInterval, EXACT at every eighth-turn boundary and
// never comparing against π, rather than through the radian-space bracket
// (normal_bound.go's radSinCosInterval, the Cone normal's own primitive) that
// a detour through π would otherwise force even at a quarter turn, and the
// amplitude √(c0²+c1²) by the rational square-root brackets
// circularLengthInterval reads an ArcSeg's radius through (ratSqrtDown/
// ratSqrtUp).
//
// The true extreme over the denoted sweep always sits at phi0, at phi1, or at
// an interior critical angle where m′(φ) = −c0·sin φ + c1·cos φ = 0. m′ is
// itself a sinusoid whose zeros are spaced exactly π apart, so an interval
// shorter than π contains AT MOST one: if m′ is proven the same sign at both
// endpoints (a certified sign, from the same enclosures — never a float
// comparison) and the interval's own width is proven under π, no interior
// critical angle can exist and the extreme is provably an endpoint.
//
// Where that cannot be certified but the interval's width is proven under 2π
// and m′ is proven STRICTLY opposite signs at the two endpoints, the
// π-spacing of m′'s zeros still decides it: a sign change over a span under
// 2π can only cross an ODD number of the (at most two) zeros that fit, so
// exactly ONE interior zero exists, and it is a maximum where m′ runs + to −
// (a minimum where it runs − to +). A critical value of this sinusoid is
// always exactly ±amp, so the true max (min) is provably the amplitude
// itself — the enclosure narrows to [ampLo, ampHi] ([−ampHi, −ampLo]) — and
// with the interval's one critical point already accounted for, the OTHER
// extreme cannot be interior and needs no widening at all.
//
// A width proven at LEAST a half turn (den.halfTurnExcessFor, checked in the
// default arm below) decides both extremes by the endpoints' own certified
// slope signs, with no interior angle to locate: a closed interval of width
// ≥ π always contains at least one critical angle, so for every direction at
// least one of {max, min} is exactly the amplitude, and which one is a fact
// about the SIGN of m′ at whichever endpoint is strict. A strictly positive
// slope at phi0 (or strictly negative at phi1) certifies the max is the
// amplitude; the mirrored sign certifies the min; an endpoint whose slope is
// exactly zero is itself a critical angle, so its own value is one extreme
// and the opposite kind's critical angle sits exactly π further inside the
// interval — both extremes are the amplitude. An extreme none of this
// certifies keeps the global widening, which is sound for any φ.
//
// Only where neither arm certifies — a straddling endpoint, or a width not
// proven under 2π — does the enclosure widen to the global amplitude bound on
// both ends (valid for ANY φ, critical or not).
func sweepExtremeBounds(c0, c1, phi0, phi1 float64, den sweepDenotation, heldLo, heldHi float64, full bool) (float64, float64) {
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
	enc0, ok0 := den.phi0.enclosureFor(phi0)
	enc1, ok1 := den.phi1.enclosureFor(phi1)
	if !ok0 || !ok1 {
		return math.Inf(1), math.Inf(1)
	}
	sin0, cos0, ok0t := den.phi0.sinCosFor(phi0)
	sin1, cos1, ok1t := den.phi1.sinCosFor(phi1)
	if !ok0t || !ok1t {
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
	// widens, and only in the arms below that cannot certify a tighter
	// answer.
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
	widthIv := intervalSub(enc1, enc0)
	widthLessThanPi := widthIv.hi.Cmp(piLower) < 0
	sameNonPos := mp0.hi.Sign() <= 0 && mp1.hi.Sign() <= 0
	sameNonNeg := mp0.lo.Sign() >= 0 && mp1.lo.Sign() >= 0
	monotonic := widthLessThanPi && (sameNonPos || sameNonNeg)
	switch {
	case monotonic:
		// No interior critical angle at all: both endpoint enclosures stand
		// as they are.
	case widthIv.hi.Cmp(twoPiInterval().lo) < 0 && mp0.lo.Sign() > 0 && mp1.hi.Sign() < 0:
		// m′ runs strictly + to strictly −: exactly one interior zero, a
		// maximum. The true max IS the amplitude, and the min cannot be
		// interior, so it keeps its endpoint-only enclosure.
		hiLo, hiHi = ampLoR, ampHiR
	case widthIv.hi.Cmp(twoPiInterval().lo) < 0 && mp0.hi.Sign() < 0 && mp1.lo.Sign() > 0:
		// The mirror case: a minimum.
		loLo, loHi = new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR)
	default:
		// The reflex arm: a sweep proven at least a half turn wide contains
		// at least one zero of m′ in its CLOSED interval, and each endpoint's
		// own certified slope says which. With α the angular distance from
		// the maximum's direction to phi0, m′(phi0) = −amp·sin α: a strictly
		// positive slope at phi0 puts α in (π, 2π), so the maximum lies
		// within 2π − α < π ≤ width AFTER phi0; a strictly negative one puts
		// α in (0, π), so the minimum lies within π − α < π ≤ width after
		// phi0. Mirrored at phi1 (the extreme lies BEFORE it): a strictly
		// negative slope there certifies the maximum, a strictly positive one
		// the minimum. An endpoint whose slope is exactly zero IS one
		// critical point, so its own value is one extreme and the opposite
		// extreme's critical point sits exactly π further in, inside a
		// width of at least π: both extremes are ±amp. A certified extreme
		// takes the amplitude bracket; an uncertified one keeps the global
		// widening below, which is sound for any φ.
		excess, okExcess := den.halfTurnExcessFor(phi0, phi1)
		atLeastHalfTurn := okExcess && excess.lo.Sign() >= 0
		crit0 := mp0.lo.Sign() == 0 && mp0.hi.Sign() == 0
		crit1 := mp1.lo.Sign() == 0 && mp1.hi.Sign() == 0
		maxIsAmp := atLeastHalfTurn && (crit0 || crit1 || mp0.lo.Sign() > 0 || mp1.hi.Sign() < 0)
		minIsAmp := atLeastHalfTurn && (crit0 || crit1 || mp0.hi.Sign() < 0 || mp1.lo.Sign() > 0)
		if maxIsAmp {
			hiLo, hiHi = ampLoR, ampHiR
		} else {
			hiHi = maxRat(hiHi, ampHiR)
		}
		if minIsAmp {
			loLo, loHi = new(big.Rat).Neg(ampHiR), new(big.Rat).Neg(ampLoR)
		} else {
			loLo = minRat(loLo, new(big.Rat).Neg(ampHiR))
		}
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
