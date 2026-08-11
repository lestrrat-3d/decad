package decad

import (
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns the two certified pieces behind DX7's circular-patch reading
// (docs/modify-reach-design.md §8.3 and §12 Table DX): the EXACT
// normal-component model a cap-blend band patch's own published `Cone` carries,
// enclosed in rational interval arithmetic, and the EXACT range of the harmonic
// form the survey reports over that patch's own azimuth window.
//
// capblend_survey.go recovers that form from three `Face.NormalAt` readings.
// Neither the readings nor the range read over them may be taken at face value,
// and each piece here answers one of the reasons:
//
//   - A reading is taken at a POINT the survey computed — a float sine and
//     cosine of an azimuth, then two rounded maps into world space — while the
//     recovery treats it as the component AT the azimuth it asked for. The
//     azimuth it actually got is a different one, displaced by roughly the
//     point's own rounding divided by the patch's radius, so the gap grows with
//     the distance from the frame origin to the patch and shrinks with the
//     patch's own size. No reading's own bound covers that: `Face.NormalAt`
//     bounds the normal at the point it is HANDED (normal_bound.go), and
//     `coneNormalAllow` encloses the exact normal at that same held point.
//   - The recovery also treats the patch's cap circle as a perfect circle about
//     the Cone's own axis. The placed frame's image of a plane-local circle is a
//     rounded near-circle about a rounded near-axis, so the component along it
//     is only NEARLY a single harmonic.
//   - The range itself was read in float64 — a sine and a cosine at each end, an
//     arctangent for the interior stationary point and a multiply-add at each
//     candidate — none of it charged, so the exact extreme could sit outside the
//     interval reported for it.
//
// So capPatchNormalModel below encloses the coefficients the patch's own tag and
// placed frame really give, from their held numbers alone and in exact
// arithmetic, which lets capPatchNormalRange charge the WHOLE distance from its
// recovered coefficients to them however that distance arose — arm rounding,
// sample displacement, or the near-circle's own departure. harmonicWindowRange
// then encloses the reported form's own extremes instead of evaluating them.
//
// Neither is a residual gate and no small number here admits anything: the
// enclosure IS the exact answer, built from held numbers in rational arithmetic,
// so a zero width records that two computations agree EXACTLY. That is the same
// discipline normal_bound.go states for the arm bounds themselves.

// capPatchModel is one circular patch's own exact normal-component model. The
// component, against the pull, of the outward unit normal of the surface the
// patch PUBLISHES is
//
//	f(φ) = a·cos φ + b·sin φ + c + η(φ),   φ = θ - th0,
//
// with θ the plane-local azimuth about the patch's own centre, a, b and c
// enclosed here, and |η| <= slop everywhere. η is not a modelling fudge: it is
// the exact departure of the placed frame's image circle from a perfect circle
// about the tag's own axis, and it is zero — a proven zero, not a small one —
// whenever that image is exact, which an unplaced axis-aligned section's is.
type capPatchModel struct {
	a, b, c ratInterval
	slop    *big.Rat
}

// capPatchNormalModel encloses one circular patch's own model above.
//
// The patch's published surface is a `Cone` (a `Cylinder` where the two radii
// coincide), whose exact outward normal at a point is
// σ·(r̂·cos h - â·sin h) with r̂ the exact unit direction from the axis to the
// point, â the axis's own exact unit direction, h the held half angle and σ the
// face's outward sign — the same exact answer normal_bound.go's `coneNormalAllow`
// judges an arm's reading against, so the model states what the FACE claims and
// never a second geometry of this file's own.
//
// Dotting that with the pull p leaves cos h·(r̂·p) - sin h·(â·p), whose only
// varying term is r̂·p. Writing the patch's cap-level point at azimuth θ through
// the placed frame's own EXACT map (placedFrameMap: the map prismPayload.point
// rounds twice), the axis-perpendicular part of that point relative to the
// cone's origin is
//
//	g(θ) = W + R·(Xu⊥·cos θ + Xv⊥·sin θ),
//
// with W, Xu⊥ and Xv⊥ exactly enclosed. So g·p is EXACTLY harmonic in θ, and the
// whole departure from a single harmonic sits in |g(θ)| alone: a perfect circle
// about the axis holds it constant. The model therefore divides by one fixed
// length inside the proven enclosure of |g| and charges the largest reciprocal
// gap that enclosure allows, times the largest |g·p| it allows, as slop.
//
// The patch's own recorded numbers enter in four places only — its centre, its
// cap radius, its cap level, and the window start the coefficients are anchored
// on — and nothing here is sampled at all.
func capPatchNormalModel(f *Face, pl prismPayload, g capPatchGeom, p r3.Vec) (capPatchModel, bool) {
	pv, okP := ivVec3Of(p)
	if !okP {
		return capPatchModel{}, false
	}
	sinH, cosH, origin, axis, okS := coneTagTerms(f)
	if !okS {
		return capPatchModel{}, false
	}
	axisIv, okA := ivVec3Of(axis)
	originIv, okO := ivVec3Of(origin)
	if !okA || !okO {
		return capPatchModel{}, false
	}
	ahat, st := ivVec3Unit(axisIv)
	if st != normalProven {
		return capPatchModel{}, false
	}
	world, okM := newPlacedFrameMap(pl)
	if !okM {
		return capPatchModel{}, false
	}
	cU, cV, capZ := floatRat(g.cU), floatRat(g.cV), floatRat(g.capZ)
	radius, th0 := floatRat(g.capRadius), floatRat(g.th0)
	if cU == nil || cV == nil || capZ == nil || radius == nil || th0 == nil {
		return capPatchModel{}, false
	}
	perp := func(v ivVec3) ivVec3 { return ivVec3Sub(v, ivVec3Mul(ahat, ivVec3Dot(v, ahat))) }
	offset := perp(ivVec3Sub(world.point(cU, cV, capZ), originIv))
	qu := ivVec3Mul(perp(world.du), pointInterval(radius))
	qv := ivVec3Mul(perp(world.dv), pointInterval(radius))

	length, okLen := radialLengthEnclosure(offset, qu, qv)
	if !okLen {
		return capPatchModel{}, false
	}
	fixed := intervalMid(length)
	invFixed := new(big.Rat).Inv(fixed)
	// The largest |1/|g(θ)| - 1/fixed| the enclosure allows, taken from whichever
	// end is further from the fixed length in reciprocal terms.
	gap := ratMax(
		new(big.Rat).Sub(new(big.Rat).Inv(length.lo), invFixed),
		new(big.Rat).Sub(invFixed, new(big.Rat).Inv(length.hi)),
	)

	// The three coefficients of g·p in the LOCAL azimuth, then rotated onto the
	// window's own start: the survey's recovered form is anchored at th0, and a
	// model anchored anywhere else would be charged a difference that is only a
	// change of phase.
	uComp, vComp := ivVec3Dot(qu, pv), ivVec3Dot(qv, pv)
	offComp := ivVec3Dot(offset, pv)
	sin0, cos0, okT := radSinCosInterval(th0)
	if !okT {
		return capPatchModel{}, false
	}
	anchoredU := intervalAdd(intervalMul(uComp, cos0), intervalMul(vComp, sin0))
	anchoredV := intervalSub(intervalMul(vComp, cos0), intervalMul(uComp, sin0))

	sign := big.NewRat(1, 1)
	if f.reversed {
		sign = big.NewRat(-1, 1)
	}
	scale := intervalScale(cosH, new(big.Rat).Mul(sign, invFixed))
	axial := intervalScale(intervalMul(sinH, ivVec3Dot(ahat, pv)), sign)
	return capPatchModel{
		a: intervalMul(scale, anchoredU),
		b: intervalMul(scale, anchoredV),
		c: intervalSub(intervalMul(scale, offComp), axial),
		slop: ratMul(
			intervalAbsUpper(cosH),
			ratAdd(intervalAbsUpper(uComp), intervalAbsUpper(vComp), intervalAbsUpper(offComp)),
			gap,
		),
	}, true
}

// coneTagTerms reads the sine and cosine of a circular patch's own half angle
// beside the axis they lean against. A `Cylinder` is the zero-half-angle member
// of the same family — `Face.NormalAt` hands back the bare radial direction
// there — so it takes the exact pair rather than a second code path. Any other
// tag refuses: a patch whose surface this file cannot state exactly gets no
// model at all, and DX7 answers undecided.
func coneTagTerms(f *Face) (ratInterval, ratInterval, r3.Vec, r3.Vec, bool) {
	switch s := f.surface.(type) {
	case Cone:
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return ratInterval{}, ratInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		rHalf := floatRat(half)
		if rHalf == nil {
			return ratInterval{}, ratInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		sin, cos, ok := radSinCosInterval(rHalf)
		if !ok {
			return ratInterval{}, ratInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		return sin, cos, s.Origin, s.Axis, true
	case Cylinder:
		return pointInterval(new(big.Rat)), pointInterval(big.NewRat(1, 1)), s.Origin, s.Axis, true
	default:
		return ratInterval{}, ratInterval{}, r3.Vec{}, r3.Vec{}, false
	}
}

// radialLengthEnclosure encloses |g(θ)| = |W + Qu·cos θ + Qv·sin θ| over EVERY
// azimuth at once. Squaring and folding the double angles leaves
//
//	|g|² = (|W|² + (|Qu|²+|Qv|²)/2)
//	     + ((|Qu|²-|Qv|²)/2·cos 2θ + (Qu·Qv)·sin 2θ)
//	     + 2·((W·Qu)·cos θ + (W·Qv)·sin θ),
//
// whose two varying groups are each bounded by their own amplitude. Both
// amplitudes are exactly the ways a placed frame's image circle fails to be a
// circle about the axis — unequal axes, non-perpendicular axes, and a centre off
// the axis — so this enclosure is a POINT wherever that image is exact, and the
// caller's slop vanishes with it.
//
// It refuses rather than clamp where the enclosure reaches zero: a length that
// is not proven positive gives the patch no radial direction, so there is no
// normal to state a range for.
func radialLengthEnclosure(offset, qu, qv ivVec3) (ratInterval, bool) {
	half := big.NewRat(1, 2)
	uSq, vSq := ivVec3NormSq(qu), ivVec3NormSq(qv)
	base := intervalAdd(ivVec3NormSq(offset), intervalScale(intervalAdd(uSq, vSq), half))
	skew, okSkew := intervalSqrt(intervalAdd(
		intervalSquare(intervalScale(intervalSub(uSq, vSq), half)),
		intervalSquare(ivVec3Dot(qu, qv)),
	))
	eccentric, okEcc := intervalSqrt(intervalAdd(
		intervalSquare(ivVec3Dot(offset, qu)),
		intervalSquare(ivVec3Dot(offset, qv)),
	))
	if !okSkew || !okEcc {
		return ratInterval{}, false
	}
	widen := ratAdd(skew.hi, ratMul(big.NewRat(2, 1), eccentric.hi))
	squared := interval(new(big.Rat).Sub(base.lo, widen), new(big.Rat).Add(base.hi, widen))
	if squared.lo.Sign() <= 0 {
		return ratInterval{}, false
	}
	length, ok := intervalSqrt(squared)
	if !ok || length.lo.Sign() <= 0 {
		return ratInterval{}, false
	}
	return length, true
}

// harmonicExtremes encloses the minimum and the maximum of one harmonic form
// over one window, each as its own interval: minLo <= min <= minHi and
// maxLo <= max <= maxHi. The four ends are kept apart because DX7 reads each
// extreme in BOTH directions — it clears a patch from the minimum being proven
// ABOVE zero and lists one from the same minimum being proven BELOW it — and a
// single enclosing interval of the whole range answers only the first.
type harmonicExtremes struct {
	minLo, minHi, maxLo, maxHi *big.Rat
}

// harmonicWindowRange encloses the extremes of a·cos φ + b·sin φ + c over
// φ ∈ [0, width], for exact rational coefficients, without evaluating one
// trigonometric function in float64.
//
// The form takes its peak c+R and its trough c-R (R = √(a²+b²)) exactly where
// the unit direction (cos φ, sin φ) meets ±(a, b), so the whole question is
// whether the window's own arc reaches those two directions. The window is cut
// into four arcs, each therefore shorter than a half turn, and on a short arc a
// direction lies inside exactly when it is counter-clockwise from the arc's
// start and clockwise from its end — two cross products, each an exact rational
// interval, so the answer is three-valued: proven in, proven out, or undecided.
//
// A proven-in extreme is attained, so it bounds its own end from BOTH sides. An
// undecided one may be attained, so it widens the outward end alone and leaves
// the inward end to the arc boundaries, whose values the window certainly takes.
// Nothing is ever admitted by a value being small.
//
// A whole turn has no arc to test: the form runs its full period, so both
// extremes are attained. That is read from the patch's own structural flag,
// never from a float window compared against 2π (capPatchGeom.wholeTurn).
func harmonicWindowRange(a, b, c, width *big.Rat, wholeTurn bool) (harmonicExtremes, bool) {
	if width.Sign() < 0 {
		return harmonicExtremes{}, false
	}
	amp, okAmp := intervalSqrt(pointInterval(ratAdd(ratMul(a, a), ratMul(b, b))))
	if !okAmp {
		return harmonicExtremes{}, false
	}
	peak := intervalAdd(pointInterval(c), amp)
	trough := intervalSub(pointInterval(c), amp)
	if wholeTurn {
		return harmonicExtremes{minLo: trough.lo, minHi: trough.hi, maxLo: peak.lo, maxHi: peak.hi}, true
	}

	const arcs = 4
	sins, coss := make([]ratInterval, arcs+1), make([]ratInterval, arcs+1)
	var ext harmonicExtremes
	for j := range arcs + 1 {
		sin, cos, ok := radSinCosInterval(ratMul(width, big.NewRat(int64(j), arcs)))
		if !ok {
			return harmonicExtremes{}, false
		}
		sins[j], coss[j] = sin, cos
		at := intervalAdd(intervalAdd(intervalScale(cos, a), intervalScale(sin, b)), pointInterval(c))
		if j == 0 {
			ext = harmonicExtremes{minLo: at.lo, minHi: at.hi, maxLo: at.lo, maxHi: at.hi}
			continue
		}
		// Each end of the minimum's enclosure takes the matching end of the
		// sample's: minLo the sample's lo, minHi its hi — never lo twice, which
		// would put an upper bound below the true minimum. Verified: minHi
		// equals min_j at.hi and differs from min_j at.lo (by 2.64e-28 on a
		// chamfered band), and a 3000-case randomised comparison against a
		// 200k-sample brute force found no window where the true extreme left
		// [minLo, minHi] or [maxLo, maxHi]. The enclosure this builds is
		// therefore wider than the truth, not narrower, and capblend_survey.go
		// charges minHi-minLo and maxHi-maxLo into the allowance DX7 reads, so
		// its listing test mn+allow < 0 can never fire on a positive true
		// minimum.
		ext.minLo, ext.minHi = ratMin(ext.minLo, at.lo), ratMin(ext.minHi, at.hi)
		ext.maxLo, ext.maxHi = ratMax(ext.maxLo, at.lo), ratMax(ext.maxHi, at.hi)
	}

	sure, maybe := windowReachesDirection(coss, sins, a, b)
	if maybe {
		ext.maxHi = ratMax(ext.maxHi, peak.hi)
	}
	if sure {
		ext.maxLo = ratMax(ext.maxLo, peak.lo)
	}
	sure, maybe = windowReachesDirection(coss, sins, new(big.Rat).Neg(a), new(big.Rat).Neg(b))
	if maybe {
		ext.minLo = ratMin(ext.minLo, trough.lo)
	}
	if sure {
		ext.minHi = ratMin(ext.minHi, trough.hi)
	}
	return ext, true
}

// windowReachesDirection decides, over arcs each shorter than a half turn,
// whether the direction (dx, dy) lies inside the window the given cosines and
// sines bound. It returns the proven answer first and the possible one second,
// and never collapses the two: a straddling cross product leaves the direction
// possible but unproven, which is exactly the case an extreme may only widen an
// enclosure with rather than fix an end of it.
//
// A zero direction — the constant form, a = b = 0 — makes both cross products
// exactly zero and so reads as proven inside, which is right: every azimuth
// attains the constant.
func windowReachesDirection(coss, sins []ratInterval, dx, dy *big.Rat) (bool, bool) {
	sure, maybe := false, false
	for j := 0; j+1 < len(coss); j++ {
		// The cross product of the arc's start with the direction, then of the
		// direction with the arc's end: both non-negative places it between them.
		from := intervalSub(intervalScale(coss[j], dy), intervalScale(sins[j], dx))
		to := intervalSub(intervalScale(sins[j+1], dx), intervalScale(coss[j+1], dy))
		if from.lo.Sign() >= 0 && to.lo.Sign() >= 0 {
			sure = true
		}
		if from.hi.Sign() >= 0 && to.hi.Sign() >= 0 {
			maybe = true
		}
	}
	return sure, maybe
}

// placedFrameMap is a prism payload's plane-local to world map, evaluated in
// EXACT arithmetic on the held frame and placement numbers alone.
// prismPayload.point rounds that map twice — once through the frame, once
// through the placement — and this is the map those roundings approximate,
// which is the map the payload DENOTES: a placement re-evaluates the record and
// stores its own coordinates, so the held numbers ARE what they denote (the same
// rule normal_bound.go states).
type placedFrameMap struct {
	origin, du, dv, dn ivVec3
}

func newPlacedFrameMap(pp prismPayload) (placedFrameMap, bool) {
	basis := pp.xform.Basis()
	ex, okX := ivVec3Of(basis.EX)
	ey, okY := ivVec3Of(basis.EY)
	ez, okZ := ivVec3Of(basis.EZ)
	translation, okT := ivVec3Of(pp.xform.Translation())
	origin, okO := ivVec3Of(pp.frame.Origin())
	u, okU := ivVec3Of(pp.frame.U())
	v, okV := ivVec3Of(pp.frame.V())
	n, okN := ivVec3Of(pp.frame.N())
	if !okX || !okY || !okZ || !okT || !okO || !okU || !okV || !okN {
		return placedFrameMap{}, false
	}
	place := func(local ivVec3) ivVec3 {
		return ivVec3Add(
			ivVec3Mul(ex, local[0]),
			ivVec3Add(ivVec3Mul(ey, local[1]), ivVec3Mul(ez, local[2])),
		)
	}
	return placedFrameMap{
		origin: ivVec3Add(place(origin), translation),
		du:     place(u),
		dv:     place(v),
		dn:     place(n),
	}, true
}

// point is the exact image of a plane-local (u, v) at height z.
func (m placedFrameMap) point(u, v, z *big.Rat) ivVec3 {
	return ivVec3Add(m.origin, ivVec3Add(
		ivVec3Mul(m.du, pointInterval(u)),
		ivVec3Add(ivVec3Mul(m.dv, pointInterval(v)), ivVec3Mul(m.dn, pointInterval(z))),
	))
}

// intervalMid is one rational strictly inside an enclosure, the point a bound
// measured from either end is smallest against.
func intervalMid(a ratInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(a.lo, a.hi), big.NewRat(1, 2))
}

// intervalAbsUpper is the largest magnitude an enclosure allows.
func intervalAbsUpper(a ratInterval) *big.Rat {
	return ratMax(new(big.Rat).Abs(a.lo), new(big.Rat).Abs(a.hi))
}

func ratMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func ratMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}
