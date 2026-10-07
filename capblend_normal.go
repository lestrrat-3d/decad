package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
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
//     bounds the normal at the point it is HANDED (internal/surfacenormal), and
//     `surfacenormal.ConeAllow` encloses the exact normal at that same held point.
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
// discipline internal/surfacenormal states for the arm bounds themselves.

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
	a, b, c proofbound.RatInterval
	slop    *big.Rat
}

// capPatchNormalModel encloses one circular patch's own model above.
//
// The patch's published surface is a `Cone` (a `Cylinder` where the two radii
// coincide), whose exact outward normal at a point is
// σ·(r̂·cos h - â·sin h) with r̂ the exact unit direction from the axis to the
// point, â the axis's own exact unit direction, h the held half angle and σ the
// face's outward sign — the same exact answer `surfacenormal.ConeAllow`
// judges an arm's reading against, so the model states what the FACE claims and
// never a second geometry of this file's own.
//
// Dotting that with the pull p leaves cos h·(r̂·p) - sin h·(â·p), whose only
// varying term is r̂·p. Writing the patch's cap-level point at azimuth θ through
// the placed frame's own EXACT map (survey2d.PlacedFrameMap: the map prismPayload.point
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
	pv, okP := survey2d.IvVec3Of(p)
	if !okP {
		return capPatchModel{}, false
	}
	sinH, cosH, origin, axis, okS := coneTagTerms(f)
	if !okS {
		return capPatchModel{}, false
	}
	axisIv, okA := survey2d.IvVec3Of(axis)
	originIv, okO := survey2d.IvVec3Of(origin)
	if !okA || !okO {
		return capPatchModel{}, false
	}
	ahat, st := surfacenormal.UnitVec3(axisIv)
	if st != surfacenormal.Proven {
		return capPatchModel{}, false
	}
	world, okM := newPlacedFrameMap(pl)
	if !okM {
		return capPatchModel{}, false
	}
	cU, cV, capZ := proofarith.FloatRat(g.cU), proofarith.FloatRat(g.cV), proofarith.FloatRat(g.capZ)
	radius, th0 := proofarith.FloatRat(g.capRadius), proofarith.FloatRat(g.th0)
	if cU == nil || cV == nil || capZ == nil || radius == nil || th0 == nil {
		return capPatchModel{}, false
	}
	perp := func(v survey2d.IvVec3) survey2d.IvVec3 {
		return survey2d.IvVec3Sub(v, survey2d.IvVec3Mul(ahat, survey2d.IvVec3Dot(v, ahat)))
	}
	offset := perp(survey2d.IvVec3Sub(world.Point(cU, cV, capZ), originIv))
	qu := survey2d.IvVec3Mul(perp(world.Du), proofbound.PointInterval(radius))
	qv := survey2d.IvVec3Mul(perp(world.Dv), proofbound.PointInterval(radius))

	length, okLen := radialLengthEnclosure(offset, qu, qv)
	if !okLen {
		return capPatchModel{}, false
	}
	fixed := intervalMid(length)
	invFixed := new(big.Rat).Inv(fixed)
	// The largest |1/|g(θ)| - 1/fixed| the enclosure allows, taken from whichever
	// end is further from the fixed length in reciprocal terms.
	gap := survey2d.RatMax(
		new(big.Rat).Sub(new(big.Rat).Inv(length.Lo), invFixed),
		new(big.Rat).Sub(invFixed, new(big.Rat).Inv(length.Hi)),
	)

	// The three coefficients of g·p in the LOCAL azimuth, then rotated onto the
	// window's own start: the survey's recovered form is anchored at th0, and a
	// model anchored anywhere else would be charged a difference that is only a
	// change of phase.
	uComp, vComp := survey2d.IvVec3Dot(qu, pv), survey2d.IvVec3Dot(qv, pv)
	offComp := survey2d.IvVec3Dot(offset, pv)
	sin0, cos0, okT := survey2d.RadSinCosInterval(th0)
	if !okT {
		return capPatchModel{}, false
	}
	anchoredU := proofbound.IntervalAdd(proofbound.IntervalMul(uComp, cos0), proofbound.IntervalMul(vComp, sin0))
	anchoredV := proofbound.IntervalSub(proofbound.IntervalMul(vComp, cos0), proofbound.IntervalMul(uComp, sin0))

	sign := big.NewRat(1, 1)
	if f.reversed {
		sign = big.NewRat(-1, 1)
	}
	scale := proofbound.IntervalScale(cosH, new(big.Rat).Mul(sign, invFixed))
	axial := proofbound.IntervalScale(proofbound.IntervalMul(sinH, survey2d.IvVec3Dot(ahat, pv)), sign)
	return capPatchModel{
		a: proofbound.IntervalMul(scale, anchoredU),
		b: proofbound.IntervalMul(scale, anchoredV),
		c: proofbound.IntervalSub(proofbound.IntervalMul(scale, offComp), axial),
		slop: proofbound.RatMul(
			proofbound.IntervalAbsUpper(cosH),
			proofbound.RatAdd(proofbound.IntervalAbsUpper(uComp), proofbound.IntervalAbsUpper(vComp), proofbound.IntervalAbsUpper(offComp)),
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
func coneTagTerms(f *Face) (proofbound.RatInterval, proofbound.RatInterval, r3.Vec, r3.Vec, bool) {
	switch s := f.surface.(type) {
	case Cone:
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		rHalf := proofarith.FloatRat(half)
		if rHalf == nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		sin, cos, ok := survey2d.RadSinCosInterval(rHalf)
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
		}
		return sin, cos, s.Origin, s.Axis, true
	case Cylinder:
		return proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1)), s.Origin, s.Axis, true
	default:
		return proofbound.RatInterval{}, proofbound.RatInterval{}, r3.Vec{}, r3.Vec{}, false
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
func radialLengthEnclosure(offset, qu, qv survey2d.IvVec3) (proofbound.RatInterval, bool) {
	half := big.NewRat(1, 2)
	uSq, vSq := survey2d.IvVec3NormSq(qu), survey2d.IvVec3NormSq(qv)
	base := proofbound.IntervalAdd(survey2d.IvVec3NormSq(offset), proofbound.IntervalScale(proofbound.IntervalAdd(uSq, vSq), half))
	skew, okSkew := survey2d.IntervalSqrt(proofbound.IntervalAdd(
		survey2d.IntervalSquare(proofbound.IntervalScale(proofbound.IntervalSub(uSq, vSq), half)),
		survey2d.IntervalSquare(survey2d.IvVec3Dot(qu, qv)),
	))
	eccentric, okEcc := survey2d.IntervalSqrt(proofbound.IntervalAdd(
		survey2d.IntervalSquare(survey2d.IvVec3Dot(offset, qu)),
		survey2d.IntervalSquare(survey2d.IvVec3Dot(offset, qv)),
	))
	if !okSkew || !okEcc {
		return proofbound.RatInterval{}, false
	}
	widen := proofbound.RatAdd(skew.Hi, proofbound.RatMul(big.NewRat(2, 1), eccentric.Hi))
	squared := proofbound.Interval(new(big.Rat).Sub(base.Lo, widen), new(big.Rat).Add(base.Hi, widen))
	if squared.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, false
	}
	length, ok := survey2d.IntervalSqrt(squared)
	if !ok || length.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, false
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
	amp, okAmp := survey2d.IntervalSqrt(proofbound.PointInterval(proofbound.RatAdd(proofbound.RatMul(a, a), proofbound.RatMul(b, b))))
	if !okAmp {
		return harmonicExtremes{}, false
	}
	peak := proofbound.IntervalAdd(proofbound.PointInterval(c), amp)
	trough := proofbound.IntervalSub(proofbound.PointInterval(c), amp)
	if wholeTurn {
		return harmonicExtremes{minLo: trough.Lo, minHi: trough.Hi, maxLo: peak.Lo, maxHi: peak.Hi}, true
	}

	const arcs = 4
	sins, coss := make([]proofbound.RatInterval, arcs+1), make([]proofbound.RatInterval, arcs+1)
	var ext harmonicExtremes
	for j := range arcs + 1 {
		sin, cos, ok := survey2d.RadSinCosInterval(proofbound.RatMul(width, big.NewRat(int64(j), arcs)))
		if !ok {
			return harmonicExtremes{}, false
		}
		sins[j], coss[j] = sin, cos
		at := proofbound.IntervalAdd(proofbound.IntervalAdd(proofbound.IntervalScale(cos, a), proofbound.IntervalScale(sin, b)), proofbound.PointInterval(c))
		if j == 0 {
			ext = harmonicExtremes{minLo: at.Lo, minHi: at.Hi, maxLo: at.Lo, maxHi: at.Hi}
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
		ext.minLo, ext.minHi = survey2d.RatMin(ext.minLo, at.Lo), survey2d.RatMin(ext.minHi, at.Hi)
		ext.maxLo, ext.maxHi = survey2d.RatMax(ext.maxLo, at.Lo), survey2d.RatMax(ext.maxHi, at.Hi)
	}

	sure, maybe := survey2d.WindowReachesDirection(coss, sins, a, b)
	if maybe {
		ext.maxHi = survey2d.RatMax(ext.maxHi, peak.Hi)
	}
	if sure {
		ext.maxLo = survey2d.RatMax(ext.maxLo, peak.Lo)
	}
	sure, maybe = survey2d.WindowReachesDirection(coss, sins, new(big.Rat).Neg(a), new(big.Rat).Neg(b))
	if maybe {
		ext.minLo = survey2d.RatMin(ext.minLo, trough.Lo)
	}
	if sure {
		ext.minHi = survey2d.RatMin(ext.minHi, trough.Hi)
	}
	return ext, true
}

func newPlacedFrameMap(pp prismPayload) (survey2d.PlacedFrameMap, bool) {
	basis := pp.xform.Basis()
	ex, okX := survey2d.IvVec3Of(basis.EX)
	ey, okY := survey2d.IvVec3Of(basis.EY)
	ez, okZ := survey2d.IvVec3Of(basis.EZ)
	translation, okT := survey2d.IvVec3Of(pp.xform.Translation())
	origin, okO := survey2d.IvVec3Of(pp.frame.Origin())
	u, okU := survey2d.IvVec3Of(pp.frame.U())
	v, okV := survey2d.IvVec3Of(pp.frame.V())
	n, okN := survey2d.IvVec3Of(pp.frame.N())
	if !okX || !okY || !okZ || !okT || !okO || !okU || !okV || !okN {
		return survey2d.PlacedFrameMap{}, false
	}
	place := func(local survey2d.IvVec3) survey2d.IvVec3 {
		return survey2d.IvVec3Add(
			survey2d.IvVec3Mul(ex, local[0]),
			survey2d.IvVec3Add(survey2d.IvVec3Mul(ey, local[1]), survey2d.IvVec3Mul(ez, local[2])),
		)
	}
	return survey2d.PlacedFrameMap{
		Origin: survey2d.IvVec3Add(place(origin), translation),
		Du:     place(u),
		Dv:     place(v),
		Dn:     place(n),
	}, true
}

// intervalMid is one rational strictly inside an enclosure, the point a bound
// measured from either end is smallest against.
func intervalMid(a proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Add(a.Lo, a.Hi), big.NewRat(1, 2))
}
