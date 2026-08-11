package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns ONE bound: how far the surface a cap-loop chamfer's band patch
// actually carries can point away from the surface the build tags it with
// (docs/modify-reach-design.md §8.3) — the `Cone` of a circular patch and the
// `Plane` of a flat one alike. `Face.NormalAt` publishes it beside its own
// arithmetic proof (normal_bound.go) and DX7 composes it into the allowance its
// decision reads (capblend_survey.go).
//
// The surface the build assembles is not the tag, and it is not the tag for two
// independent reasons that no bound may separate:
//
//   - The GEOMETRY, which is the circular patch's alone. The build rules the
//     patch with straight `Line3` rulings between a side-level directrix and a
//     cap-level one, and a non-tangential miter corner trims the cap directrix
//     to a narrower angular window than the side one sweeps. A straight-ruled
//     surface between two arcs sweeping different windows is not a cone at all.
//   - The ARITHMETIC OF PLACEMENT. Every world coordinate the build emits — each
//     directrix centre, each ruling endpoint, the tag's own origin and its half
//     angle — is a ROUNDED image of the plane-local number it denotes, and the
//     roundings are independent of one another. Two of them that would coincide
//     in exact arithmetic no longer do, so a band whose windows genuinely
//     coincide — a tangent join, a reflex corner's apex patch, a whole turn —
//     still carries a built surface that leaves its tag. The gap grows with the
//     distance from the world origin to the patch and shrinks with the patch's
//     own size, so a small band placed far away shows it at a scale orders past
//     any reading's own arithmetic bound. A FLAT patch is exempt from the first
//     reason and not from this one: its four corners round independently too,
//     and the `Plane` the build tags it with is fixed from three of them.
//
// A bound derived from plane-local angles alone is blind to the second, and a
// zero it publishes on a placed band is an ASSERTION rather than a measurement.
// So this file states the whole thing once, in WORLD space, from the held
// numbers the body actually publishes — the directrices' own `Arc3` centres,
// axes and radii or their own straight endpoints, the rulings' own endpoints,
// and the tag's own frame, origin, axis, radius and half angle — and in exact
// rational interval arithmetic, so a zero width here records that two
// computations agree EXACTLY. Nothing is sampled and no small residual admits
// anything: the enclosure IS the exact answer.
//
// THE CIRCULAR DERIVATION. Write â for the tag's unit axis, O for its origin, h
// for its held half angle and R for its held radius at O, and take any point P
// of the patch. The tag's own unit normal there is
//
//	n = σ·(r̂_P·cos h - â·sin h),  r̂_P the unit radial direction from the axis,
//
// the same exact answer `Face.NormalAt`'s Cone arm computes and normal_bound.go
// judges. The built surface at P is spanned by its own two tangents — the
// straight ruling r and the directrix tangent t — so writing p for the part of n
// lying in that span and decomposing p in the (generally oblique) basis (r̂, t̂),
//
//	sin∠(n, n_built) = |p| <= (|n·r̂| + |n·t̂|) / sin∠(r, t).
//
// Each of the three is bounded uniformly over the whole patch:
//
//   - |n·r̂|. Every point X of either directrix has an EXACT axial coordinate
//     z_X = (X-O)·â and a radial one ρ_X = |(X-O)_⊥|, and κ_X = cos h·(ρ_X - R) -
//     sin h·z_X is exactly its signed distance from the tag cone — zero where the
//     directrix lies on the tag. With Δφ the azimuth difference between the
//     ruling's two ends, n·(X-O) = κ_X - cos h·ρ_X·(1 - cos Δφ), and r is the
//     difference of two such terms.
//   - |n·t̂|. A directrix is a circle about â, so its tangent is â × (X - C) for
//     its own held centre C; â·(â × w) is exactly zero, and what is left is
//     cos h·(ρ·|sin Δφ| + |(C-O)_⊥|) — the azimuth spread and the centre's own
//     distance off the tag's axis, nothing else. t is a NON-NEGATIVE combination
//     of the two directrices' tangents, so the ratio |n·t|/|t| never exceeds the
//     larger of the two per-directrix ratios, divided by the cosine of half the
//     angle between them.
//   - sin∠(r, t). Both tangents' t is exactly perpendicular to â, so projecting t
//     out of r leaves r's whole axial component untouched: sin∠(r, t) >=
//     |z_cap - z_side| / |r|.
//
// The azimuth spread Δφ is the one quantity the corners fix. Both directrices are
// swept azimuth-affinely between the azimuths of the held ruling endpoints, so
// Δφ is affine in the patch parameter and is bounded by its values at the two
// ends — measured about the TAG's own axis, from the held vertices, which is
// exactly where the placement's rounding enters.
//
// A held boundary vertex is not obliged to sit exactly on the circle its own
// edge publishes, so a reader could reconstruct the patch's extreme rulings
// either between the circle points at those azimuths or between the vertices
// themselves. Both are covered: each directrix's radial and axial enclosures
// hull its own ruling endpoints in beside its circle, and every step above holds
// for any point inside those enclosures rather than for the circle alone. A whole turn carries one such
// end (its two seam vertices) and sweeps both directrices a full period, so its
// Δφ is constant. A reflex corner's apex patch has a POINT side directrix; its
// azimuth spread is zero as a fact of the construction, but only once the apex
// vertex and the tag's origin are proven to be the same held point, which this
// file checks rather than assumes.
//
// THE FLAT DERIVATION. A straight wall's patch is bounded by four straight
// edges — the wall's own side-level segment, the two slant rulings, and the
// cap-level segment the offset denotes — so the surface the build assembles
// between them is the bilinear patch those four held corners rule:
//
//	S(u, v) = (1-v)·((1-u)·A₀ + u·A₁) + v·((1-u)·B₀ + u·B₁),
//
// A the side-level pair and B the cap-level one, paired end for end by the
// rulings themselves. The `Plane` the build tags it with is fixed from THREE of
// those corners (capblend_geom.go's planeFromThree), so the fourth is free to
// leave it — and on a placed body it does. A straight wall's own offset is
// parallel to it in exact arithmetic and not in float, and that freedom is
// exactly what this bound measures rather than asserts away.
//
// The built normal is N = S_u × S_v, and both factors are affine in the SAME
// difference:
//
//	S_u = P₀ + v·ΔP,  S_v = R₀ + u·ΔP,
//	P₀ = A₁-A₀,  ΔP = (B₁-B₀) - P₀,  R₀ = B₀-A₀,
//
// so the product's own uv term is ΔP × ΔP and vanishes exactly, leaving
//
//	N(u, v) = P₀×R₀ + u·(P₀×ΔP) + v·(ΔP×R₀).
//
// Crossing that with the tag's own held normal n is linear, so n × N is again
// affine in (u, v) and its three coefficients are exact rationals in which the
// cancellation has ALREADY happened — nothing is enclosed before it does, which
// is what keeps the bound at the scale of the departure rather than at the
// scale of the patch. Over u, v ∈ [0, 1] each coefficient contributes at most
// its own magnitude, so with C₀ = P₀×R₀, C_u = P₀×ΔP and C_v = ΔP×R₀,
//
//	sin∠(n, N) = |n×N| / (|n|·|N|)
//	          <= (|n×C₀| + |n×C_u| + |n×C_v|) / (|n| · (|C₀| - |C_u| - |C_v|)),
//
// refused wherever the denominator is not proven positive. The four corner
// values N(0,0), N(1,0), N(0,1), N(1,1) are the normals of the four triangles
// the quad's two diagonals split it into, so this one bound covers every
// triangulation a reader takes of the patch as well as the ruled surface
// itself.
//
// The angle itself is recovered from its sine, which needs the two normals to be
// closer than a right angle. That is exactly the condition the bound is refused
// under: where the enclosure cannot put the sine below one, the answer is the
// trivial `ruledNormalAllowUnbounded`, never a clamped number that would read as
// a proof. Below a right angle the two surfaces bound the same material on the
// same side, so the outward sense the face publishes and the built patch's own
// pick the same one of the two candidate directions, and the unoriented angle IS
// the distance the readings owe.

// ruledNormalAllowUnbounded is the trivial bound on the distance between two
// unit vectors: any two of them differ by at most 2. It is what
// capPatchNormalAllow returns wherever its own derivation degenerates — an
// honest "this reading proves nothing" rather than a smaller number nothing
// supports. Every consumer treats it as undecided rather than as a width.
const ruledNormalAllowUnbounded = 2.0

// capDirectrixRef is one built directrix as the body PUBLISHES it: the centre,
// axis and radius of its own `Arc3`/`Circle3` curve, and the ruling endpoints
// its own boundary vertices sit at. A reflex corner's apex patch has a
// degenerate side directrix — radius zero, every ruling endpoint the same apex
// vertex — which capPatchDeparture admits only against the tag's own origin.
//
// straight records a directrix the body publishes as a `Line3` instead: a
// straight wall patch's own side and cap edges. It carries no circle at all, so
// only its two ruling endpoints are read and the circular derivation refuses it
// outright rather than reading its unset centre and radius as a point
// directrix.
type capDirectrixRef struct {
	center   r3.Vec
	axis     r3.Vec
	radius   float64
	ends     []r3.Vec
	straight bool
}

// capPatchBuilt is one band patch's built ruled surface, named by the two
// directrices it runs between. ends are paired by index: ruling k joins
// sideDir.ends[k] to capDir.ends[k].
type capPatchBuilt struct {
	sideDir capDirectrixRef
	capDir  capDirectrixRef
}

// capPatchNormalAllow is the file's answer for one patch: the proven bound on
// how far the RULED surface the build assembles can carry a normal differing
// from the surface the patch publishes.
//
// Both patch kinds answer from their own held corners, and neither is exempt.
// A straight wall's offset family is affine in the offset amount, so the exact
// surface a flat patch denotes IS its tag's plane — but the corners the build
// emits are each rounded once more, and the tag is fixed through three of the
// four, so the built quad still leaves the plane by an amount only a world-space
// reading of those four corners states.
func capPatchNormalAllow(f *Face, g capPatchGeom, b capPatchBuilt) float64 {
	departure := capPlaneDeparture
	if g.circular {
		departure = capPatchDeparture
	}
	allow, ok := departure(f, b)
	if !ok || isNonFinite(allow) || allow < 0 {
		return ruledNormalAllowUnbounded
	}
	return math.Min(ruledNormalAllowUnbounded, allow)
}

// capPlaneDeparture is the flat half of this file's derivation, evaluated over
// exact rational arithmetic on the patch's own four held corners and the tag's
// own held frame normal. It reports ok false wherever an enclosure fails to
// prove what a step needs — a directrix the body does not publish as a straight
// pair of ends, a built normal not proven away from zero, a tag frame normal not
// proven away from zero, or a sine not proven below one — and the caller
// publishes the trivial bound there.
func capPlaneDeparture(f *Face, b capPatchBuilt) (float64, bool) {
	tag, okTag := f.surface.(Plane)
	if !okTag {
		return 0, false
	}
	n, okN := ivVec3Of(tag.Frame.N())
	a0, a1, okS := capStraightEnds(b.sideDir)
	b0, b1, okC := capStraightEnds(b.capDir)
	if !okN || !okS || !okC {
		return 0, false
	}

	// The bilinear patch's own normal N(u, v) = C0 + u·Cu + v·Cv, exactly.
	p0 := ivVec3Sub(a1, a0)
	dp := ivVec3Sub(ivVec3Sub(b1, b0), p0)
	r0 := ivVec3Sub(b0, a0)
	c0, cu, cv := ivVec3Cross(p0, r0), ivVec3Cross(p0, dp), ivVec3Cross(dp, r0)

	// |N| from below: the constant term's own length, less what the two affine
	// terms can take from it anywhere in the unit square.
	baseLen, okB := intervalSqrt(ivVec3NormSq(c0))
	uLen, okU := intervalSqrt(ivVec3NormSq(cu))
	vLen, okV := intervalSqrt(ivVec3NormSq(cv))
	tagLen, okL := intervalSqrt(ivVec3NormSq(n))
	if !okB || !okU || !okV || !okL {
		return 0, false
	}
	builtLo := new(big.Rat).Sub(baseLen.lo, ratAdd(uLen.hi, vLen.hi))
	if builtLo.Sign() <= 0 || tagLen.lo.Sign() <= 0 {
		return 0, false
	}

	// |n × N| from above, coefficient by coefficient: the cross with the held
	// normal is taken on each exact coefficient BEFORE anything is bounded, so
	// the cancellation the near-planar quad carries survives into the answer.
	crossSq := new(big.Rat)
	k0, ku, kv := ivVec3Cross(n, c0), ivVec3Cross(n, cu), ivVec3Cross(n, cv)
	for i := range 3 {
		comp := ratAdd(intervalAbsUpper(k0[i]), intervalAbsUpper(ku[i]), intervalAbsUpper(kv[i]))
		crossSq.Add(crossSq, new(big.Rat).Mul(comp, comp))
	}
	crossLen, okX := intervalSqrt(pointInterval(crossSq))
	if !okX {
		return 0, false
	}

	sine := new(big.Rat).Quo(crossLen.hi, new(big.Rat).Mul(tagLen.lo, builtLo))
	if sine.Cmp(big.NewRat(1, 1)) >= 0 {
		return 0, false
	}
	held := ratFloatUp(sine)
	if isNonFinite(held) || held >= 1 {
		return 0, false
	}
	// A chord never exceeds its own arc, so the angle bounds the distance
	// between the two unit directions as well as their separation.
	return upRound(math.Asin(held)), true
}

// capStraightEnds encloses one straight directrix's two held ruling endpoints,
// which is everything the flat derivation reads off it. A directrix the body
// publishes as anything but a `Line3` carrying exactly two ends is refused.
func capStraightEnds(ref capDirectrixRef) (ivVec3, ivVec3, bool) {
	if !ref.straight || len(ref.ends) != 2 {
		return ivVec3{}, ivVec3{}, false
	}
	first, okF := ivVec3Of(ref.ends[0])
	second, okS := ivVec3Of(ref.ends[1])
	if !okF || !okS {
		return ivVec3{}, ivVec3{}, false
	}
	return first, second, true
}

// capPatchDeparture is the derivation stated in this file's own comment,
// evaluated over exact rational intervals. It reports ok false wherever an
// enclosure fails to prove what a step needs — a directrix axis not proven
// parallel to the tag's, an azimuth pair not proven inside a half turn, a
// vanishing tangent arm or sweep height, or a sine not proven below one — and
// the caller publishes the trivial bound there.
func capPatchDeparture(f *Face, b capPatchBuilt) (float64, bool) {
	sinH, cosH, originVec, axisVec, ok := coneTagTerms(f)
	if !ok {
		return 0, false
	}
	tagRadius, okR := coneTagRadius(f)
	axisIv, okA := ivVec3Of(axisVec)
	originIv, okO := ivVec3Of(originVec)
	if !okR || !okA || !okO {
		return 0, false
	}
	ahat, st := ivVec3Unit(axisIv)
	if st != normalProven {
		return 0, false
	}

	side, okS := capDirectrixEnclose(b.sideDir, originVec, axisVec, ahat, originIv)
	capped, okC := capDirectrixEnclose(b.capDir, originVec, axisVec, ahat, originIv)
	if !okS || !okC {
		return 0, false
	}
	sigma, okSig := capRulingSkew(b, side, capped, ahat, originIv)
	if !okSig {
		return 0, false
	}

	half := big.NewRat(1, 2)
	sigmaSq := new(big.Rat).Mul(sigma, sigma)
	cosMax := intervalAbsUpper(cosH)
	kappa := func(d capDirectrix) *big.Rat {
		return intervalAbsUpper(intervalSub(
			intervalMul(cosH, intervalSub(d.rho, pointInterval(tagRadius))),
			intervalMul(sinH, d.z),
		))
	}
	// |n·r|: each end's own distance from the tag cone, plus the azimuth spread's
	// second-order term (1 - cos σ <= σ²/2, exact for every real σ).
	ruling := ratAdd(
		kappa(side),
		kappa(capped),
		ratMul(cosMax, ratAdd(side.rho.hi, capped.rho.hi), sigmaSq, half),
	)

	// |n·t̂|: the larger per-directrix ratio, widened by the cosine of half the
	// angle between the two directrices' own tangents (sin σ <= σ).
	tangent := new(big.Rat)
	spread := new(big.Rat).Set(sigma)
	arms := 0
	for _, d := range [...]capDirectrix{side, capped} {
		if d.point {
			continue
		}
		if d.armLo.Sign() <= 0 {
			return 0, false
		}
		arms++
		tangent = ratMax(tangent, new(big.Rat).Quo(
			ratMul(cosMax, ratAdd(ratMul(d.rho.hi, sigma), d.offset)),
			d.armLo,
		))
		spread.Add(spread, new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), d.offset), d.armLo))
	}
	if arms == 0 {
		return 0, false
	}
	if arms == 2 {
		// cos(x) >= 1 - x²/2, which is a usable divisor only while the two
		// tangents stay well inside a half turn of each other.
		if spread.Cmp(big.NewRat(1, 1)) >= 0 {
			return 0, false
		}
		shrink := new(big.Rat).Sub(big.NewRat(1, 1), ratMul(spread, spread, big.NewRat(1, 8)))
		tangent = new(big.Rat).Quo(tangent, shrink)
	}

	// |r| from above and |z_cap - z_side| from below: |r|² = Δz² + |Δq|² and
	// |Δq|² = (ρ_cap - ρ_side)² + 2·ρ_side·ρ_cap·(1 - cos Δφ).
	dz := intervalSub(capped.z, side.z)
	var dzLo *big.Rat
	switch {
	case dz.lo.Sign() > 0:
		dzLo = dz.lo
	case dz.hi.Sign() < 0:
		dzLo = new(big.Rat).Neg(dz.hi)
	default:
		return 0, false
	}
	rulingLen, okLen := intervalSqrt(pointInterval(ratAdd(
		intervalSquare(dz).hi,
		intervalSquare(intervalSub(capped.rho, side.rho)).hi,
		ratMul(side.rho.hi, capped.rho.hi, sigmaSq),
	)))
	if !okLen {
		return 0, false
	}
	sine := new(big.Rat).Quo(ratAdd(ruling, ratMul(tangent, rulingLen.hi)), dzLo)
	if sine.Cmp(big.NewRat(1, 1)) >= 0 {
		return 0, false
	}
	held := ratFloatUp(sine)
	if isNonFinite(held) || held >= 1 {
		return 0, false
	}
	// A chord never exceeds its own arc, so the angle bounds the distance
	// between the two unit directions as well as their separation.
	return upRound(math.Asin(held)), true
}

// capDirectrix is one built directrix reduced to the four readings the
// derivation needs, all measured in the TAG's own cylindrical frame: the radial
// and axial coordinates the whole curve and its ruling endpoints span, the
// smallest tangent arm |(X - C)_⊥| any of them gives, and the centre's own
// distance off the tag's axis.
type capDirectrix struct {
	rho    ratInterval
	z      ratInterval
	armLo  *big.Rat
	offset *big.Rat
	// point records a directrix that is a single point ON the tag's own axis,
	// which is the one configuration with no tangent arm and no azimuth of its
	// own: a reflex corner's apex patch.
	point bool
}

// capDirectrixEnclose measures one published directrix against the tag. The
// circle's own points are enclosed from its held centre and radius — every one
// of them shares the centre's axial coordinate exactly, and its radial one lies
// within the centre's off-axis distance of the held radius — and the ruling
// endpoints are enclosed one by one and hulled in, since a held vertex is not
// obliged to sit exactly on the curve its own edge publishes.
//
// It refuses a directrix whose own axis is not proven exactly parallel to the
// tag's: the enclosure of a circle's axial coordinate as a constant is that
// parallelism, and nothing else here would notice its loss.
func capDirectrixEnclose(ref capDirectrixRef, tagOrigin, tagAxis r3.Vec, ahat, origin ivVec3) (capDirectrix, bool) {
	radius := floatRat(ref.radius)
	if ref.straight || radius == nil || radius.Sign() < 0 {
		return capDirectrix{}, false
	}
	centerIv, okC := ivVec3Of(ref.center)
	if !okC || len(ref.ends) == 0 {
		return capDirectrix{}, false
	}
	if radius.Sign() == 0 {
		// A point directrix carries no circle and no azimuth. It is admitted
		// only where it IS the tag's own origin, held for held: that is what
		// makes its azimuth spread against the other directrix an exact zero
		// rather than an arbitrary direction read off a vanishing vector.
		if ref.center != tagOrigin {
			return capDirectrix{}, false
		}
		for _, v := range ref.ends {
			if v != ref.center {
				return capDirectrix{}, false
			}
		}
		zero := new(big.Rat)
		return capDirectrix{rho: pointInterval(zero), z: pointInterval(zero), armLo: zero, offset: zero, point: true}, true
	}
	if !capAxesParallel(ref.axis, tagAxis) {
		return capDirectrix{}, false
	}

	rel := ivVec3Sub(centerIv, origin)
	zc := ivVec3Dot(rel, ahat)
	offset, okOff := intervalSqrt(ivVec3NormSq(ivVec3Sub(rel, ivVec3Mul(ahat, zc))))
	if !okOff {
		return capDirectrix{}, false
	}
	out := capDirectrix{
		rho:    interval(new(big.Rat).Sub(radius, offset.hi), new(big.Rat).Add(radius, offset.hi)),
		z:      zc,
		armLo:  new(big.Rat).Set(radius),
		offset: offset.hi,
	}
	for _, v := range ref.ends {
		vIv, ok := ivVec3Of(v)
		if !ok {
			return capDirectrix{}, false
		}
		relV := ivVec3Sub(vIv, origin)
		zv := ivVec3Dot(relV, ahat)
		rho, okRho := intervalSqrt(ivVec3NormSq(ivVec3Sub(relV, ivVec3Mul(ahat, zv))))
		armRel := ivVec3Sub(vIv, centerIv)
		arm, okArm := intervalSqrt(ivVec3NormSq(ivVec3Sub(armRel, ivVec3Mul(ahat, ivVec3Dot(armRel, ahat)))))
		if !okRho || !okArm {
			return capDirectrix{}, false
		}
		out.rho = intervalHull(out.rho, rho)
		out.z = intervalHull(out.z, zv)
		out.armLo = ratMin(out.armLo, arm.lo)
	}
	return out, true
}

// capRulingSkew is the widest azimuth difference the patch's rulings span,
// measured about the TAG's own axis from the held ruling endpoints. Both
// directrices are swept azimuth-affinely between those endpoints, so the
// difference is affine in the patch's own parameter and its ends bound it
// everywhere between.
//
// The angle is read as an arctangent of the two endpoints' cross and dot
// products about the axis and charged its own tangent, which bounds it from
// above (atan x <= x). A pair whose dot product is not proven positive is
// refused rather than unwrapped onto a branch nothing chose.
//
// A patch with a point directrix has no such pair at all: its rulings all leave
// the tag's own origin, where every ruling's own azimuth is the other end's, so
// the spread is an exact zero.
func capRulingSkew(b capPatchBuilt, side, capped capDirectrix, ahat, origin ivVec3) (*big.Rat, bool) {
	if side.point || capped.point {
		return new(big.Rat), true
	}
	if len(b.sideDir.ends) != len(b.capDir.ends) {
		return nil, false
	}
	skew := new(big.Rat)
	for i := range b.sideDir.ends {
		qs, oks := capAxisPerp(b.sideDir.ends[i], ahat, origin)
		qc, okc := capAxisPerp(b.capDir.ends[i], ahat, origin)
		if !oks || !okc {
			return nil, false
		}
		dot := ivVec3Dot(qs, qc)
		if dot.lo.Sign() <= 0 {
			return nil, false
		}
		cross := ivVec3Dot(ahat, ivVec3Cross(qs, qc))
		skew = ratMax(skew, new(big.Rat).Quo(intervalAbsUpper(cross), dot.lo))
	}
	return skew, true
}

// capAxisPerp is a held point's own axis-perpendicular offset from the tag's
// origin, exactly enclosed.
func capAxisPerp(p r3.Vec, ahat, origin ivVec3) (ivVec3, bool) {
	pIv, ok := ivVec3Of(p)
	if !ok {
		return ivVec3{}, false
	}
	rel := ivVec3Sub(pIv, origin)
	return ivVec3Sub(rel, ivVec3Mul(ahat, ivVec3Dot(rel, ahat))), true
}

// capAxesParallel decides, in exact arithmetic on the two held vectors alone,
// whether a directrix's own axis is parallel to the tag's. Both are the SAME
// plane normal placed once — the band's arcs and its cone tag read
// prismPayload.dir(0, 0, ±1), whose sign flip is an exact negation — so the
// cross product is exactly zero wherever the build is the one this file
// describes, and anything else refuses.
func capAxesParallel(a, b r3.Vec) bool {
	av, oka := ivVec3Of(a)
	bv, okb := ivVec3Of(b)
	if !oka || !okb {
		return false
	}
	cross := ivVec3Cross(av, bv)
	for _, c := range cross {
		if c.lo.Sign() != 0 || c.hi.Sign() != 0 {
			return false
		}
	}
	return true
}

func ivVec3Cross(a, b ivVec3) ivVec3 {
	return ivVec3{
		intervalSub(intervalMul(a[1], b[2]), intervalMul(a[2], b[1])),
		intervalSub(intervalMul(a[2], b[0]), intervalMul(a[0], b[2])),
		intervalSub(intervalMul(a[0], b[1]), intervalMul(a[1], b[0])),
	}
}

// coneTagRadius is the radius a circular patch's own tag carries at its origin
// — zero for every tag the cap band builds, since coneSurface anchors a Cone at
// its apex. It is read rather than assumed, so the signed distance κ this file
// measures stays the tag's own implicit function whatever radius a tag states.
func coneTagRadius(f *Face) (*big.Rat, bool) {
	var value units.Value
	switch s := f.surface.(type) {
	case Cone:
		value = s.Radius
	case Cylinder:
		value = s.Radius
	default:
		return nil, false
	}
	mm, err := value.In(units.Millimeter)
	if err != nil {
		return nil, false
	}
	r := floatRat(mm)
	if r == nil {
		return nil, false
	}
	return r, true
}

// capBuiltPatch names one band patch's built ruled surface from the two
// published directrix edges and the boundary vertices that pair them: ruling k
// joins sideEnds[k] to capEnds[k]. A directrix this evaluator cannot read
// leaves its own reference empty, which either departure derivation refuses and
// the caller publishes the trivial bound for, rather than a number no reading
// supports.
func capBuiltPatch(sideEdge, capEdge *Edge, sideEnds, capEnds []*Vertex) capPatchBuilt {
	return capPatchBuilt{
		sideDir: capDirectrixFromEdge(sideEdge, sideEnds...),
		capDir:  capDirectrixFromEdge(capEdge, capEnds...),
	}
}

// capBuiltApexPatch is the same for a reflex corner's apex patch, whose side
// directrix is the single original corner VERTEX the rulings all leave: a
// circle of radius zero, which capPatchDeparture admits only where that vertex
// is the tag's own origin held for held.
func capBuiltApexPatch(apex *Vertex, capEdge *Edge, capEnds []*Vertex) capPatchBuilt {
	capped := capDirectrixFromEdge(capEdge, capEnds...)
	if apex == nil {
		return capPatchBuilt{capDir: capped}
	}
	ends := make([]r3.Vec, len(capped.ends))
	for i := range ends {
		ends[i] = apex.position
	}
	return capPatchBuilt{
		sideDir: capDirectrixRef{center: apex.position, ends: ends},
		capDir:  capped,
	}
}

// capDirectrixFromEdge reads one published directrix edge. Only the three curve
// kinds a band's directrix is ever built as are admitted — the two circular ones
// a wall over an arc gives, and the `Line3` a straight wall gives, which carries
// no circle and is read through its ruling endpoints alone; anything else leaves
// the patch without a stated built surface and the caller publishes the trivial
// bound.
func capDirectrixFromEdge(e *Edge, ends ...*Vertex) capDirectrixRef {
	var out capDirectrixRef
	var radius units.Value
	switch c := e.curve.(type) {
	case Arc3:
		out.center, out.axis, radius = c.Center, c.Axis, c.Radius
	case Circle3:
		out.center, out.axis, radius = c.Center, c.Axis, c.Radius
	case Line3:
		out.straight = true
	default:
		return capDirectrixRef{}
	}
	if !out.straight {
		mm, err := radius.In(units.Millimeter)
		if err != nil {
			return capDirectrixRef{}
		}
		out.radius = mm
	}
	for _, v := range ends {
		if v == nil {
			return capDirectrixRef{}
		}
		out.ends = append(out.ends, v.position)
	}
	return out
}
