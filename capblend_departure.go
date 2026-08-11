package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns ONE bound: how far the surface a cap-loop chamfer's circular
// band patch actually carries can point away from the `Cone` the build tags it
// with (docs/modify-reach-design.md §8.3). `Face.NormalAt` publishes it beside
// its own arithmetic proof (normal_bound.go) and DX7 composes it into the
// allowance its decision reads (capblend_survey.go).
//
// The surface the build assembles is not the tag, and it is not the tag for two
// independent reasons that no bound may separate:
//
//   - The GEOMETRY. The build rules the patch with straight `Line3` rulings
//     between a side-level directrix and a cap-level one, and a non-tangential
//     miter corner trims the cap directrix to a narrower angular window than the
//     side one sweeps. A straight-ruled surface between two arcs sweeping
//     different windows is not a cone at all.
//   - The ARITHMETIC OF PLACEMENT. Every world coordinate the build emits — each
//     directrix centre, each ruling endpoint, the tag's own origin and its half
//     angle — is a ROUNDED image of the plane-local number it denotes, and the
//     roundings are independent of one another. Two of them that would coincide
//     in exact arithmetic no longer do, so a band whose windows genuinely
//     coincide — a tangent join, a reflex corner's apex patch, a whole turn —
//     still carries a built surface that leaves its tag. The gap grows with the
//     distance from the world origin to the patch and shrinks with the patch's
//     own size, so a small band placed far away shows it at a scale orders past
//     any reading's own arithmetic bound.
//
// A bound derived from plane-local angles alone is blind to the second, and a
// zero it publishes on a placed band is an ASSERTION rather than a measurement.
// So this file states the whole thing once, in WORLD space, from the held
// numbers the body actually publishes — the two directrices' own `Arc3` centres,
// axes and radii, the rulings' own endpoints, and the tag's own origin, axis,
// radius and half angle — and in exact rational interval arithmetic, so a zero
// width here records that two computations agree EXACTLY. Nothing is sampled and
// no small residual admits anything: the enclosure IS the exact answer.
//
// THE DERIVATION. Write â for the tag's unit axis, O for its origin, h for its
// held half angle and R for its held radius at O, and take any point P of the
// patch. The tag's own unit normal there is
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
type capDirectrixRef struct {
	center r3.Vec
	axis   r3.Vec
	radius float64
	ends   []r3.Vec
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
// A `Plane` patch is exempt and answers an exact zero: a straight wall's own
// offset family is affine in the offset amount, so ruling the patch between the
// wall's side-level segment and its cap-level one reproduces the denoted offset
// at every level, and the build tags the patch with the plane through its own
// built corners.
func capPatchNormalAllow(f *Face, g capPatchGeom, b capPatchBuilt) float64 {
	if !g.circular {
		return 0
	}
	allow, ok := capPatchDeparture(f, b)
	if !ok || isNonFinite(allow) || allow < 0 {
		return ruledNormalAllowUnbounded
	}
	return math.Min(ruledNormalAllowUnbounded, allow)
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
	if radius == nil || radius.Sign() < 0 {
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
// leaves its own reference empty, which capPatchDeparture refuses and the
// caller publishes the trivial bound for, rather than a number no reading
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

// capDirectrixFromEdge reads one published directrix edge's own circle. Only
// the two circular curve kinds a band's directrix is ever built as are
// admitted; anything else leaves the patch without a stated built surface and
// the caller publishes the trivial bound.
func capDirectrixFromEdge(e *Edge, ends ...*Vertex) capDirectrixRef {
	var out capDirectrixRef
	var radius units.Value
	switch c := e.curve.(type) {
	case Arc3:
		out.center, out.axis, radius = c.Center, c.Axis, c.Radius
	case Circle3:
		out.center, out.axis, radius = c.Center, c.Axis, c.Radius
	default:
		return capDirectrixRef{}
	}
	mm, err := radius.In(units.Millimeter)
	if err != nil {
		return capDirectrixRef{}
	}
	out.radius = mm
	for _, v := range ends {
		if v == nil {
			return capDirectrixRef{}
		}
		out.ends = append(out.ends, v.position)
	}
	return out
}
