package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is evalCapBlendContext — the build orchestration — plus the
// bounded mass-property integrals of docs/modify-reach-design.md §8.4: area,
// signed volume and centroid, closed form per patch, with a proven bound and
// Exact reserved for a proven-zero one. It never uses quadrature to claim
// Exact.
//
// Volume is computed by the divergence theorem, per loop, per cap, entirely
// in the payload's own PLANE-LOCAL (u, v, z) coordinates (a rigid transform
// preserves volume, so this is exact to reproduce in world space): the whole
// body's volume is the sum, over every loop (holes negative, exactly as the
// straight-prism reduction already sums signed loop areas), of that loop's
// own straight-slab contribution (area times its own straight height) plus
// its chamfer band contribution(s). Each band is closed off with two flat
// artificial disks — the offset loop's own enclosed region at the cap level,
// the original loop's own enclosed region at the side level — so the whole
// band is a genuinely closed sub-solid and its volume is the
// divergence-theorem flux sum over its patches and the two disks, taken
// relative to the plane-local origin (valid because the WHOLE band, patches
// plus its two disks, is a closed surface, and a closed surface's flux
// integral is reference-point independent). A flat Plane patch's flux is the
// tetrahedron identity, a polynomial in the payload's own floats, taken
// EXACTLY over big.Rat and rounded once at the end; a Cone patch's is a
// closed-form polynomial-plus-trig expression evaluated in floats and never
// claimed Exact.
//
// The volume bound is composed term by term, and the reason is the band's own
// shape. Every one of these flux terms is a DIFFERENCE that cancels: the two
// closing disks each carry the whole prism's flux and differ by the band's,
// smaller by the ratio of the sweep height to the setback; a ruled-cone
// integral cancels likewise. A rounding budget scaled by the sum they
// cancelled to under-counts by exactly that ratio, so no bound here is ever
// read off a summed result — each is charged against the absolute terms the
// step acted on, and boundedAdd/boundedMul then sum bounds while the values
// cancel. The Plane arm escapes that composition entirely rather than manage
// it: an exact rational has nothing to cancel, and its committed rounding is
// measured (rationalFloatError), not budgeted. The one term that passes
// through math.Sincos carries the magnitude envelope moments.go's
// analyticRoundBound doc reserves for a libm result, never that helper's
// roundoff budget.

// evalCapBlendContext builds the analytic cap-blend body from the payload
// (BX3): the trimmed prism side walls (buildLoopSidesAs, unmodified) plus,
// for every chamfered loop, the chamfer band patches and the offset cap
// boundary; every other loop and every unchamfered cap keeps the ordinary
// prism construction. One shell, one lump, watertight by the same argument
// evalPrism's is (every edge bounds exactly two faces).
func evalCapBlendContext(ctx context.Context, d *Document, ref StepRef, cbp capBlendPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	work := newFreeformWork()
	loops := cbp.loops()
	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	var faces []*Face
	startLoopObjs := make([]*Loop, len(loops))
	endLoopObjs := make([]*Loop, len(loops))
	var startArea, endArea, sideArea, patchArea, slabVolume, bandVolume boundedScalar
	// Appended in build order — loop index, then the chamfered cap, then each
	// band's own patch index — which IS Table BX row BX3's deterministic patch
	// order, the order the DX7 survey then reports its faces in.
	var patchGeoms []capPatch
	collectPatchGeoms := func(band capBandResult) {
		for i, f := range band.patches {
			if len(f.origins) == 0 {
				continue
			}
			patchGeoms = append(patchGeoms, capPatch{role: f.origins[0].Role, geom: band.geom[i]})
		}
	}

	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		onStart, onEnd := cbp.startLoops[li], cbp.endLoops[li]
		sign := 1.0
		if li != 0 {
			sign = -1
		}

		// A chamfered end pulls its own straight level in by the setback, and
		// that float sum rounds. The rounding is an ulp of the SWEEP, but it
		// multiplies the whole section area below, so it reaches the volume at
		// the scale of the band itself and is charged here — the same term
		// capBandVolume charges for the identical level it reads as sideZ.
		zLo, zHi := exactScalar(cbp.z0), exactScalar(cbp.z1)
		if onStart {
			v := cbp.z0 + cbp.d
			zLo = measuredScalar(v, addRoundError(cbp.z0, cbp.d, v))
		}
		if onEnd {
			v := cbp.z1 - cbp.d
			zHi = measuredScalar(v, addRoundError(cbp.z1, -cbp.d, v))
		}
		ppFor := prismPayload{frame: cbp.frame, z0: zLo.value, z1: zHi.value, xform: cbp.xform}
		sideFaces, bottomCo, topCo, loopLen, err := buildLoopSidesAs(ctx, body, ref, ppFor, li, li != 0, loop, work)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		for _, f := range sideFaces {
			sideArea = boundedAdd(sideArea, measuredScalar(f.area, f.areaBound))
		}
		_ = loopLen

		loopArea, err := loopEnclosedAreaContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		straightHeight := boundedSub(zHi, zLo)
		loopSlab := boundedMul(loopArea, straightHeight)
		slabVolume = boundedAdd(slabVolume, measuredScalar(sign*loopSlab.value, loopSlab.bound))

		startCo, endCo := bottomCo, topCo

		if onStart {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z0, +1, bottomCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			startCo = band.capCo
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z0, +1)
			if err != nil {
				return nil, err
			}
			bandVolume = boundedAdd(bandVolume, measuredScalar(sign*v.value, v.bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = boundedAdd(patchArea, measuredScalar(pa, pb))
			}
		}
		if onEnd {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z1, -1, topCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			endCo = band.capCo
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z1, -1)
			if err != nil {
				return nil, err
			}
			bandVolume = boundedAdd(bandVolume, measuredScalar(sign*v.value, v.bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = boundedAdd(patchArea, measuredScalar(pa, pb))
			}
		}

		startLoopObjs[li] = &Loop{coedges: startCo, outer: li == 0}
		endLoopObjs[li] = &Loop{coedges: endCo, outer: li == 0}

		startBoundary := loop
		if onStart {
			startBoundary, err = capLoopBoundary(ctx, loop, cbp.d)
			if err != nil {
				return nil, err
			}
		}
		sArea, err := loopEnclosedAreaContext(ctx, startBoundary)
		if err != nil {
			return nil, err
		}
		startArea = boundedAdd(startArea, measuredScalar(sign*sArea.value, sArea.bound))

		endBoundary := loop
		if onEnd {
			endBoundary, err = capLoopBoundary(ctx, loop, cbp.d)
			if err != nil {
				return nil, err
			}
		}
		eArea, err := loopEnclosedAreaContext(ctx, endBoundary)
		if err != nil {
			return nil, err
		}
		endArea = boundedAdd(endArea, measuredScalar(sign*eArea.value, eArea.bound))
	}

	pl := prismPayload{frame: cbp.frame, xform: cbp.xform}
	startFrame, err := capFrame(pl, cbp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := capFrame(pl, cbp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := &Face{
		surface:   Plane{Frame: startFrame},
		origins:   []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:      body,
		loops:     startLoopObjs,
		area:      startArea.value,
		areaBound: startArea.bound,
	}
	capEnd := &Face{
		surface:   Plane{Frame: endFrame},
		origins:   []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:      body,
		loops:     endLoopObjs,
		area:      endArea.value,
		areaBound: endArea.bound,
	}
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	volume := boundedAdd(slabVolume, bandVolume)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.value),
		Exactness: exactnessOf(volume.bound),
		Bound:     units.CubicMillimeters(volume.bound),
	}

	totalArea := boundedAdd(boundedAdd(boundedAdd(startArea, endArea), sideArea), patchArea)
	body.area = Measurement{
		Value:     units.SquareMillimeters(totalArea.value),
		Exactness: exactnessOf(totalArea.bound),
		Bound:     units.SquareMillimeters(totalArea.bound),
	}

	// Centroid: an honest area-weighted estimate over the built faces'
	// representative points, backed by the geometry safety net every
	// prism/revolve/cup centroid already falls back to (the true centroid of
	// a bounded solid lies within its own bounding box, so |estimate-true| is
	// bounded by the box's own reach from the estimate regardless of how the
	// estimate was formed) — never a tighter claim than proven.
	bounds, err := capBlendBoundsContext(ctx, cbp, work)
	if err != nil {
		return nil, err
	}
	centroidValue, centroidBound := capBlendCentroidEstimate(faces, bounds)
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	cbp.patches = patchGeoms
	body.payload = cbp
	return body, nil
}

// capLoopBoundary returns loop_li's OWN offset-by-d boundary as a standalone
// LoopRecord, used to compute a chamfered cap's per-loop enclosed area and
// the band's closing disk at the cap level.
func capLoopBoundary(ctx context.Context, loop LoopRecord, d float64) (LoopRecord, error) {
	budget := newWorkBudget(ctx)
	work := newFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return LoopRecord{}, err
	}
	segs, err := offsetLoopBudget(budget, cl, 1, d)
	if err != nil {
		return LoopRecord{}, err
	}
	return LoopRecord{Segments: segs}, nil
}

// capBandVolume is one loop's chamfer-band volume contribution (positive,
// the material remaining in the band — the caller's "slab" term already
// covers the straight portion), by the divergence theorem over the band's
// own closed boundary: the two flat disks (the loop's own enclosed area at
// capZ and at sideZ) plus the patches (buildCapBand's geom).
//
// It returns the contribution WITH its own bound, and that is the whole of
// why the bound is sound. The two disk terms are each of magnitude
// |capZ|·|area| — the whole prism's flux — while their sum is the band's,
// smaller by a factor of the sweep height over the setback (H/d). A budget
// scaled by the SUM is scaled by a quantity these terms cancelled away, so
// every mechanism below is charged against the term it acts on, before the
// cancellation:
//
//   - each disk's own area bound (loopEnclosedAreaContext's, which for a
//     circular contour is a certified bracket and for a polygonal one is the
//     exact rational's rounding) multiplied by the level it sits at;
//   - the rounding of sideZ itself, which multiplies a whole disk area;
//   - each float multiplication and addition, whose committed error
//     boundedMul/boundedAdd take EXACTLY over big.Rat rather than estimate;
//   - each patch's own flux bound (patchRawFlux), including the Sincos term
//     that analyticRoundBound may never speak for.
//
// boundedAdd sums bounds, so no step of this composition is ever rescaled by
// a result the step's own operands cancelled down to.
func capBandVolume(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign float64) (boundedScalar, error) {
	sideZ := capZ + matSign*cbp.d
	// sideZ multiplies a whole disk area below, so its own rounding is a term
	// of the band and is charged here — an error the size of an ulp of the
	// sweep height, amplified by the section's area, which is of the order of
	// the band itself once the disks cancel.
	sideZBound := addRoundError(capZ, matSign*cbp.d, sideZ)
	// The two closing disks are loopEnclosedAreaContext's ABSOLUTE areas, so
	// the sub-solid they close off is the region the loop encloses read as
	// POSITIVELY oriented — counter-clockwise — whichever way the loop was
	// actually recorded. Every patch, in contrast, is built from the loop's
	// OWN walk, so a clockwise loop (a hole) hands the band patches facing
	// into it. orient rotates them back onto the disks' own orientation.
	signedArea, err := loopSignedAreaBudget(newWorkBudget(ctx), loop)
	if err != nil {
		return boundedScalar{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}
	sideArea, err := loopEnclosedAreaContext(ctx, loop)
	if err != nil {
		return boundedScalar{}, err
	}
	capBoundary, err := capLoopBoundary(ctx, loop, cbp.d)
	if err != nil {
		return boundedScalar{}, err
	}
	capArea, err := loopEnclosedAreaContext(ctx, capBoundary)
	if err != nil {
		return boundedScalar{}, err
	}
	// Outward normal signs (docs/modify-reach-design.md §8.4): the disk at
	// capZ faces -matSign*Z, the disk at sideZ faces +matSign*Z, both away
	// from the band's own material. A flat disk's raw flux (P.N over the
	// disk) is its constant Z coordinate times its signed normal times its
	// area — no triangulation needed.
	// The two sign factors are +1 or -1, so applying them is exact and each
	// level keeps whatever bound it arrived with: capZ is a payload coordinate
	// and exact, sideZ carries the rounding of its own sum.
	fluxTotal := boundedAdd(
		boundedMul(exactScalar(capZ*(-matSign)), capArea),
		boundedMul(measuredScalar(sideZ*matSign, sideZBound), sideArea),
	)
	// patchRawFlux's own v0..v3 (or triangle-fan) vertex order is FIXED —
	// side-level vertices first, cap-level second — regardless of which cap
	// the band sits on. That fixed order is "CCW as seen from outside" for
	// one Z ordering of (sideZ, capZ) and its mirror for the other, exactly
	// the same start/end asymmetry capblend_geom.go's fixPatchOrientation
	// corrects for the SURFACE normal — so the flux sign needs the same
	// -matSign correction here, confirmed empirically
	// (TestCapBlendStartCapVolumeMatchesEndCap). -matSign speaks for the
	// AXIAL half and orient for the IN-PLANE half; patchRawFlux itself has
	// already put each patch in its own walk's sense.
	for _, g := range geom {
		f := patchRawFlux(g)
		fluxTotal = boundedAdd(fluxTotal, measuredScalar(-matSign*orient*f.value, f.bound))
	}
	return boundedQuotient(fluxTotal.value, fluxTotal.bound, 3, 0), nil
}

// patchRawFlux is one patch's contribution to the band's total raw flux
// (3x its own volume-by-divergence-theorem share), taken relative to the
// plane-local origin (0, 0, 0), WITH its own proven bound. A flat Plane
// patch's contribution is the tetrahedron identity
// (Flux_triangle = 3*tetraVolume = (1/2)*v0.(v1 x v2)), evaluated over
// big.Rat; a Cone patch's is the closed-form polynomial-plus-trig integral
// over its linear-in-s ruled parametrization, evaluated in floats.
//
// The two arms differ in KIND of bound and that is §8.4's own rule ("report
// each exactly representable result as Exact") deciding it, not a preference.
// A Plane patch's flux is a polynomial in the payload's own float coordinates,
// so its exact value is a rational the arithmetic can carry and the ONLY
// rounding is the final one into a float64: exactPlanePatchFlux takes it, and
// rationalFloatError reports the committed |rational - held| — zero wherever
// the true flux is a float64, which is what lets an all-Plane cap-loop band
// publish the Exact volume §8.4 owes it. Evaluating the same identity in
// floats forfeits that Exact and cannot get it back: a triple product cancels,
// so the only honest budget for a float evaluation is an envelope of the
// ABSOLUTE terms it is built from (tripleProductUpper), and an envelope over
// an exact value is still positive, which publishes Approximate. The float
// path therefore survives only as the fallback for a patch whose coordinates
// do not lift (a non-finite one), where the float result is no better.
//
// A Cone patch keeps the float closed form and its envelope bound for the same
// reason in reverse: its flux passes through math.Sincos, whose result no
// rational carries, so a mixed section holding one circular wall stays
// Approximate however exactly its Plane patches integrate.
func patchRawFlux(g capPatchGeom) boundedScalar {
	if !g.circular {
		v0 := r3.NewVec(g.sideA.U, g.sideA.V, g.sideZ)
		v1 := r3.NewVec(g.sideB.U, g.sideB.V, g.sideZ)
		v2 := r3.NewVec(g.capB.U, g.capB.V, g.capZ)
		v3 := r3.NewVec(g.capA.U, g.capA.V, g.capZ)
		if exact, ok := exactPlanePatchFlux(v0, v1, v2, v3); ok {
			held, _ := exact.Float64()
			return measuredScalar(held, rationalFloatError(exact, held))
		}
		tri := func(a, b, c r3.Vec) float64 { return a.Dot(b.Cross(c)) }
		value := 0.5*tri(v0, v1, v2) + 0.5*tri(v0, v2, v3)
		// tripleProductUpper is an upper bound on every intermediate of one
		// triple product AND on the product itself, so the two together are
		// the envelope of the whole expression; the dropped halvings only
		// widen it.
		env := absSumUpper(tripleProductUpper(v0, v1, v2), tripleProductUpper(v0, v2, v3))
		return measuredScalar(value, analyticRoundBound(env))
	}
	R0, R1 := g.sideRadius, g.capRadius
	z0, z1 := g.sideZ, g.capZ
	th0, th1 := g.th0, g.th1
	H := z1 - z0
	dR := R1 - R0
	dth := th1 - th0
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	sc := sin1 - sin0
	ss := cos0 - cos1
	term1 := H * (R0 + R1) / 2 * (g.cU*sc + g.cV*ss)
	term2 := H * dth * (R0*R0 + R0*R1 + R1*R1) / 3
	term3 := -dR * dth * (z0*R0 + (z0*dR+H*R0)/2 + H*dR/3)
	flux := term1 + term2 + term3

	absH, absDth, absDR := math.Abs(H), math.Abs(dth), math.Abs(dR)
	absR0, absR1, absZ0 := math.Abs(R0), math.Abs(R1), math.Abs(z0)
	// polyEnv envelopes term2 and term3 and every intermediate of both: the
	// same products over absolute operands, with the 1/2 and 1/3 divisors
	// dropped because dropping a divisor above one only widens an envelope.
	// These two terms carry no libm result, so their rounding is ordinary
	// basic arithmetic and analyticRoundBound is the budget for it.
	polyEnv := absSumUpper(
		productUpper(productUpper(absH, absDth), absSumUpper(
			productUpper(absR0, absR0), productUpper(absR0, absR1), productUpper(absR1, absR1))),
		productUpper(productUpper(absDR, absDth), absSumUpper(
			productUpper(absZ0, absR0), productUpper(absZ0, absDR),
			productUpper(absH, absR0), productUpper(absH, absDR))),
	)
	// term1 is the eccentric term, and it is the one that passes through
	// math.Sincos. moments.go's analyticRoundBound states the rule it breaks:
	// Go gives Sin and Cos no ulp contract, so a result computed through them
	// never rests on that roundoff budget. trigEnv is the magnitude envelope
	// that stands in its place — |sin| and |cos| never exceed 1, so |sc| and
	// |ss| never exceed 2 and the TRUE term1 never exceeds
	// |H|·(R0+R1)/2 · 2·(|cU|+|cV|), whatever the platform's libm returns.
	trigEnv := productUpper(productUpper(absH, absSumUpper(absR0, absR1)), absSumUpper(g.cU, g.cV))
	trigBound := conservativeValueError(term1, trigEnv)
	if g.wholeTurn {
		// A full period integrates both cos and sin to exactly zero, so this
		// patch's TRUE term1 is zero and the held value is its own whole
		// error — a fact about the surface, read off capblend_geom.go's
		// structural flag and never off a comparison of th0 and th1.
		trigBound = upRound(math.Abs(term1))
	}
	bound := absSumUpper(analyticRoundBound(absSumUpper(polyEnv, trigEnv)), trigBound)
	if !g.sweepCCW {
		// The integral is taken over the NORMALIZED window th0 < th1, which
		// orients the patch radially OUTWARD from its own centre. That is the
		// orientation the patch's own walk gives only while the walk runs
		// counter-clockwise; a clockwise-walked one (a hole's whole circle, a
		// concave arc, and every reflex corner's apex connector) bounds the
		// mirror surface, so its flux is negated back to the sense its walk
		// actually has. The caller then rotates the whole band into the
		// virtual band's own orientation.
		return measuredScalar(-flux, bound)
	}
	return measuredScalar(flux, bound)
}

// exactPlanePatchFlux is the flat quad patch's raw flux
// (1/2)*v0.(v1 x v2) + (1/2)*v0.(v2 x v3) as an EXACT rational. Every
// coordinate it reads is one of the payload's own float64s, hence an exact
// rational already, and every operation the identity performs is an addition,
// a subtraction or a multiplication — closed over the rationals — so the
// returned value is the true flux of the quad the topology actually built,
// with no rounding anywhere on the way. Its one caller rounds it to a float64
// once and reports that single rounding as the bound.
//
// It reports ok=false rather than panicking where a coordinate does not lift,
// which is exactly the non-finite case: a public measurement must refuse or
// bound, never abort, and its caller has a float evaluation to fall back on
// that is no worse.
func exactPlanePatchFlux(v0, v1, v2, v3 r3.Vec) (*big.Rat, bool) {
	lift := func(v r3.Vec) (ratV3, bool) {
		x, y, z := floatRat(v.X), floatRat(v.Y), floatRat(v.Z)
		if x == nil || y == nil || z == nil {
			return ratV3{}, false
		}
		return ratV3{x, y, z}, true
	}
	r0, ok0 := lift(v0)
	r1, ok1 := lift(v1)
	r2, ok2 := lift(v2)
	r3v, ok3 := lift(v3)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return nil, false
	}
	sum := new(big.Rat).Add(rvDot(r0, rvCross(r1, r2)), rvDot(r0, rvCross(r2, r3v)))
	return sum.Mul(sum, big.NewRat(1, 2)), true
}

// tripleProductUpper bounds |a·(b×c)| and every intermediate the float
// evaluation of it passes through: each cross-product component is a
// difference of two of the six products below, and each final product is one
// of a_i times such a difference, so their absolute sum dominates all of
// them. It is the envelope analyticRoundBound is owed for a determinant,
// whose own value cancels to an arbitrarily small fraction of it.
func tripleProductUpper(a, b, c r3.Vec) float64 {
	return absSumUpper(
		productUpper(math.Abs(a.X), productUpper(math.Abs(b.Y), math.Abs(c.Z))),
		productUpper(math.Abs(a.X), productUpper(math.Abs(b.Z), math.Abs(c.Y))),
		productUpper(math.Abs(a.Y), productUpper(math.Abs(b.Z), math.Abs(c.X))),
		productUpper(math.Abs(a.Y), productUpper(math.Abs(b.X), math.Abs(c.Z))),
		productUpper(math.Abs(a.Z), productUpper(math.Abs(b.X), math.Abs(c.Y))),
		productUpper(math.Abs(a.Z), productUpper(math.Abs(b.Y), math.Abs(c.X))),
	)
}

// crossProductUpper bounds every component of a×b and every product the float
// evaluation forms on the way: each component is a difference of two of the
// six products below, so their absolute sum dominates all of them AND the
// resulting vector's own norm. A band patch is a thin ruled quad, so that
// difference is exactly where its cross product cancels — the wall direction
// crossed with the offset displacement is smaller than either term by the
// ratio of the wall's length to the setback — and a budget read off the
// surviving area under-counts by that ratio.
func crossProductUpper(a, b r3.Vec) float64 {
	return absSumUpper(
		productUpper(math.Abs(a.Y), math.Abs(b.Z)),
		productUpper(math.Abs(a.Z), math.Abs(b.Y)),
		productUpper(math.Abs(a.Z), math.Abs(b.X)),
		productUpper(math.Abs(a.X), math.Abs(b.Z)),
		productUpper(math.Abs(a.X), math.Abs(b.Y)),
		productUpper(math.Abs(a.Y), math.Abs(b.X)),
	)
}

// patchAreaOf is one patch's own surface area and the proven bound on it:
// a two-triangle cross-product sum for a Plane, a closed form for a Cone. The
// Plane reading is NEVER exact. Its two terms are each a float subtraction, a
// float cross product and a float norm — r3's own Len is a nested Hypot, which
// carries no ulp contract at all — and their sum rounds once more, so the held
// value is a float evaluation and sumSlop is the proven, never-zero bound
// bounds.go keeps for that shape (the same one boolean_body.go's mesh facet
// areas take). Returning a zero bound there would publish an Exact the
// arithmetic has not earned, on the face AND in the body's own area sum.
//
// sumSlop alone is not the whole of it, because it charges each term a few
// ulps of the term's OWN value and a band patch's cross product cancels
// before that value is reached. crossProductUpper carries the envelope those
// products actually reach, so the charge tracks the terms rather than what
// they cancelled to — the same correction capBandVolume makes for the flux.
func patchAreaOf(g capPatchGeom) (float64, float64) {
	if !g.circular {
		v0 := r3.NewVec(g.sideA.U, g.sideA.V, g.sideZ)
		v1 := r3.NewVec(g.sideB.U, g.sideB.V, g.sideZ)
		v2 := r3.NewVec(g.capB.U, g.capB.V, g.capZ)
		v3 := r3.NewVec(g.capA.U, g.capA.V, g.capZ)
		a1 := v1.Sub(v0).Cross(v2.Sub(v0)).Len() / 2
		a2 := v2.Sub(v0).Cross(v3.Sub(v0)).Len() / 2
		area := a1 + a2
		crossEnv := absSumUpper(
			crossProductUpper(v1.Sub(v0), v2.Sub(v0)),
			crossProductUpper(v2.Sub(v0), v3.Sub(v0)),
		)
		return area, absSumUpper(sumSlop(2, absSumUpper(a1, a2)), analyticRoundBound(crossEnv))
	}
	R0, R1 := g.sideRadius, g.capRadius
	H := g.capZ - g.sideZ
	dR := R1 - R0
	slant := math.Hypot(dR, H)
	dth := math.Abs(g.th1 - g.th0)
	area := dth / 2 * (R0 + R1) * slant
	return area, conservativeValueError(area, dth*(R0+R1)*(math.Abs(dR)+math.Abs(H)))
}

// capBlendBoundsContext is the placed body's axis-aligned bounding box, read
// along each world axis from the payload's OWN exact patch extrema
// (extentAlongWork, Table DX row DX5) exactly as prismBoundsContext reads a
// prism's — so every face of the box is a value the body attains, and the
// reported zero Bound is the truth about it. Min and Max are positions and
// Bound is the absolute error on them (measurement.go), so a box widened
// outward by the setback d would be a Min sitting d millimetres from the true
// extreme while claiming an error of zero; §8.4 asks for bounds from patch
// extrema for that reason, and capblend.go's extentAlong already states why
// padding the receiver prism by d proves nothing about attainment.
func capBlendBoundsContext(ctx context.Context, cbp capBlendPayload, work *freeformWork) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, err := cbp.extentAlongWork(ctx, axis, work)
		if err != nil {
			return Box{}, err
		}
		minC[i], maxC[i] = lo, hi
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// capBlendCentroidEstimate is an area-weighted average of every face's own
// representative point, with the geometric safety-net bound every analytic
// centroid already falls back to: the true centroid lies within the returned
// Bounds box, so |estimate-true| is bounded by the box's own reach from the
// estimate — sound whatever the estimate's own accuracy.
//
// The reach is maximized over all EIGHT corners of the box, and that is the
// whole of the proof rather than a thoroughness flourish. p -> |p - estimate| is
// convex, so its maximum over the box — a convex hull of its eight corners — is
// attained AT a corner; taking the max over all eight therefore bounds the
// distance to every point the box holds, the true centroid among them, wherever
// the estimate itself sits. Reading only Min and Max leaves six corners
// unexamined, and a box whose extent along one axis is far larger than along
// another puts its farthest corner among exactly those six: the reported bound
// is then smaller than the estimate's own error and encloses nothing.
func capBlendCentroidEstimate(faces []*Face, bounds Box) (r3.Vec, float64) {
	var sum r3.Vec
	var totalArea float64
	for _, f := range faces {
		p := faceRepresentativePoint(f)
		sum = sum.Add(p.Scale(f.area))
		totalArea += f.area
	}
	estimate := r3.NewVec(0, 0, 0)
	if totalArea > 0 {
		estimate = sum.Scale(1 / totalArea)
	}
	xs := [2]float64{bounds.Min.X, bounds.Max.X}
	ys := [2]float64{bounds.Min.Y, bounds.Max.Y}
	zs := [2]float64{bounds.Min.Z, bounds.Max.Z}
	reach := 0.0
	for _, x := range xs {
		for _, y := range ys {
			for _, z := range zs {
				dd := r3.NewVec(x, y, z).Sub(estimate).Len()
				if dd > reach {
					reach = dd
				}
			}
		}
	}
	return estimate, upRound(reach)
}

// faceRepresentativePoint is any point provably on the face, used only to
// seed the centroid estimate — not a claim of precision; the reported bound
// is the geometric safety net above, not this point's own accuracy.
func faceRepresentativePoint(f *Face) r3.Vec {
	for _, l := range f.loops {
		for _, ce := range l.coedges {
			if v := ce.Start(); v != nil {
				return v.position
			}
		}
	}
	switch s := f.surface.(type) {
	case Plane:
		return s.Frame.Origin()
	case Cylinder:
		return s.Origin
	case Cone:
		return s.Origin
	}
	return r3.NewVec(0, 0, 0)
}
