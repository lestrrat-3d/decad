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
	// muTotal, mvTotal, mzTotal accumulate the body's own plane-local first
	// moments (docs/modify-reach-design.md §8.4's fourth reading): the SAME
	// per-loop sign this loop already applies to slabVolume/bandVolume, over
	// the SAME two-part decomposition (a signed slab term via
	// loopEnclosedMomentsContext, a band term via capBandMoment).
	var muTotal, mvTotal, mzTotal boundedScalar
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

		// A chamfered end pulls its own straight level in by the setback. Its
		// unit conversion and float sum both round. The rounding is an ulp of
		// the SWEEP, but it multiplies the whole section area below, so it
		// reaches the volume at the scale of the band itself and is charged here
		// — the same term capBandVolume charges for the identical level it reads
		// as sideZ.
		zLo, zHi := measuredScalar(cbp.z0, cbp.z0Delta), measuredScalar(cbp.z1, cbp.z1Delta)
		setback := measuredScalar(cbp.d, cbp.dDelta)
		if onStart {
			zLo = boundedAdd(zLo, setback)
		}
		if onEnd {
			zHi = boundedSub(zHi, setback)
		}
		// The side walls are built over those same two bounded levels, so each
		// end's displacement goes in with it: the wall vertices, the vertical
		// edge lengths and the side face areas all read a level the setback
		// computed, never one they can claim was recorded.
		ppFor := prismPayload{
			frame:   cbp.frame,
			z0:      zLo.value,
			z1:      zHi.value,
			z0Delta: zLo.bound,
			z1Delta: zHi.bound,
			xform:   cbp.xform,
		}
		sideFaces, bottomCo, topCo, loopLen, err := buildLoopSidesAs(ctx, body, ref, ppFor, li, li != 0, loop, work, nil)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		for _, f := range sideFaces {
			sideArea = boundedAdd(sideArea, measuredScalar(f.area, f.areaBound))
		}
		_ = loopLen

		loopArea, loopMu, loopMv, err := loopEnclosedMomentsContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		straightHeight := boundedSub(zHi, zLo)
		loopSlab := boundedMul(loopArea, straightHeight)
		slabVolume = boundedAdd(slabVolume, measuredScalar(sign*loopSlab.value, loopSlab.bound))

		// M_slab = (mu·h, mv·h, A·h·(zLo+zHi)/2), docs/modify-reach-design.md
		// §8.4(a): mu, mv are the loop's own signed first moments (already
		// canonicalized the same way loopArea is), and A·h is loopSlab itself,
		// so the z component reuses it rather than recomputing A·h a second
		// time.
		zMid := boundedDiv(boundedAdd(zLo, zHi), exactScalar(2))
		loopMuSlab := boundedMul(loopMu, straightHeight)
		loopMvSlab := boundedMul(loopMv, straightHeight)
		loopMzSlab := boundedMul(loopSlab, zMid)
		muTotal = boundedAdd(muTotal, measuredScalar(sign*loopMuSlab.value, loopMuSlab.bound))
		mvTotal = boundedAdd(mvTotal, measuredScalar(sign*loopMvSlab.value, loopMvSlab.bound))
		mzTotal = boundedAdd(mzTotal, measuredScalar(sign*loopMzSlab.value, loopMzSlab.bound))

		startCo, endCo := bottomCo, topCo

		// startBand/endBand default to the zero capBandResult (delta 0, capCo
		// nil) where the loop is not chamfered on that cap, which is what
		// leaves the area correction below at its own zero — the delta<=0
		// guards in sectionDisplacementArea and bandPatchAreaAllow, not a
		// separate onStart/onEnd branch.
		var startBand, endBand capBandResult
		if onStart {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z0, +1, bottomCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			startCo = band.capCo
			startBand = band
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z0, +1, band.delta)
			if err != nil {
				return nil, err
			}
			bandVolume = boundedAdd(bandVolume, measuredScalar(sign*v.value, v.bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = boundedAdd(patchArea, measuredScalar(pa, pb))
			}
			bmu, bmv, bmz, err := capBandMoment(ctx, loop, cbp, band.geom, cbp.z0, +1, band.delta, work)
			if err != nil {
				return nil, err
			}
			muTotal = boundedAdd(muTotal, measuredScalar(sign*bmu.value, bmu.bound))
			mvTotal = boundedAdd(mvTotal, measuredScalar(sign*bmv.value, bmv.bound))
			mzTotal = boundedAdd(mzTotal, measuredScalar(sign*bmz.value, bmz.bound))
		}
		if onEnd {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z1, -1, topCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			endCo = band.capCo
			endBand = band
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z1, -1, band.delta)
			if err != nil {
				return nil, err
			}
			bandVolume = boundedAdd(bandVolume, measuredScalar(sign*v.value, v.bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = boundedAdd(patchArea, measuredScalar(pa, pb))
			}
			bmu, bmv, bmz, err := capBandMoment(ctx, loop, cbp, band.geom, cbp.z1, -1, band.delta, work)
			if err != nil {
				return nil, err
			}
			muTotal = boundedAdd(muTotal, measuredScalar(sign*bmu.value, bmu.bound))
			mvTotal = boundedAdd(mvTotal, measuredScalar(sign*bmv.value, bmv.bound))
			mzTotal = boundedAdd(mzTotal, measuredScalar(sign*bmz.value, bmz.bound))
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
		// sArea is measured on the BUILT cap contour (loopEnclosedAreaContext's
		// own arithmetic bound), which says nothing about how far that contour
		// sits from the one the offset DENOTES. sectionDisplacementArea is the
		// same 2D set-displacement identity extrude.go's own region area
		// composes (docs/prism-boolean-design.md §7, one dimension down from
		// sweptVolumeAllow above): a set bound, sound even where the
		// displacement changes which regions the offset merged, charged
		// against this loop's own held boundary via capContourPerimeterUpper
		// so it stands whatever the corner geometry did.
		sBound := absSumUpper(sArea.bound,
			sectionDisplacementArea(startBand.delta, len(loop.Segments), capContourPerimeterUpper(startBand.capCo)))
		startArea = boundedAdd(startArea, measuredScalar(sign*sArea.value, sBound))

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
		eBound := absSumUpper(eArea.bound,
			sectionDisplacementArea(endBand.delta, len(loop.Segments), capContourPerimeterUpper(endBand.capCo)))
		endArea = boundedAdd(endArea, measuredScalar(sign*eArea.value, eBound))
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
		surface:       Plane{Frame: startFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:          body,
		loops:         startLoopObjs,
		area:          startArea.value,
		areaBound:     startArea.bound,
		axialDelta:    cbp.z0Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       Plane{Frame: endFrame},
		origins:       []FeatureRef{{Step: ref, Role: roleCapEnd}},
		body:          body,
		loops:         endLoopObjs,
		area:          endArea.value,
		areaBound:     endArea.bound,
		axialDelta:    cbp.z1Delta,
		hasAxialDelta: true,
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

	// Centroid: the closed-form first moments divided by the body's own
	// volume (docs/modify-reach-design.md §8.4), lifted to world through the
	// SAME frame/placement lift a prism centroid uses, backed by the
	// geometry safety-net ceiling every analytic centroid falls back to.
	bounds, err := capBlendBoundsContext(ctx, cbp, work)
	if err != nil {
		return nil, err
	}
	cu := boundedQuotient(muTotal.value, muTotal.bound, volume.value, volume.bound)
	cv := boundedQuotient(mvTotal.value, mvTotal.bound, volume.value, volume.bound)
	cz := boundedQuotient(mzTotal.value, mzTotal.bound, volume.value, volume.bound)
	centroidValue := pl.point(cu.value, cv.value, cz.value)
	formulaBound := prismPointBound(pl, cu, cv, cz)
	geometryBound := capBlendCentroidGeometryBound(centroidValue, bounds)
	centroidBound := math.Min(formulaBound, geometryBound)
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

// capContourPerimeterUpper is a proven upper bound on the total length of a
// chamfer band's own cap-level boundary, read off the coedges buildCapBand
// already built for it (band.capCo) rather than re-walked: each edge on that
// boundary already carries its own held length and proven bound (a wall's
// straight or circular capEdge, a reflex corner's connector arc), computed
// from the SAME offset math loopEnclosedAreaContext(capLoopBoundary(...))
// measures the area of, so the two readings of one contour never disagree
// about which boundary they describe. A nil capCo (the loop is not chamfered
// on this cap) sums to zero, which is what leaves sectionDisplacementArea's
// own delta<=0 guard as the only gate that matters.
func capContourPerimeterUpper(capCo []coedge) float64 {
	total := boundedScalar{}
	for _, ce := range capCo {
		total = boundedAdd(total, measuredScalar(ce.edge.length, ce.edge.lengthBound))
	}
	return absSumUpper(total.value, total.bound)
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
//
// None of those terms speaks for the cap contour's own displacement (delta,
// capblend_contour.go): capArea and every patchRawFlux term above read the
// contour's coordinates as exact inputs, but the contour is a COMPUTED offset
// that sits within delta of the one it denotes. That is one displacement
// acting on the whole surface the band closes on — this loop's patches AND
// the cap disk they meet — so it is composed ONCE here, after the flux sum,
// via bounds.go's sweptVolumeAllow(delta, areaUpper): charging it inside
// capArea's own bound, or inside each patchRawFlux term, would count the SAME
// displaced coordinates twice, since patchRawFlux already reads them.
func capBandVolume(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign, delta float64) (boundedScalar, error) {
	capZB := cbp.capBandLevel(capZ, matSign)
	sideZB := boundedAdd(capZB, measuredScalar(matSign*cbp.d, cbp.dDelta))
	sideZ := sideZB.value
	// sideZ multiplies a whole disk area below, so its own rounding is a term
	// of the band and is charged here — an error the size of an ulp of the
	// sweep height, amplified by the section's area, which is of the order of
	// the band itself once the disks cancel. sideZB also preserves the cap
	// level's inherited axial displacement: the side level is derived from
	// that computed cap, not from an exact coordinate.
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
	// level keeps whatever bound it arrived with: capZ can inherit a body-
	// relative stop's axial displacement, and sideZ adds its own rounding to
	// that same displacement.
	fluxTotal := boundedAdd(
		boundedMul(measuredScalar(capZB.value*(-matSign), capZB.bound), capArea),
		boundedMul(measuredScalar(sideZ*matSign, sideZB.bound), sideArea),
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
	patchAreaTotal := boundedScalar{}
	for _, g := range geom {
		f := patchRawFlux(g)
		fluxTotal = boundedAdd(fluxTotal, measuredScalar(-matSign*orient*f.value, f.bound))
		pa, pb := patchAreaOf(g)
		patchAreaTotal = boundedAdd(patchAreaTotal, measuredScalar(pa, pb))
	}
	result := boundedQuotient(fluxTotal.value, fluxTotal.bound, 3, 0)
	// areaUpper is the surface the contour's own displacement acted on: this
	// band's patches plus the cap disk they close on (capArea) — the same two
	// terms patchRawFlux and the disk flux above both read displaced
	// coordinates from.
	areaUpper := absSumUpper(patchAreaTotal.value, patchAreaTotal.bound, capArea.value, capArea.bound)
	result.bound = absSumUpper(result.bound, sweptVolumeAllow(delta, areaUpper))
	return result, nil
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
//
// A regular wall's Cone patch (capPatchGeom.circular with a nonzero
// sideRadius, i.e. neither a reflex corner's apex patch nor a cornerless
// whole circle) is bounded by two STRAIGHT rulings between two directrices
// that, at a non-tangential miter corner, sweep DIFFERENT angular windows
// (th0/th1 the side, capTh0/capTh1 the trimmed cap — capblend_geom.go). What
// this function integrates is the exact flux of THAT ruled patch — the
// straight-Line3-bounded surface the topology actually builds — and its bound
// above is sound for exactly that solid, but it is NOT a bound against the
// DENOTED chamfer: the true miter locus at axial fraction s is the section
// offset by s*d (docs/modify-reach-design.md §8.3's own reduction), a curve
// the straight ruling only touches at s=0 and s=1 and chords in between. That
// gap is real, grows as the setback approaches the section's own inradius (the
// cap window closes hard against the side one), and can exceed the arithmetic
// bound above by several times over — bounds.go's chordLocusVolumeAllow is the
// dedicated proven term for it (chordLocusResidualAllow below gathers this
// patch's own inputs to it), added into the bound whenever the two windows
// genuinely differ and left at its own zero, unchanged, wherever they
// coincide. TestCapBlendErosionFamilyVolumeBoundEncloses judges the composed
// bound against an independent erosion-family reference, including the
// setback-limit family this term exists for.
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
	thS0, thS1 := g.th0, g.th1
	thC0, thC1 := g.capTh0, g.capTh1
	H := z1 - z0
	dS := thS1 - thS0
	dC := thC1 - thC0

	// The STRAIGHT-SLANT RULED PATCH (docs/modify-reach-design.md §8.3): the
	// surface between the SIDE arc (radius R0, level z0, sweeping thS0..thS1,
	// the wall's own full recorded window) and the CAP arc (radius R1, level
	// z1, sweeping thC0..thC1, the offset TRIMMED at the mitered corner
	// feet), joined by STRAIGHT rulings — exactly the Line3 slant edges the
	// topology actually builds — never a single rotationally-symmetric cone
	// sector spanning one shared window (the old, wrong shape whenever a
	// circular wall meets a neighbour at a non-tangential miter corner).
	//
	// P(u, v) = (1-v)*(R0*cos(thS(u)), R0*sin(thS(u)), z0)
	//         +    v *(R1*cos(thC(u)), R1*sin(thC(u)), z1),
	// thS(u) = thS0 + u*dS, thC(u) = thC0 + u*dC, u, v in [0, 1] — matching
	// all four boundary edges exactly (the two arcs at v=0/v=1, the two
	// slants at u=0/u=1). Its raw flux P.(Pu x Pv), integrated over the unit
	// square, splits — by linearity in the plane-local origin shift (cU, cV)
	// — into an ORIGIN term (one sub-term per directrix, generalizing the old
	// shared-window eccentric term1) and a LOCAL term: a pure polynomial
	// piece (poly, generalizing term2+term3) plus one CROSS term whose own
	// trig factor, ruledAngleCos, is closed form because thS(u) and thC(u)
	// are both linear in u (a product-to-sum reduction, not a numerical
	// integral). thS0=thC0 and dS=dC (a tangent join, or the degenerate
	// single-window cases below) collapses this exactly onto the OLD
	// term1+term2+term3 formula — verified algebraically and numerically,
	// never merely asserted.
	sinS0, cosS0 := math.Sincos(thS0)
	sinS1, cosS1 := math.Sincos(thS1)
	sinC0, cosC0 := math.Sincos(thC0)
	sinC1, cosC1 := math.Sincos(thC1)
	originR0 := H / 2 * R0 * (g.cU*(sinS1-sinS0) + g.cV*(cosS0-cosS1))
	originR1 := H / 2 * R1 * (g.cU*(sinC1-sinC0) + g.cV*(cosC0-cosC1))
	origin := originR0 + originR1

	// poly and cross are regrouped about the band's OWN z origin — z0, the
	// side level — rather than left as functions of the two absolute levels
	// z0 AND z1 independently: z1 = z0 + H splits each into a piece
	// proportional to z0 (the band's own position along the sweep, which can
	// sit arbitrarily far from the plane-local origin) and a piece
	// proportional to H alone (the band's own small axial extent) — the same
	// split the cU/cV origin term above already makes for the in-plane
	// eccentricity, and a constant z shift across every patch of a band and
	// the cap disk it closes on cancels in the closed-surface sum the same
	// way. Both identities are exact (reassociated, not approximated):
	// R0²z1dS/2 - R1²z0dC/2 = z0·(R1²dSC - dS·dR·(R0+R1))/2 + R0²·H·dS/2, and
	// R0R1/2·(z1dC - z0dS) = z0·R0R1·(dC-dS)/2 + R0R1·H·dC/2. dSC = dS - dC is
	// the WINDOW SKEW — exactly zero at a tangent join or either degenerate
	// patch (capTh0/capTh1 literally th0/th1) — so at every one of those
	// already-shipped junctions the z0-proportional part of poly collapses to
	// -z0·dS·dR·(R0+R1)/2, the SAME dR-scaled product the pre-restructure
	// term3 already carried, and cross's z0-proportional part vanishes
	// outright, rather than the unconstrained R0²/R1² an envelope read
	// straight off z0, z1 independently would carry.
	dR := R1 - R0
	dSC := dS - dC
	polyZ0 := R1*R1*dSC - dS*dR*(R0+R1)
	poly := z0*polyZ0/2 + R0*R0*H*dS/2
	crossZ0 := -R0 * R1 * dSC
	cross := z0*crossZ0/2 + R0*R1*H*dC/2

	absH, absR0, absR1 := math.Abs(H), math.Abs(R0), math.Abs(R1)
	absZ0, absDS, absDC, absDR, absDSC := math.Abs(z0), math.Abs(dS), math.Abs(dC), math.Abs(dR), math.Abs(dSC)
	// polyEnv envelopes poly and every intermediate of the regrouped form
	// above: the z0-proportional coefficient's own envelope (a window-skew
	// term plus a dR-scaled one, both zero-or-small at the already-shipped
	// junctions) multiplied by |z0|, plus the H-only piece — with the /2
	// divisors dropped because dropping a divisor above one only widens an
	// envelope. It carries no libm result, so its rounding is ordinary basic
	// arithmetic and analyticRoundBound is the budget for it.
	polyZ0Env := absSumUpper(
		productUpper(productUpper(absR1, absR1), absDSC),
		productUpper(productUpper(absDS, absDR), absSumUpper(absR0, absR1)),
	)
	polyEnv := absSumUpper(
		productUpper(absZ0, polyZ0Env),
		productUpper(productUpper(absR0, absR0), productUpper(absH, absDS)),
	)
	// origin and cross both carry a trig factor — origin through Sincos
	// directly, cross through ruledAngleCos's cos/sinc — and moments.go's
	// analyticRoundBound states the rule both break: Go gives Sin, Cos and
	// Atan2 no ulp contract, so a result computed through them never rests on
	// that roundoff budget alone. originEnv/crossEnv are the STRUCTURAL
	// magnitude envelopes that stand in its place instead — never read off
	// the computed value, so they bound the TRUE sub-term regardless of what
	// the platform's libm returned: |sin| and |cos| never exceed 1, so
	// origin's magnitude never exceeds |H|*(|R0|+|R1|)*(|cU|+|cV|), and
	// |ruledAngleCos| never exceeds 1 (|cos| <= 1, |sinc| <= 1 for every
	// real argument), so cross's magnitude never exceeds the same
	// z0-proportional-plus-H-only envelope poly's own regrouping gives it.
	originEnv := productUpper(productUpper(absH, absSumUpper(absR0, absR1)), absSumUpper(g.cU, g.cV))
	crossZ0Env := productUpper(productUpper(absR0, absR1), absDSC)
	crossEnv := absSumUpper(
		productUpper(absZ0, crossZ0Env),
		productUpper(productUpper(absR0, absR1), productUpper(absH, absDC)),
	)

	var flux, trigBound float64
	if g.wholeTurn {
		// Structural: this patch's window is a genuinely FULL period built
		// from the SAME floats on both directrices (capblend_geom.go's
		// cornerless closed circle branch, and every apex patch's degenerate
		// R0 = 0 side), so thetaS(u) == thetaC(u) identically for every u —
		// ruledAngleCos's TRUE value is exactly 1, never read off Sincos or
		// sinc here at all — and origin's own TRUE value is exactly zero (a
		// full period integrates both cos and sin to zero, capblend_geom.go's
		// structural flag, never a comparison of the windows). The held
		// origin is therefore its own whole error, and cross (now ordinary
		// arithmetic, no trig) is charged the same rounding budget poly is.
		flux = poly + cross + origin
		trigBound = upRound(math.Abs(origin))
	} else {
		intCos := ruledAngleCos(thS0, thS1, thC0, thC1)
		trig := origin + cross*intCos
		flux = poly + trig
		trigBound = conservativeValueError(trig, absSumUpper(originEnv, crossEnv))
	}
	bound := absSumUpper(analyticRoundBound(absSumUpper(polyEnv, originEnv, crossEnv)), trigBound)
	if !g.wholeTurn {
		// The arithmetic bound above is only for the STRAIGHT-RULED patch
		// this evaluator actually builds; at a non-tangential corner (where
		// the cap-level window genuinely differs from the side-level one) it
		// says nothing about that patch's own gap from the TRUE curved miter
		// locus the construction denotes (bounds.go's chordLocusVolumeAllow).
		bound = absSumUpper(bound, chordLocusResidualAllow(g))
	}
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

// chordLocusResidualAllow gathers this ONE patch's own inputs to
// bounds.go's chordLocusVolumeAllow: the two reference cone-sector fluxes
// (the wide, side-window-only reading and the narrow, cap-window-only one —
// both this same patchRawFlux formula, degenerate to the ordinary
// rotationally-symmetric cone sector once a patch's two directrices share one
// window) and this patch's own held area, the surface bounds.go's
// sweptVolumeAllow needs. Zero wherever the two windows already coincide (a
// tangent join, or either degenerate patch), matching every already-shipped
// reading those configurations publish.
//
// g.capTh0/g.capTh1 and g.th0/g.th1 are not guaranteed to share a branch:
// capWallSweep (capblend_geom.go) anchors capTh0 at a raw Atan2, always in
// (-pi, pi], while th0 comes from the recorded arc range and can sit a full
// turn away for a major-arc wall — the same corner, described a multiple of
// 2*pi apart. capWindowOnBranch puts the cap window back on th0's own branch,
// shifting capTh0 and capTh1 together so the window's WIDTH (capTh1-capTh0)
// is untouched, before either is differenced against th0/th1 below; every
// other reader of these two fields (patchAreaOf, patchRawFlux) only ever
// takes a WITHIN-pair difference (a width), which a shared branch shift
// cannot change, so this is the one site the mismatch reaches.
func chordLocusResidualAllow(g capPatchGeom) float64 {
	capTh0, capTh1 := capWindowOnBranch(g.capTh0, g.capTh1, g.th0)
	windowSkewMax := math.Max(capTh0-g.th0, g.th1-capTh1)
	// windowSkewMax is never negative: this is a checkable fact, not a
	// defensive assumption. The cap window is the SIDE window trimmed by
	// erosion — the cap contour is the wall's own offset, and offsetting a
	// point radially preserves its polar angle about the wall's centre — so
	// the cap window is always a SUBSET of the side window and can only be
	// narrower or equal, never wider. For a circle/line miter this reduces to
	// a closed form: with the corner's own half-angle cosine a = cos(alpha)
	// and setback d, the per-corner skew is a' - a = d(1-a)/(R+d) >= 0 for a
	// hole wall (offset radius R+d) and d(1+a)/(R-d) >= 0 for an outer wall
	// (offset radius R-d) — both non-negative for every admitted 0 < d < R
	// and -1 <= a <= 1, zero only at a tangent join (a = 1) or a
	// degenerate/whole-turn patch, which is the only way this guard fires.
	if windowSkewMax <= 0 {
		return 0
	}
	wideGeom, narrowGeom := g, g
	wideGeom.capTh0, wideGeom.capTh1 = g.th0, g.th1
	narrowGeom.th0, narrowGeom.th1 = capTh0, capTh1
	narrowGeom.capTh0, narrowGeom.capTh1 = capTh0, capTh1
	wide := patchRawFlux(wideGeom)
	narrow := patchRawFlux(narrowGeom)
	pa, pb := patchAreaOf(g)
	return chordLocusVolumeAllow(wide.value, wide.bound, narrow.value, narrow.bound,
		g.sideRadius, g.capRadius, windowSkewMax, absSumUpper(pa, pb))
}

// capWindowOnBranch shifts the cap-level window (capTh0, capTh1) by the
// integer multiple of 2*pi that lands capTh0 nearest th0, preserving the
// window's own width (capTh1-capTh0) exactly — a pure rotation of the branch
// index, never of the angle the window actually spans. The two windows
// describe the SAME corner (capTh0 is the offset foot near the wall's own
// th0 corner), so the nearest branch is the true one wherever a genuine
// (non-tangent) miter's own skew stays under half a turn, which a chamfer
// setback small relative to the wall's own radius always keeps it.
func capWindowOnBranch(capTh0, capTh1, th0 float64) (float64, float64) {
	shift := 2 * math.Pi * math.Round((th0-capTh0)/(2*math.Pi))
	return capTh0 + shift, capTh1 + shift
}

// ruledAngleCos is the closed form of the ruled patch's one remaining
// integral, ∫[0,1] cos(thetaS(u) - thetaC(u))du with thetaS(u) = thS0 + u*dS
// and thetaC(u) = thC0 + u*dC both linear in u: writing phi0 = thS0-thC0,
// phi1 = thS1-thC1 and delta = dS-dC, the antiderivative is
// (sin(phi0+delta)-sin(phi0))/delta = (sin(phi1)-sin(phi0))/delta, which the
// product-to-sum identity restates as cos((phi0+phi1)/2)*sinc(delta/2) — the
// numerically stable form (no difference-of-sines cancellation as delta gets
// small, and no division at all at delta = 0, the equal-window case a tangent
// join or either degenerate patch reaches). Both factors are magnitude-capped
// at 1 for every real argument, which is what lets patchRawFlux's own
// envelope bound this whole term without trusting Sin/Cos's accuracy.
func ruledAngleCos(thS0, thS1, thC0, thC1 float64) float64 {
	phi0 := thS0 - thC0
	phi1 := thS1 - thC1
	delta := (thS1 - thS0) - (thC1 - thC0)
	return math.Cos((phi0+phi1)/2) * sincHalf(delta)
}

// sincHalf returns sin(x/2)/(x/2), continuous (and exactly 1, no library call
// at all) at x = 0.
func sincHalf(x float64) float64 {
	h := x / 2
	if h == 0 {
		return 1
	}
	return math.Sin(h) / h
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
//
// Neither arm's arithmetic bound speaks for the cap-level directrix's own
// contour displacement (capblend_contour.go): both read g's coordinates as
// exact inputs, but a chamfered patch's cap-level corner sits where a float
// offset solve put it, not where the offset denotes. g.contourAllow is that
// allowance, computed once at build time (bounds.go's bandPatchAreaAllow)
// from this patch's own held chord and slant, and it is zero wherever the
// band's contour displacement is zero — an axis-aligned section's exact
// miters — which is what keeps TestCapBlendPlanePatchVolumeIsExact's area
// arithmetic unchanged there.
//
// The SIDE level owes the same kind of allowance and both arms charge it:
// g.sideZ is capZ + matSign*d rounded to a float (g.levelDelta,
// capblend_geom.go), so the whole side directrix sits that far from the level
// it denotes, and an arm reading g.sideZ as an exact input bounds only the
// patch it BUILT. It is the dominant residual wherever the sweep is large
// enough to round that sum: on a right-triangle band (legs 9e4 x 3e6 mm, 1e15
// mm sweep, 0.2 mm chamfer) levelDelta is 0.04999999999999999 mm, and
// substituting the HELD side level for the denoted one in a 512-bit reference
// drops the three patches' residuals from 3358.207374649123 /
// 111990.60743768104 / 111940.24564437279 mm^2 to 5.236122866634288e-06 /
// 8.892307483919435e-06 / 2.7272066797437425e-06 mm^2. bounds.go's
// bandLevelAreaAllow is that charge, and it is zero wherever the sum is exact
// (an ordinary sweep and setback), which is what leaves every tight reading
// tight.
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
		// The quad's two directrices are its side-level chord (sideA -> sideB)
		// and its cap-level one (capA -> capB), both plane-local at their own
		// single level, so each 3D length IS its 2D one.
		levelAllow := bandLevelAreaAllow(g.levelDelta,
			absSumUpper(chordUpper2(g.sideA, g.sideB), chordUpper2(g.capA, g.capB)))
		bound := absSumUpper(sumSlop(2, absSumUpper(a1, a2)), analyticRoundBound(crossEnv), g.contourAllow, levelAllow)
		return area, bound
	}
	R0, R1 := g.sideRadius, g.capRadius
	H := g.capZ - g.sideZ
	dR := R1 - R0
	slant := math.Hypot(dR, H)
	// dth is the CAP-level (trimmed) sweep — the actual held chord's own
	// angle, capTh1-capTh0 — never the wall's own recorded thS1-thS0: a
	// regular wall's two directrices generally sweep different angles at a
	// non-tangent miter corner (docs/modify-reach-design.md §8.3), the same
	// fact patchRawFlux's ruled-patch integral reads off both windows.
	dth := math.Abs(g.capTh1 - g.capTh0)
	area := dth / 2 * (R0 + R1) * slant
	// This formula is the constant-slant frustum-sector shape unchanged; it
	// is never claimed exact even when the two windows coincide (a Plane
	// patch's own two-triangle bound above is the same kind of arithmetic-
	// only envelope, never a tight one). Where the windows genuinely differ,
	// the true ruled surface's own rulings are not all the same length —
	// this formula's own "slant" assumption — so windowSkew widens the
	// envelope by the reading the OTHER window would have given, a sound
	// (generous, not tight) allowance for the gap between the two: the true
	// area lies within the family this envelope already covers, and
	// windowSkew is zero wherever the windows coincide (a tangent join, or
	// either degenerate patch), leaving the rest of the bound unchanged.
	// A reviewer reported this term failing to enclose on a REACHABLE body — a
	// pi/2 sector sketch extruded 1e15 mm, cap chamfered 0.3 mm — quoting a
	// true ruled-patch area of 2.9431408325276074 mm^2 against a published
	// bound of 1.9647343287969778 mm^2. That reference is wrong, not this
	// bound: it applied TRAPEZOID weights (1,2,2,...,2,1 per axis) under
	// Simpson's 1/(9n^2) normalizer, so it returns exactly 4/9 of the
	// integral. Its ratio to a converged reference is 0.4444444457 at n=200,
	// 0.4444444447 at n=400 and 0.4444444445 at n=1200 — approaching 4/9
	// rather than shrinking, which a genuine discretization error would.
	// Tensor-product Gauss-Legendre on [0,1]^2 of |P_u x P_t| gives
	// 6.4373261229841905 (n=8) through 6.4373261229841869 (n=64), successive
	// deltas at float64 roundoff, and a correctly weighted composite Simpson
	// agrees at 6.4373261229841914 (n=800). The true residual there is
	// 0.6331514578014259, which this bound encloses with 3.10x margin. A sweep
	// of 451 reachable Cone patches (height 10..1e16, phi pi/6..6, d 0.05..4,
	// R 3/10/250) against GL-64/GL-96 found zero violations at a minimum
	// enclosure ratio of 2.875871x.
	//
	// What that sweep does NOT cover, and what this term is therefore still
	// only measured on: a side window of [0,2pi] against a cap window of
	// [0.25,0.251] does exceed the bound, but reaching it means assigning
	// capPatchGeom's fields directly. A regular wall's cap window is the
	// offset trimmed at its own miter feet, so the skew is corner-local, and
	// no reachable body in the sweep produced one.
	sideDth := math.Abs(g.th1 - g.th0)
	windowSkew := productUpper(math.Abs(sideDth-dth), productUpper(absSumUpper(R0, R1), slant))
	// The core term (everything but windowSkew, contourAllow and the level
	// allowance below, which stand unchanged either way) takes the SMALLER of
	// two independently sound
	// bounds: the unconditional envelope conservativeValueError always gives,
	// and coneFrustumAreaBracket's certified interval — sound wherever it
	// manages to build one, +Inf (hence never the min) where it cannot. This
	// can only shrink the published bound, never grow it.
	core := math.Min(
		conservativeValueError(area, dth*(R0+R1)*(math.Abs(dR)+math.Abs(H))),
		coneFrustumAreaBracket(R0, R1, H, dth, g.capThAllow, area),
	)
	// The level allowance is the one term neither reading of the core speaks
	// for: the bracket lifts H as an EXACT rational (it is a difference of two
	// payload floats, so it brackets the area of the patch this build HOLDS),
	// and the fallback envelope covers it only by accident of its own width.
	// The axial displacement is the side level's own rounding plus whatever
	// the H subtraction just above committed, and the frustum sector's two
	// directrix arcs are Δθ·R0 and Δθ·R1, read at an upper bound on the true
	// window rather than at the held one.
	levelAllow := bandLevelAreaAllow(
		absSumUpper(g.levelDelta, addRoundError(g.capZ, -g.sideZ, H)),
		productUpper(absSumUpper(dth, g.capThAllow), absSumUpper(R0, R1)),
	)
	bound := absSumUpper(core, windowSkew, g.contourAllow, levelAllow)
	return area, bound
}

// chordUpper2 is a PROVEN upper bound on the distance between two plane-local
// points, taken without trusting any libm contract: the 1-norm dominates the
// 2-norm, each coordinate difference is one float subtraction, and
// analyticRoundBound covers those two roundings at the sum's own magnitude.
func chordUpper2(a, b Point2) float64 {
	raw := absSumUpper(b.U-a.U, b.V-a.V)
	return absSumUpper(raw, analyticRoundBound(raw))
}

// coneFrustumAreaBracket is the certified interval bound on patchAreaOf's
// Cone-arm closed form A = (Δθ/2)·(R0+R1)·√(ΔR²+H²) — the constant-slant
// frustum-sector area that formula denotes. R0, R1 (the patch's own
// side/cap radii) and H (= capZ − sideZ) are payload float64s, hence exact
// rationals with NO rounding on the way, and ΔR is formed HERE as the exact
// rational R1−R0 rather than lifted from the caller's own float subtraction:
// fl(R1−R0) is a DIFFERENT number wherever the two radii are far enough apart
// to round, and an interval built on it encloses the area of a frustum whose
// slant is √(fl(R1−R0)²+H²) — a patch nobody holds and nobody denotes. Only
// the outward arm of the cap offset reaches that (a hole or concave arc, whose
// cap radius is R+d; the inward arm's R−d is Sterbenz-exact), and it starts
// rounding at an ordinary setback-to-radius ratio, so forming the difference
// inside is what keeps the interval a statement about the held patch.
//
// The remaining inexact factors are the sweep Δθ — bracketed by dthAllow,
// capThAllow's own proven enclosure of the true window (an atan2Interval
// bracket on the patch's own offset feet, or piLower/piUpper for the
// structurally whole-turn circle; capblend_contour.go/capblend_geom.go) — and
// the square root, rounded outward by intervalSqrt. Interval arithmetic is
// inclusion-monotonic, so the composed product [dth−dthAllow, dth+dthAllow] ×
// [R0+R1] × [slant] encloses the true A whatever the platform's Hypot, Atan2
// or Sincos returned: nowhere here is an ulp contract on any of them assumed.
// Returns +Inf where a factor fails to lift (non-finite geometry), so the
// caller's math.Min falls back to the unconditional envelope.
//
// "The true A" here is the true area of the frustum sector the HELD R0, R1
// and H describe, and lifting those three exactly is precisely what makes it
// so. It is not a claim about the patch the chamfer DENOTES: H is capZ minus
// a ROUNDED side level, and the distance between the two patches is the
// caller's own separate charge (bandLevelAreaAllow), which no tightening here
// can ever substitute for. That charge already covers the H subtraction's own
// rounding (patchAreaOf folds addRoundError(capZ, −sideZ, H) in beside the
// level's displacement), which is why H stays a parameter where ΔR does not.
func coneFrustumAreaBracket(R0, R1, H, dth, dthAllow, held float64) float64 {
	if isNonFinite(dthAllow) {
		return math.Inf(1)
	}
	rR0, rR1 := floatRat(R0), floatRat(R1)
	rH := floatRat(H)
	rdth, rAllow := floatRat(dth), floatRat(dthAllow)
	if rR0 == nil || rR1 == nil || rH == nil || rdth == nil || rAllow == nil {
		return math.Inf(1)
	}
	rdR := new(big.Rat).Sub(rR1, rR0)
	slantIv, ok := intervalSqrt(intervalAdd(intervalSquare(pointInterval(rdR)), intervalSquare(pointInterval(rH))))
	if !ok {
		return math.Inf(1)
	}
	radiiIv := pointInterval(new(big.Rat).Add(rR0, rR1))
	// [dth-dthAllow, dth+dthAllow], taken EXACTLY over rationals (never as a
	// float subtraction/addition, which could round the interval's own
	// endpoint the wrong way and silently exclude the true value it is
	// supposed to enclose).
	dthIv := interval(new(big.Rat).Sub(rdth, rAllow), new(big.Rat).Add(rdth, rAllow))
	areaIv := intervalScale(intervalMul(dthIv, intervalMul(radiiIv, slantIv)), big.NewRat(1, 2))
	return intervalFloatError(areaIv, held)
}

// capBlendBoundsContext is the placed body's axis-aligned bounding box, read
// along each world axis from the payload's OWN patch extrema
// (extentBoundedAlong, Table DX row DX5) exactly as prismBoundsContext reads a
// prism's — so every face of the box is a value the body attains. Min and Max
// are positions and Bound is the absolute error on them (measurement.go), so a
// box widened outward by the setback d would be a Min sitting d millimetres
// from the true extreme while claiming an error of zero; §8.4 asks for bounds
// from patch extrema for that reason, and capblend.go's extentAlong already
// states why padding the receiver prism by d proves nothing about attainment.
//
// The Bound is the reading's own, never a fixed zero. An extreme held by a
// recorded coordinate really does have none, which is the ordinary case on an
// unplaced body whose plane is axis aligned — a placement's own arithmetic can
// round it even so, and the reading charges that; an extreme held by the COMPUTED cap
// contour is known only to that contour's proven displacement, and publishing
// it as an Exact position would assert an accuracy the offset solve never had.
// extentBoundedAlong keeps the two apart per candidate, so a contour that loses
// the extremization contributes nothing here.
func capBlendBoundsContext(ctx context.Context, cbp capBlendPayload, work *freeformWork) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	bound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, axisBound, err := cbp.extentBoundedAlong(ctx, axis, work)
		if err != nil {
			return Box{}, err
		}
		minC[i], maxC[i] = lo, hi
		bound = math.Max(bound, axisBound)
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}
