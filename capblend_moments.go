package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/cappatch"
	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"
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
// closed-form polynomial-plus-trig expression evaluated over exact rationals
// with every sine and cosine enclosed (conePatchFluxInterval), held at the
// enclosure's midpoint and never claimed Exact.
//
// The volume bound is composed term by term, and the reason is the band's own
// shape. Every one of these flux terms is a DIFFERENCE that cancels: the two
// closing disks each carry the whole prism's flux and differ by the band's,
// smaller by the ratio of the sweep height to the setback; a ruled-cone
// integral cancels likewise. A rounding budget scaled by the sum they
// cancelled to under-counts by exactly that ratio, so no bound here is ever
// read off a summed result — each is charged against the absolute terms the
// step acted on, and proofbound.BoundedAdd/proofbound.BoundedMul then sum bounds while the values
// cancel. The Plane arm escapes that composition entirely rather than manage
// it: an exact rational has nothing to cancel, and its committed rounding is
// measured (rationalFloatError), not budgeted. The Cone arm escapes it the
// same way: its closed form is an exact-rational interval whose trig factors
// are certified enclosures, so its bound is the interval's reach from the held
// midpoint (proofbound.IntervalFloatError). Only the whole-turn arm, which has no trig
// term to enclose, and the non-finite fallback, which has no rational to
// carry, still pass through math.Sincos; there the magnitude envelope
// internal/proofbound/bounded.go's proofbound.AnalyticRoundBound doc reserves for a libm result stands, never
// that helper's roundoff budget alone.

// evalCapBlendContext builds the analytic cap-blend body from the payload
// (BX3): the trimmed prism side walls (buildLoopSidesAs, unmodified) plus,
// for every chamfered loop, the chamfer band patches and the offset cap
// boundary; every other loop and every unchamfered cap keeps the ordinary
// prism construction. One shell, one lump, watertight by the same argument
// evalPrism's is (every edge bounds exactly two faces).
func evalCapBlendContext(ctx context.Context, d *Document, ref producerID, cbp capBlendPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	work := freeform.NewFreeformWork()
	loops := cbp.loops()
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true}

	var faces []*Face
	startLoopObjs := make([]*Loop, len(loops))
	endLoopObjs := make([]*Loop, len(loops))
	var startArea, endArea, sideArea, patchArea, slabVolume, bandVolume proofbound.BoundedScalar
	// muTotal, mvTotal, mzTotal accumulate the body's own plane-local first
	// moments (docs/modify-reach-design.md §8.4's fourth reading): the SAME
	// per-loop sign this loop already applies to slabVolume/bandVolume, over
	// the SAME two-part decomposition (a signed slab term via
	// loopEnclosedMomentsContext, a band term via capBandMoment).
	var muTotal, mvTotal, mzTotal proofbound.BoundedScalar
	// Appended in build order — loop index, then the chamfered cap, then each
	// band's own patch index — which IS Table BX row BX3's deterministic patch
	// order, the order the DX7 survey then reports its faces in.
	var patchGeoms []capPatch
	// bandDeltas carries each band's own contour displacement onto the payload,
	// keyed by the (loop, cap) it was built for. The tessellator reads it so its
	// cap-level facets charge the SAME displacement this build's own vertices,
	// edges and areas already did.
	bandDeltas := map[capBandKey]float64{}
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
		zLo, zHi := proofbound.MeasuredScalar(cbp.z0, cbp.z0Delta), proofbound.MeasuredScalar(cbp.z1, cbp.z1Delta)
		setback := proofbound.MeasuredScalar(cbp.d, cbp.dDelta)
		if onStart {
			zLo = proofbound.BoundedAdd(zLo, setback)
		}
		if onEnd {
			zHi = proofbound.BoundedSub(zHi, setback)
		}
		// The side walls are built over those same two bounded levels, so each
		// end's displacement goes in with it: the wall vertices, the vertical
		// edge lengths and the side face areas all read a level the setback
		// computed, never one they can claim was recorded.
		ppFor := prismPayload{
			frame:   cbp.frame,
			z0:      zLo.Value,
			z1:      zHi.Value,
			z0Delta: zLo.Bound,
			z1Delta: zHi.Bound,
			xform:   cbp.xform,
		}
		sideFaces, bottomCo, topCo, loopLen, err := buildLoopSidesAs(ctx, body, ref, ppFor, li, li != 0, loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		for _, f := range sideFaces {
			sideArea = proofbound.BoundedAdd(sideArea, proofbound.MeasuredScalar(f.area, f.areaBound))
		}
		_ = loopLen

		loopArea, loopMu, loopMv, err := loopEnclosedMomentsContext(ctx, loop)
		if err != nil {
			return nil, err
		}
		straightHeight := proofbound.BoundedSub(zHi, zLo)
		loopSlab := proofbound.BoundedMul(loopArea, straightHeight)
		slabVolume = proofbound.BoundedAdd(slabVolume, proofbound.MeasuredScalar(sign*loopSlab.Value, loopSlab.Bound))

		// M_slab = (mu·h, mv·h, A·h·(zLo+zHi)/2), docs/modify-reach-design.md
		// §8.4(a): mu, mv are the loop's own signed first moments (already
		// canonicalized the same way loopArea is), and A·h is loopSlab itself,
		// so the z component reuses it rather than recomputing A·h a second
		// time.
		zMid := proofbound.BoundedDiv(proofbound.BoundedAdd(zLo, zHi), proofbound.ExactScalar(2))
		loopMuSlab := proofbound.BoundedMul(loopMu, straightHeight)
		loopMvSlab := proofbound.BoundedMul(loopMv, straightHeight)
		loopMzSlab := proofbound.BoundedMul(loopSlab, zMid)
		muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*loopMuSlab.Value, loopMuSlab.Bound))
		mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*loopMvSlab.Value, loopMvSlab.Bound))
		mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*loopMzSlab.Value, loopMzSlab.Bound))

		startCo, endCo := bottomCo, topCo

		// startBand/endBand default to the zero capBandResult (delta 0, capCo
		// nil) where the loop is not chamfered on that cap, which is what
		// leaves the area correction below at its own zero — the delta<=0
		// guards in proofbound.SectionDisplacementArea and proofbound.BandPatchAreaAllow, not a
		// separate onStart/onEnd branch.
		var startBand, endBand capBandResult
		if onStart {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z0, +1, bottomCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			bandDeltas[capBandKey{loop: li, start: true}] = band.delta
			startCo = band.capCo
			startBand = band
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z0, +1, band.delta)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = proofbound.BoundedAdd(patchArea, proofbound.MeasuredScalar(pa, pb))
			}
			bmu, bmv, bmz, err := capBandMoment(ctx, loop, cbp, band.geom, cbp.z0, +1, band.delta, work)
			if err != nil {
				return nil, err
			}
			muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*bmu.Value, bmu.Bound))
			mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*bmv.Value, bmv.Bound))
			mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*bmz.Value, bmz.Bound))
		}
		if onEnd {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z1, -1, topCo, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			bandDeltas[capBandKey{loop: li, start: false}] = band.delta
			endCo = band.capCo
			endBand = band
			v, err := capBandVolume(ctx, loop, cbp, band.geom, cbp.z1, -1, band.delta)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.geom {
				pa, pb := patchAreaOf(g)
				patchArea = proofbound.BoundedAdd(patchArea, proofbound.MeasuredScalar(pa, pb))
			}
			bmu, bmv, bmz, err := capBandMoment(ctx, loop, cbp, band.geom, cbp.z1, -1, band.delta, work)
			if err != nil {
				return nil, err
			}
			muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*bmu.Value, bmu.Bound))
			mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*bmv.Value, bmv.Bound))
			mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*bmz.Value, bmz.Bound))
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
		// sits from the one the offset DENOTES. proofbound.SectionDisplacementArea is the
		// same 2D set-displacement identity extrude.go's own region area
		// composes (docs/prism-boolean-design.md §7, one dimension down from
		// proofbound.SweptVolumeAllow above): a set bound, sound even where the
		// displacement changes which regions the offset merged, charged
		// against this loop's own held boundary via capContourPerimeterUpper
		// so it stands whatever the corner geometry did.
		sBound := proofbound.AbsSumUpper(sArea.Bound,
			proofbound.SectionDisplacementArea(startBand.delta, len(loop.Segments), capContourPerimeterUpper(startBand.capCo)))
		startArea = proofbound.BoundedAdd(startArea, proofbound.MeasuredScalar(sign*sArea.Value, sBound))

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
		eBound := proofbound.AbsSumUpper(eArea.Bound,
			proofbound.SectionDisplacementArea(endBand.delta, len(loop.Segments), capContourPerimeterUpper(endBand.capCo)))
		endArea = proofbound.BoundedAdd(endArea, proofbound.MeasuredScalar(sign*eArea.Value, eBound))
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
		origins:       []FeatureRef{{producer: ref, Role: roleCapStart}},
		body:          body,
		loops:         startLoopObjs,
		area:          startArea.Value,
		areaBound:     startArea.Bound,
		axialDelta:    cbp.z0Delta,
		hasAxialDelta: true,
	}
	capEnd := &Face{
		surface:       Plane{Frame: endFrame},
		origins:       []FeatureRef{{producer: ref, Role: roleCapEnd}},
		body:          body,
		loops:         endLoopObjs,
		area:          endArea.Value,
		areaBound:     endArea.Bound,
		axialDelta:    cbp.z1Delta,
		hasAxialDelta: true,
	}
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	volume := proofbound.BoundedAdd(slabVolume, bandVolume)
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: exactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}

	totalArea := proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedAdd(startArea, endArea), sideArea), patchArea)
	body.area = Measurement{
		Value:     units.SquareMillimeters(totalArea.Value),
		Exactness: exactnessOf(totalArea.Bound),
		Bound:     units.SquareMillimeters(totalArea.Bound),
	}

	// Centroid: the closed-form first moments divided by the body's own
	// volume (docs/modify-reach-design.md §8.4), lifted to world through the
	// SAME frame/placement lift a prism centroid uses, backed by the
	// geometry safety-net ceiling every analytic centroid falls back to.
	bounds, err := capBlendBoundsContext(ctx, cbp, work)
	if err != nil {
		return nil, err
	}
	cu := proofbound.BoundedQuotient(muTotal.Value, muTotal.Bound, volume.Value, volume.Bound)
	cv := proofbound.BoundedQuotient(mvTotal.Value, mvTotal.Bound, volume.Value, volume.Bound)
	cz := proofbound.BoundedQuotient(mzTotal.Value, mzTotal.Bound, volume.Value, volume.Bound)
	centroidValue := pl.point(cu.Value, cv.Value, cz.Value)
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
	cbp.bandDelta = bandDeltas
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
// on this cap) sums to zero, which is what leaves proofbound.SectionDisplacementArea's
// own delta<=0 guard as the only gate that matters.
func capContourPerimeterUpper(capCo []coedge) float64 {
	total := proofbound.BoundedScalar{}
	for _, ce := range capCo {
		total = proofbound.BoundedAdd(total, proofbound.MeasuredScalar(ce.edge.length, ce.edge.lengthBound))
	}
	return proofbound.AbsSumUpper(total.Value, total.Bound)
}

// capLoopBoundary returns loop_li's OWN offset-by-d boundary as a standalone
// LoopRecord, used to compute a chamfered cap's per-loop enclosed area and
// the band's closing disk at the cap level.
func capLoopBoundary(ctx context.Context, loop LoopRecord, d float64) (LoopRecord, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
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
//     proofbound.BoundedMul/proofbound.BoundedAdd take EXACTLY over big.Rat rather than estimate;
//   - each patch's own flux bound (patchRawFlux), including a Cone patch's
//     trig terms, which a certified enclosure bounds (a magnitude envelope
//     only where no enclosure lifts) and proofbound.AnalyticRoundBound never speaks for.
//
// proofbound.BoundedAdd sums bounds, so no step of this composition is ever rescaled by
// a result the step's own operands cancelled down to.
//
// None of those terms speaks for the cap contour's own displacement (delta,
// capblend_contour.go): capArea and every patchRawFlux term above read the
// contour's coordinates as exact inputs, but the contour is a COMPUTED offset
// that sits within delta of the one it denotes. That is one displacement
// acting on the whole surface the band closes on — this loop's patches AND
// the cap disk they meet — so it is composed ONCE here, after the flux sum,
// via internal/proofbound/bounds.go's proofbound.SweptVolumeAllow(delta, areaUpper): charging it inside
// capArea's own bound, or inside each patchRawFlux term, would count the SAME
// displaced coordinates twice, since patchRawFlux already reads them.
func capBandVolume(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign, delta float64) (proofbound.BoundedScalar, error) {
	capZB := cbp.capBandLevel(capZ, matSign)
	sideZB := proofbound.BoundedAdd(capZB, proofbound.MeasuredScalar(matSign*cbp.d, cbp.dDelta))
	sideZ := sideZB.Value
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
	signedArea, err := loopSignedAreaBudget(proofbound.NewWorkBudget(ctx), loop)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}
	sideArea, err := loopEnclosedAreaContext(ctx, loop)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	capBoundary, err := capLoopBoundary(ctx, loop, cbp.d)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	capArea, err := loopEnclosedAreaContext(ctx, capBoundary)
	if err != nil {
		return proofbound.BoundedScalar{}, err
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
	fluxTotal := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(capZB.Value*(-matSign), capZB.Bound), capArea),
		proofbound.BoundedMul(proofbound.MeasuredScalar(sideZ*matSign, sideZB.Bound), sideArea),
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
	patchAreaTotal := proofbound.BoundedScalar{}
	for _, g := range geom {
		f := patchRawFlux(g)
		fluxTotal = proofbound.BoundedAdd(fluxTotal, proofbound.MeasuredScalar(-matSign*orient*f.Value, f.Bound))
		pa, pb := patchAreaOf(g)
		patchAreaTotal = proofbound.BoundedAdd(patchAreaTotal, proofbound.MeasuredScalar(pa, pb))
	}
	result := proofbound.BoundedQuotient(fluxTotal.Value, fluxTotal.Bound, 3, 0)
	// areaUpper is the surface the contour's own displacement acted on: this
	// band's patches plus the cap disk they close on (capArea) — the same two
	// terms patchRawFlux and the disk flux above both read displaced
	// coordinates from.
	areaUpper := proofbound.AbsSumUpper(patchAreaTotal.Value, patchAreaTotal.Bound, capArea.Value, capArea.Bound)
	result.Bound = proofbound.AbsSumUpper(result.Bound, proofbound.SweptVolumeAllow(delta, areaUpper))
	return result, nil
}

func patchRawFlux(g capPatchGeom) proofbound.BoundedScalar { return cappatch.RawFlux(g.patch()) }

func patchAreaOf(g capPatchGeom) (float64, float64) { return cappatch.AreaOf(g.patch()) }

func patchDisplacementAreaAllow(g capPatchGeom) float64 {
	return cappatch.DisplacementAreaAllow(g.patch())
}

func capWindowOnBranch(capTh0, capTh1, th0 float64) (float64, float64) {
	return cappatch.WindowOnBranch(capTh0, capTh1, th0)
}

func ruledAngleCos(thS0, thS1, thC0, thC1 float64) float64 {
	return cappatch.RuledAngleCos(thS0, thS1, thC0, thC1)
}

func conePatchFluxInterval(g capPatchGeom) (proofbound.RatInterval, bool) {
	return cappatch.ConeFluxInterval(g.patch())
}

func coneFrustumAreaBracket(R0, R1, H, dth, dthAllow, held float64) float64 {
	return cappatch.FrustumAreaBracket(R0, R1, H, dth, dthAllow, held)
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
func capBlendBoundsContext(ctx context.Context, cbp capBlendPayload, work *freeform.FreeformWork) (Box, error) {
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
