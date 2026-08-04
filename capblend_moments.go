package decad

import (
	"context"
	"math"

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
// integral is reference-point independent). A flat Plane patch's flux is
// exact (the rational tetrahedron identity); a Cone patch's is a closed-form
// polynomial-plus-trig expression, bounded — never claimed Exact.

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
	var startArea, endArea, sideArea, patchArea, slabVolume boundedScalar
	var bandVolume float64
	patchGeoms := map[string]capPatchGeom{}
	collectPatchGeoms := func(band capBandResult) {
		for i, f := range band.patches {
			if len(f.origins) == 0 {
				continue
			}
			patchGeoms[f.origins[0].Role] = band.geom[i]
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

		zLo, zHi := cbp.z0, cbp.z1
		if onStart {
			zLo = cbp.z0 + cbp.d
		}
		if onEnd {
			zHi = cbp.z1 - cbp.d
		}
		ppFor := prismPayload{frame: cbp.frame, z0: zLo, z1: zHi, xform: cbp.xform}
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
		straightHeight := boundedSub(exactScalar(zHi), exactScalar(zLo))
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
			bandVolume += sign * v
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
			bandVolume += sign * v
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

	volume := boundedAdd(slabVolume, measuredScalar(bandVolume, capBlendVolumeBound(bandVolume)))
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
func capBandVolume(ctx context.Context, loop LoopRecord, cbp capBlendPayload, geom []capPatchGeom, capZ, matSign float64) (float64, error) {
	sideZ := capZ + matSign*cbp.d
	sideArea, err := loopEnclosedAreaContext(ctx, loop)
	if err != nil {
		return 0, err
	}
	capBoundary, err := capLoopBoundary(ctx, loop, cbp.d)
	if err != nil {
		return 0, err
	}
	capArea, err := loopEnclosedAreaContext(ctx, capBoundary)
	if err != nil {
		return 0, err
	}
	// Outward normal signs (docs/modify-reach-design.md §8.4): the disk at
	// capZ faces -matSign*Z, the disk at sideZ faces +matSign*Z, both away
	// from the band's own material. A flat disk's raw flux (P.N over the
	// disk) is its constant Z coordinate times its signed normal times its
	// area — no triangulation needed.
	fluxTotal := capZ*(-matSign)*capArea.value + sideZ*matSign*sideArea.value
	// patchRawFlux's own v0..v3 (or triangle-fan) vertex order is FIXED —
	// side-level vertices first, cap-level second — regardless of which cap
	// the band sits on. That fixed order is "CCW as seen from outside" for
	// one Z ordering of (sideZ, capZ) and its mirror for the other, exactly
	// the same start/end asymmetry capblend_geom.go's fixPatchOrientation
	// corrects for the SURFACE normal — so the flux sign needs the same
	// -matSign correction here, confirmed empirically
	// (TestCapBlendStartCapVolumeMatchesEndCap). That correction speaks for
	// the AXIAL half only; an apex patch's own radial inversion is charged
	// inside patchRawFlux.
	for _, g := range geom {
		fluxTotal += -matSign * patchRawFlux(g)
	}
	return fluxTotal / 3, nil
}

// patchRawFlux is one patch's contribution to the band's total raw flux
// (3x its own volume-by-divergence-theorem share), taken relative to the
// plane-local origin (0, 0, 0). A flat Plane patch's contribution is the
// exact rational tetrahedron identity (Flux_triangle = 3*tetraVolume =
// (1/2)*v0.(v1 x v2)); a Cone patch's is the closed-form
// polynomial-plus-trig integral over its linear-in-s ruled parametrization.
func patchRawFlux(g capPatchGeom) float64 {
	if !g.circular {
		v0 := r3.NewVec(g.sideA.U, g.sideA.V, g.sideZ)
		v1 := r3.NewVec(g.sideB.U, g.sideB.V, g.sideZ)
		v2 := r3.NewVec(g.capB.U, g.capB.V, g.capZ)
		v3 := r3.NewVec(g.capA.U, g.capA.V, g.capZ)
		tri := func(a, b, c r3.Vec) float64 { return a.Dot(b.Cross(c)) }
		return 0.5*tri(v0, v1, v2) + 0.5*tri(v0, v2, v3)
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
	if g.apex {
		// The fixed parameter order this integral is taken in orients a Cone
		// patch RADIALLY OUTWARD (its normal reads (H*cos, H*sin, -dR), so a
		// regular wall patch — whose offset SHRINKS the radius, dR < 0 — comes
		// out pointing away from the axis, which is that patch's own outward
		// sense; the caller's -matSign then fixes the axial half). An apex
		// patch inverts BOTH: its offset GROWS the radius from zero (dR > 0),
		// and the sector it cuts off is the void, so the solid lies radially
		// OUTSIDE the cone and the true outward normal points at the axis. The
		// caller's single -matSign speaks only for the axial ordering, so the
		// radial inversion is charged here — the same fact capblend_geom.go's
		// apex orientation reference states for the surface normal.
		return -flux
	}
	return flux
}

// capBlendVolumeBound is a conservative outward bound on the band-volume
// contribution's rounding: the flux formula mixes trig terms with polynomial
// ones, so it is never claimed Exact except the proven-zero case of no
// chamfered loop at all.
func capBlendVolumeBound(bandVolume float64) float64 {
	if bandVolume == 0 {
		return 0
	}
	return analyticRoundBound(math.Abs(bandVolume))
}

// patchAreaOf is one patch's own surface area: exact (a cross-product
// magnitude) for a Plane, closed form with a proven bound (a slant length
// involving a square root) for a Cone.
func patchAreaOf(g capPatchGeom) (float64, float64) {
	if !g.circular {
		v0 := r3.NewVec(g.sideA.U, g.sideA.V, g.sideZ)
		v1 := r3.NewVec(g.sideB.U, g.sideB.V, g.sideZ)
		v2 := r3.NewVec(g.capB.U, g.capB.V, g.capZ)
		v3 := r3.NewVec(g.capA.U, g.capA.V, g.capZ)
		a1 := v1.Sub(v0).Cross(v2.Sub(v0)).Len() / 2
		a2 := v2.Sub(v0).Cross(v3.Sub(v0)).Len() / 2
		return a1 + a2, 0
	}
	R0, R1 := g.sideRadius, g.capRadius
	H := g.capZ - g.sideZ
	dR := R1 - R0
	slant := math.Hypot(dR, H)
	dth := math.Abs(g.th1 - g.th0)
	area := dth / 2 * (R0 + R1) * slant
	return area, conservativeValueError(area, dth*(R0+R1)*(math.Abs(dR)+math.Abs(H)))
}

// capBlendBoundsContext is the placed body's axis-aligned bounding box: the
// receiver's own bounds widened by d along every axis — a chamfer point
// never lies farther than d beyond the original prism's boundary (a convex
// removal stays strictly inside it; a reflex corner's apex patch bulges by
// at most d) — a sound, if not razor-tight, envelope, evaluated directly for
// soundness rather than reused from a tighter but unproven claim.
func capBlendBoundsContext(ctx context.Context, cbp capBlendPayload, work *freeformWork) (Box, error) {
	pp := prismPayload{profile: cbp.profile, frame: cbp.frame, z0: cbp.z0, z1: cbp.z1, xform: cbp.xform}
	base, err := prismBoundsContext(ctx, pp, work)
	if err != nil {
		return Box{}, err
	}
	pad := r3.NewVec(cbp.d, cbp.d, cbp.d)
	return Box{
		Min:       base.Min.Sub(pad),
		Max:       base.Max.Add(pad),
		Exactness: Approximate,
		Bound:     units.Millimeters(0),
	}, nil
}

// capBlendCentroidEstimate is an area-weighted average of every face's own
// representative point, with the geometric safety-net bound every analytic
// centroid already falls back to: the true centroid lies within the returned
// Bounds box, so |estimate-true| is bounded by the box's own reach from the
// estimate — sound whatever the estimate's own accuracy.
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
	reach := 0.0
	for _, c := range []r3.Vec{bounds.Min, bounds.Max} {
		dd := c.Sub(estimate).Len()
		if dd > reach {
			reach = dd
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
