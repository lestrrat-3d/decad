package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file assembles the cap-blend body and adapts its mass readings to
// internal/capband. See docs/modify-reach-design.md §8.4.

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
	body := bornBody(&Body{doc: d, n: topo.Body{Solid: true, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}}})

	var faces []*Face
	startLoopObjs := make([]*Loop, len(loops))
	endLoopObjs := make([]*Loop, len(loops))
	var startArea, endArea, sideArea, patchArea, slabVolume, bandVolume proofbound.BoundedScalar
	// muTotal, mvTotal, mzTotal accumulate the body's own plane-local first
	// moments (docs/modify-reach-design.md §8.4's fourth reading): the SAME
	// per-loop sign this loop already applies to slabVolume/bandVolume, over
	// the SAME two-part decomposition (a signed slab term via
	// momentinput.LoopEnclosedMoments, a band term via capband.BandMoment).
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
			if len(f.n.Origins) == 0 {
				continue
			}
			patchGeoms = append(patchGeoms, capPatch{role: f.n.Origins[0].Role, geom: band.geom[i]})
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

		// A chamfered end pulls its own straight level in by that cap's side
		// setback ds (docs/modify-reach-design.md §8.3.1). Its unit conversion
		// and float sum both round. The rounding is an ulp of
		// the SWEEP, but it multiplies the whole section area below, so it
		// reaches the volume at the scale of the band itself and is charged here
		// — the same term capband.BandVolume charges for the identical level it reads
		// as sideZ.
		zLo, zHi := proofbound.MeasuredScalar(cbp.z0, cbp.z0Delta), proofbound.MeasuredScalar(cbp.z1, cbp.z1Delta)
		if onStart {
			zLo = proofbound.BoundedAdd(zLo, proofbound.MeasuredScalar(cbp.start.ds, cbp.start.dsDelta))
		}
		if onEnd {
			zHi = proofbound.BoundedSub(zHi, proofbound.MeasuredScalar(cbp.end.ds, cbp.end.dsDelta))
		}
		// The side walls are built over those same two bounded levels, so each
		// end's displacement goes in with it: the wall vertices, the vertical
		// edge lengths and the side face areas all read a level the setback
		// computed, never one they can claim was recorded.
		ppFor := prismPayload{Payload: prismrecord.Payload{
			Frame:   cbp.frame,
			Z0:      zLo.Value,
			Z1:      zHi.Value,
			Z0Delta: zLo.Bound,
			Z1Delta: zHi.Bound,
			Xform:   cbp.xform,
		}}
		sideFaces, bottomCo, topCo, loopLen, err := buildLoopSidesAs(ctx, body, ref, ppFor, li, li != 0, loop, work, nil, levelToken{}, levelToken{}, false)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		for _, f := range sideFaces {
			sideArea = proofbound.BoundedAdd(sideArea, proofbound.MeasuredScalar(f.n.Area, f.n.AreaBound))
		}
		_ = loopLen

		loopArea, loopMu, loopMv, err := momentinput.LoopEnclosedMoments(ctx, loop)
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
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z0, +1, bottomCo, nil, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			bandDeltas[capBandKey{loop: li, start: true}] = band.delta
			startCo = band.capCo
			startBand = band
			mass, err := readBandMassWithSideArea(ctx, li, loop, cbp, band.geom, cbp.z0, +1,
				band.delta, band.closure, &loopArea)
			if err != nil {
				return nil, err
			}
			v, err := capband.BandVolume(mass, work)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.geom {
				pa, pb := capband.AreaOf(g)
				patchArea = proofbound.BoundedAdd(patchArea, proofbound.MeasuredScalar(pa, pb))
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			bmu, bmv, bmz, err := capband.BandMoment(mass, work)
			if err != nil {
				return nil, err
			}
			muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*bmu.Value, bmu.Bound))
			mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*bmv.Value, bmv.Bound))
			mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*bmz.Value, bmz.Bound))
		}
		if onEnd {
			band, err := buildCapBand(ctx, body, ref, cbp, li, loop, cbp.z1, -1, topCo, nil, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.patches...)
			collectPatchGeoms(band)
			bandDeltas[capBandKey{loop: li, start: false}] = band.delta
			endCo = band.capCo
			endBand = band
			mass, err := readBandMassWithSideArea(ctx, li, loop, cbp, band.geom, cbp.z1, -1,
				band.delta, band.closure, &loopArea)
			if err != nil {
				return nil, err
			}
			v, err := capband.BandVolume(mass, work)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.geom {
				pa, pb := capband.AreaOf(g)
				patchArea = proofbound.BoundedAdd(patchArea, proofbound.MeasuredScalar(pa, pb))
			}
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			bmu, bmv, bmz, err := capband.BandMoment(mass, work)
			if err != nil {
				return nil, err
			}
			muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*bmu.Value, bmu.Bound))
			mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*bmv.Value, bmv.Bound))
			mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*bmz.Value, bmz.Bound))
		}

		startLoopObjs[li] = bornLoop(&Loop{n: topo.Loop{Outer: li == 0, CoEdges: startCo}})
		endLoopObjs[li] = bornLoop(&Loop{n: topo.Loop{Outer: li == 0, CoEdges: endCo}})

		startBoundary := loop
		if onStart {
			startBoundary, err = capband.LoopBoundary(ctx, loop, cbp.start.dc, shellTol)
			if err != nil {
				return nil, err
			}
		}
		sArea, err := momentinput.LoopEnclosedArea(ctx, startBoundary)
		if err != nil {
			return nil, err
		}
		// sArea is measured on the BUILT cap contour (momentinput.LoopEnclosedArea's
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
			endBoundary, err = capband.LoopBoundary(ctx, loop, cbp.end.dc, shellTol)
			if err != nil {
				return nil, err
			}
		}
		eArea, err := momentinput.LoopEnclosedArea(ctx, endBoundary)
		if err != nil {
			return nil, err
		}
		eBound := proofbound.AbsSumUpper(eArea.Bound,
			proofbound.SectionDisplacementArea(endBand.delta, len(loop.Segments), capContourPerimeterUpper(endBand.capCo)))
		endArea = proofbound.BoundedAdd(endArea, proofbound.MeasuredScalar(sign*eArea.Value, eBound))
	}

	pl := prismPayload{Payload: prismrecord.Payload{Frame: cbp.frame, Xform: cbp.xform}}
	startFrame, err := pl.CapFrame(cbp.z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := pl.CapFrame(cbp.z1, false)
	if err != nil {
		return nil, err
	}
	capStart := bornFace(&Face{
		n: topo.Face{
			Area:          startArea.Value,
			AreaBound:     startArea.Bound,
			AxialDelta:    cbp.z0Delta,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: startFrame},
			CapRecord:     &topo.CapRecord{Frame: cbp.frame, Xform: cbp.xform, Z: cbp.z0},
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
			Body:          bodyNode(body),
			Loops:         loopNodes(startLoopObjs),
		},
	})
	capEnd := bornFace(&Face{
		n: topo.Face{
			Area:          endArea.Value,
			AreaBound:     endArea.Bound,
			AxialDelta:    cbp.z1Delta,
			HasAxialDelta: true,
			Surface:       surfacegeom.Plane{Frame: endFrame},
			CapRecord:     &topo.CapRecord{Frame: cbp.frame, Xform: cbp.xform, Z: cbp.z1},
			Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
			Body:          bodyNode(body),
			Loops:         loopNodes(endLoopObjs),
		},
	})
	if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	body.lumps = []*Lump{{shells: []*FaceShell{{faces: faces}}}}

	volume := proofbound.BoundedAdd(slabVolume, bandVolume)
	body.n.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: measurement.ExactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}

	totalArea := proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedAdd(startArea, endArea), sideArea), patchArea)
	body.n.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(totalArea.Value),
		Exactness: measurement.ExactnessOf(totalArea.Bound),
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
	centroidValue := pl.Point(cu.Value, cv.Value, cz.Value)
	formulaBound := pl.PointBound(cu, cv, cz)
	geometryBound := capband.CentroidGeometryBound(centroidValue, boxToInternal(bounds))
	centroidBound := math.Min(formulaBound, geometryBound)
	body.n.Centroid = measurement.VecMeasurement{
		Value:     centroidValue,
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.n.Bounds = boxToInternal(bounds)
	if err := chargePrismMap(body, cbp.frame, cbp.xform); err != nil {
		return nil, err
	}
	if cbp.loftSource != nil {
		if err := widenLoftCapBandMeasurements(body, cbp.loftSource.proof); err != nil {
			return nil, err
		}
	}
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
// from the SAME offset math momentinput.LoopEnclosedArea(capband.LoopBoundary(...))
// measures the area of, so the two readings of one contour never disagree
// about which boundary they describe. A nil capCo (the loop is not chamfered
// on this cap) sums to zero, which is what leaves proofbound.SectionDisplacementArea's
// own delta<=0 guard as the only gate that matters.
func capContourPerimeterUpper(capCo []topo.CoEdge) float64 {
	total := proofbound.BoundedScalar{}
	for _, ce := range capCo {
		total = proofbound.BoundedAdd(total, proofbound.MeasuredScalar(ce.Edge.Length, ce.Edge.LengthBound))
	}
	return proofbound.AbsSumUpper(total.Value, total.Bound)
}

// contourOf is capband.LoopBoundary read under cbp's corner rule for loop li: a
// draft view's far section is the loop's sharp offset
// (offset2d.BuildSharpLoop), every corner mitered and each walk moved its own
// amount (walkAmounts), where a chamfer's cap contour closes a reflex corner
// with an arc.
func (cbp capBlendPayload) contourOf(ctx context.Context, li int, loop loopRecord, d float64) (loopRecord, error) {
	if !cbp.draft {
		return capband.LoopBoundary(ctx, loop, d, shellTol)
	}
	budget := proofbound.NewWorkBudget(ctx)
	cl, err := oneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	if err != nil {
		return loopRecord{}, err
	}
	segs, _, err := offset2d.BuildSharpLoop(budget, cl.Walks, cbp.walkAmounts(li, cl.Walks, d), shellTol)
	if err != nil {
		return loopRecord{}, offset2d.DraftOffsetError(err)
	}
	return loopRecord{Segments: segs}, nil
}

// readBandMass adapts the payload and built band to the recorded readings both
// mass integrals consume. The root package owns the offset contour and the
// authenticated loop areas; internal/capband owns their flux composition.
func readBandMass(ctx context.Context, li int, loop loopRecord, cbp capBlendPayload,
	geom []capPatchGeom, capZ, matSign, delta float64, closure capBandClosure) (capband.BandMassInput, error) {
	return readBandMassWithSideArea(ctx, li, loop, cbp, geom, capZ, matSign, delta, closure, nil)
}

// readBandMassWithSideArea reuses the area already computed with a loop's slab
// moments. Other callers pass nil and read the area from the loop as before.
func readBandMassWithSideArea(ctx context.Context, li int, loop loopRecord, cbp capBlendPayload,
	geom []capPatchGeom, capZ, matSign, delta float64, closure capBandClosure,
	knownSideArea *proofbound.BoundedScalar) (capband.BandMassInput, error) {
	setback := cbp.setbackAt(matSign)
	capZB := cbp.capBandLevel(capZ, matSign)
	sideZB := proofbound.BoundedAdd(capZB,
		proofbound.MeasuredScalar(matSign*setback.ds, setback.dsDelta))
	signedArea, err := sectionaudit.LoopSignedArea(proofbound.NewWorkBudget(ctx), loop)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	orient := 1.0
	if signedArea < 0 {
		orient = -1
	}
	var sideArea proofbound.BoundedScalar
	if knownSideArea != nil {
		if err := ctx.Err(); err != nil {
			return capband.BandMassInput{}, err
		}
		sideArea = *knownSideArea
	} else {
		sideArea, err = momentinput.LoopEnclosedArea(ctx, loop)
		if err != nil {
			return capband.BandMassInput{}, err
		}
	}
	capBoundary, err := cbp.contourOf(ctx, li, loop, setback.dc)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	capArea, err := momentinput.LoopEnclosedArea(ctx, capBoundary)
	if err != nil {
		return capband.BandMassInput{}, err
	}
	return capband.BandMassInput{
		Loop: loop, CapBoundary: capBoundary, Patches: geom,
		CapLevel: capZB, SideLevel: sideZB, SideArea: sideArea, CapArea: capArea,
		MaterialSign: matSign, Orientation: orient, Delta: delta,
		LevelDelta: capband.BandLevelDelta(capZ, matSign, setback.ds, setback.dsDelta), Closure: closure,
	}, nil
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
