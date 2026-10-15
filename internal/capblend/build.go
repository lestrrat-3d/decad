package capblend

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// This file assembles the cap-blend body and adapts its mass readings to
// internal/capband. See docs/modify-reach-design.md §8.4.

// Build builds the analytic cap-blend body from the payload
// (BX3): the trimmed prism side walls (prismrecord.BuildLoopSidesAs, unmodified) plus,
// for every chamfered loop, the chamfer band patches and the offset cap
// boundary; every other loop and every unchamfered cap keeps the ordinary
// prism construction. One shell, one lump, watertight by the same argument
// evalPrism's is (every edge bounds exactly two faces).
func Build(ctx context.Context, env topo.Env, cbp Payload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := env.Ref
	work := freeform.NewFreeformWork()
	loops := cbp.Loops()
	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: ref, Role: provenance.RoleBody}
	body.Doc = env.Doc

	var faces []*topo.Face
	startLoopObjs := make([]*topo.Loop, len(loops))
	endLoopObjs := make([]*topo.Loop, len(loops))
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
	var patchGeoms []Patch
	// bandDeltas carries each band's own contour displacement onto the payload,
	// keyed by the (loop, cap) it was built for. The tessellator reads it so its
	// cap-level facets charge the SAME displacement this build's own vertices,
	// edges and areas already did.
	bandDeltas := map[BandKey]float64{}
	collectPatchGeoms := func(band BandResult) {
		for i, f := range band.Patches {
			if len(f.Origins) == 0 {
				continue
			}
			patchGeoms = append(patchGeoms, Patch{Role: f.Origins[0].Role, Geom: band.Geom[i]})
		}
	}

	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		onStart, onEnd := cbp.StartLoops[li], cbp.EndLoops[li]
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
		zLo, zHi := proofbound.MeasuredScalar(cbp.Z0, cbp.Z0Delta), proofbound.MeasuredScalar(cbp.Z1, cbp.Z1Delta)
		if onStart {
			zLo = proofbound.BoundedAdd(zLo, proofbound.MeasuredScalar(cbp.Start.Ds, cbp.Start.DsDelta))
		}
		if onEnd {
			zHi = proofbound.BoundedSub(zHi, proofbound.MeasuredScalar(cbp.End.Ds, cbp.End.DsDelta))
		}
		// The side walls are built over those same two bounded levels, so each
		// end's displacement goes in with it: the wall vertices, the vertical
		// edge lengths and the side face areas all read a level the setback
		// computed, never one they can claim was recorded.
		ppFor := prismrecord.Payload{
			Frame:   cbp.Frame,
			Z0:      zLo.Value,
			Z1:      zHi.Value,
			Z0Delta: zLo.Bound,
			Z1Delta: zHi.Bound,
			Xform:   cbp.Xform,
		}
		sideFaces, bottomCo, topCo, loopLen, err := prismrecord.BuildLoopSidesAs(ctx, env, body, ppFor, li, li != 0, loop, work, nil, denotation.LevelToken{}, denotation.LevelToken{}, false)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sideFaces...)
		for _, f := range sideFaces {
			sideArea = proofbound.BoundedAdd(sideArea, proofbound.MeasuredScalar(f.Area, f.AreaBound))
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

		// startBand/endBand default to the zero BandResult (delta 0, capCo
		// nil) where the loop is not chamfered on that cap, which is what
		// leaves the area correction below at its own zero — the delta<=0
		// guards in proofbound.SectionDisplacementArea and proofbound.BandPatchAreaAllow, not a
		// separate onStart/onEnd branch.
		var startBand, endBand BandResult
		if onStart {
			band, err := BuildBand(ctx, env, body, cbp, li, loop, cbp.Z0, +1, bottomCo, nil, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.Patches...)
			collectPatchGeoms(band)
			bandDeltas[BandKey{Loop: li, Start: true}] = band.Delta
			startCo = band.CapCo
			startBand = band
			mass, err := readBandMassWithSideArea(ctx, li, loop, cbp, band.Geom, cbp.Z0, +1,
				band.Delta, band.Closure, &loopArea)
			if err != nil {
				return nil, err
			}
			v, err := capband.BandVolume(mass, work)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.Geom {
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
			band, err := BuildBand(ctx, env, body, cbp, li, loop, cbp.Z1, -1, topCo, nil, work)
			if err != nil {
				return nil, err
			}
			faces = append(faces, band.Patches...)
			collectPatchGeoms(band)
			bandDeltas[BandKey{Loop: li, Start: false}] = band.Delta
			endCo = band.CapCo
			endBand = band
			mass, err := readBandMassWithSideArea(ctx, li, loop, cbp, band.Geom, cbp.Z1, -1,
				band.Delta, band.Closure, &loopArea)
			if err != nil {
				return nil, err
			}
			v, err := capband.BandVolume(mass, work)
			if err != nil {
				return nil, err
			}
			bandVolume = proofbound.BoundedAdd(bandVolume, proofbound.MeasuredScalar(sign*v.Value, v.Bound))
			for _, g := range band.Geom {
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

		startLoopObjs[li] = topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: startCo})
		endLoopObjs[li] = topobuild.NewLoop(env.Alloc, topo.Loop{Outer: li == 0, CoEdges: endCo})

		startBoundary := loop
		if onStart {
			startBoundary, err = capband.LoopBoundary(ctx, loop, cbp.Start.Dc, offset2d.ShellTol)
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
		// against this loop's own held boundary via ContourPerimeterUpper
		// so it stands whatever the corner geometry did.
		sBound := proofbound.AbsSumUpper(sArea.Bound,
			proofbound.SectionDisplacementArea(startBand.Delta, len(loop.Segments), ContourPerimeterUpper(startBand.CapCo)))
		startArea = proofbound.BoundedAdd(startArea, proofbound.MeasuredScalar(sign*sArea.Value, sBound))

		endBoundary := loop
		if onEnd {
			endBoundary, err = capband.LoopBoundary(ctx, loop, cbp.End.Dc, offset2d.ShellTol)
			if err != nil {
				return nil, err
			}
		}
		eArea, err := momentinput.LoopEnclosedArea(ctx, endBoundary)
		if err != nil {
			return nil, err
		}
		eBound := proofbound.AbsSumUpper(eArea.Bound,
			proofbound.SectionDisplacementArea(endBand.Delta, len(loop.Segments), ContourPerimeterUpper(endBand.CapCo)))
		endArea = proofbound.BoundedAdd(endArea, proofbound.MeasuredScalar(sign*eArea.Value, eBound))
	}

	pl := prismrecord.Payload{Frame: cbp.Frame, Xform: cbp.Xform}
	startFrame, err := pl.CapFrame(cbp.Z0, true)
	if err != nil {
		return nil, err
	}
	endFrame, err := pl.CapFrame(cbp.Z1, false)
	if err != nil {
		return nil, err
	}
	capStart := topobuild.NewFace(env.Alloc, topo.Face{
		Area:          startArea.Value,
		AreaBound:     startArea.Bound,
		AxialDelta:    cbp.Z0Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: startFrame},
		CapRecord:     &topo.CapRecord{Frame: cbp.Frame, Xform: cbp.Xform, Z: cbp.Z0},
		Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
		Body:          body,
		Loops:         startLoopObjs,
	})
	capEnd := topobuild.NewFace(env.Alloc, topo.Face{
		Area:          endArea.Value,
		AreaBound:     endArea.Bound,
		AxialDelta:    cbp.Z1Delta,
		HasAxialDelta: true,
		Surface:       surfacegeom.Plane{Frame: endFrame},
		CapRecord:     &topo.CapRecord{Frame: cbp.Frame, Xform: cbp.Xform, Z: cbp.Z1},
		Origins:       []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
		Body:          body,
		Loops:         endLoopObjs,
	})
	if err := topobuild.AttachFaceLoops(ctx, []*topo.Face{capStart, capEnd}); err != nil {
		return nil, err
	}
	faces = append(faces, capStart, capEnd)
	shell := env.Alloc.NewFaceShell()
	shell.Faces = faces
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	volume := proofbound.BoundedAdd(slabVolume, bandVolume)
	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: measurement.ExactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}

	totalArea := proofbound.BoundedAdd(proofbound.BoundedAdd(proofbound.BoundedAdd(startArea, endArea), sideArea), patchArea)
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(totalArea.Value),
		Exactness: measurement.ExactnessOf(totalArea.Bound),
		Bound:     units.SquareMillimeters(totalArea.Bound),
	}

	// Centroid: the closed-form first moments divided by the body's own
	// volume (docs/modify-reach-design.md §8.4), lifted to world through the
	// SAME frame/placement lift a prism centroid uses, backed by the
	// geometry safety-net ceiling every analytic centroid falls back to.
	bounds, err := Bounds(ctx, cbp, work)
	if err != nil {
		return nil, err
	}
	cu := proofbound.BoundedQuotient(muTotal.Value, muTotal.Bound, volume.Value, volume.Bound)
	cv := proofbound.BoundedQuotient(mvTotal.Value, mvTotal.Bound, volume.Value, volume.Bound)
	cz := proofbound.BoundedQuotient(mzTotal.Value, mzTotal.Bound, volume.Value, volume.Bound)
	centroidValue := pl.Point(cu.Value, cv.Value, cz.Value)
	formulaBound := pl.PointBound(cu, cv, cz)
	geometryBound := capband.CentroidGeometryBound(centroidValue, bounds)
	centroidBound := math.Min(formulaBound, geometryBound)
	body.Centroid = measurement.VecMeasurement{
		Value:     centroidValue,
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}
	body.Bounds = bounds
	if err := prismrecord.ChargeMap(body, cbp.Frame, cbp.Xform); err != nil {
		return nil, err
	}
	if cbp.LoftSource != nil {
		if err := widenLoftMeasurements(body, cbp.LoftSource.Proof); err != nil {
			return nil, err
		}
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	cbp.Patches = patchGeoms
	cbp.BandDelta = bandDeltas
	body.Payload = cbp
	return body, nil
}

// ContourPerimeterUpper is a proven upper bound on the total length of a
// chamfer band's own cap-level boundary, read off the coedges BuildBand
// already built for it (band.capCo) rather than re-walked: each edge on that
// boundary already carries its own held length and proven bound (a wall's
// straight or circular capEdge, a reflex corner's connector arc), computed
// from the SAME offset math momentinput.LoopEnclosedArea(capband.LoopBoundary(...))
// measures the area of, so the two readings of one contour never disagree
// about which boundary they describe. A nil capCo (the loop is not chamfered
// on this cap) sums to zero, which is what leaves proofbound.SectionDisplacementArea's
// own delta<=0 guard as the only gate that matters.
func ContourPerimeterUpper(capCo []topo.CoEdge) float64 {
	total := proofbound.BoundedScalar{}
	for _, ce := range capCo {
		total = proofbound.BoundedAdd(total, proofbound.MeasuredScalar(ce.Edge.Length, ce.Edge.LengthBound))
	}
	return proofbound.AbsSumUpper(total.Value, total.Bound)
}
