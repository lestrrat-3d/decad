package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// pocketShellBand records the rounded transition below one exact rectangular
// blind pocket. The side ring is the outward-rounded pocket at floorZ; the
// cap ring is the producer's square at lowerZ. The four circular side walks
// collapse to four sphere poles on the cap ring.
type pocketShellBand struct {
	outer, pocket, inner bossRect
	rounded              profileRecord
	z0, floorZ, topZ     float64
	lowerZ, t            float64
	floorFace            int
	frame                r3.Frame
}

// shellBlindPocket takes only the exact two-slab rectangular blind Cut whose
// top cap is removed inward. A different stack retains route S's SG3 refusal.
func shellBlindPocket(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 ||
		len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	lower, upper := sp.slabs[0], sp.slabs[1]
	if len(lower.Regions[0].Holes) != 0 || len(upper.Regions[0].Holes) != 1 ||
		len(sp.interfaces[0].LowerExposed) != 1 || len(sp.interfaces[0].UpperExposed) != 0 ||
		!brepgeom.LoopsEqual(lower.Regions[0].Outer, upper.Regions[0].Outer) {
		return nil, false, nil
	}
	outer, oOK := recordedBossRect(lower.Regions[0])
	pocketLoop, err := offset2d.ReverseLoopRecordContext(ctx, upper.Regions[0].Holes[0])
	if err != nil {
		return nil, true, err
	}
	pocket, pOK := recordedBossRect(profileRecord{Outer: pocketLoop})
	if !oOK || !pOK || !(pocket.u0 > outer.u0 && pocket.v0 > outer.v0 &&
		pocket.u1 < outer.u1 && pocket.v1 < outer.v1) {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 ||
		call.removed[0] != facesByRole(source)[roleCapEnd] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: rectangular blind-pocket shell %s", ErrUnsupported, why)
	}
	if sp.sectionDelta != 0 || lower.Z0Delta != 0 || lower.Z1Delta != 0 ||
		upper.Z0Delta != 0 || upper.Z1Delta != 0 || lower.Z1 != upper.Z0 ||
		!(lower.Z0 < lower.Z1 && upper.Z0 < upper.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.tmm
	if !(lower.Z1-lower.Z0 > 2*t && outer.width() > 2*t && outer.height() > 2*t &&
		pocket.width() > 2*t && pocket.height() > 2*t &&
		pocket.u0 > outer.u0+2*t && pocket.v0 > outer.v0+2*t &&
		pocket.u1 < outer.u1-2*t && pocket.v1 < outer.v1-2*t) {
		return refuse("has insufficient floor, outer-wall, pocket, or cavity clearance")
	}
	inner := outer.inset(t)
	lowerZ := lower.Z1 - t
	if proofbound.ExactSumRound(lowerZ, lower.Z1, -t) != 0 ||
		proofbound.ExactSumRound(lower.Z0+t, lower.Z0, t) != 0 ||
		proofbound.ExactSumRound(lower.Z1-lower.Z0, lower.Z1, -lower.Z0) != 0 ||
		proofbound.ExactSumRound(upper.Z1-upper.Z0, upper.Z1, -upper.Z0) != 0 ||
		!bossRectArithmeticExact(outer, t) || !bossRectArithmeticExact(pocket, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	budget := proofbound.NewWorkBudget(ctx)
	rounded, err := offsetProfile(budget, pocket.profile(), -1, t)
	if err != nil {
		return nil, true, err
	}
	delta, err := offsetSectionDelta(budget, pocket.profile(), -1, t, 0)
	if err != nil {
		return nil, true, err
	}
	if delta != 0 {
		return refuse("needs an exact rounded-pocket offset")
	}
	read, err := filletLoopOf(budget, rounded.Outer, t, "pocket floor", freeform.NewFreeformWork())
	if err != nil {
		return nil, true, err
	}
	joins, err := filletband.OffsetJoins(budget, read.walks, t, 0, shellTol)
	if err != nil {
		return nil, true, err
	}
	collapsed, err := filletband.OffsetLoop(budget, read.walks, joins, t, shellTol)
	if err != nil {
		return nil, true, err
	}
	if len(read.walks) != 8 || !brepgeom.LoopsEqual(loopRecord{Segments: collapsed}, pocket.profile().Outer) {
		return refuse("does not collapse to the source pocket floor")
	}
	for _, walk := range read.walks {
		if walk.IsCircular() && !filletband.SphereWalk(walk, t) {
			return refuse("has a non-spherical floor corner")
		}
	}
	topCapIdx := -1
	for i, face := range view.faces {
		if face.planar() && face.outward && face.z0 == upper.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{xform: view.xform, faces: make([]brepFace, 0, len(view.faces)+20)}
	result.faces = append(result.faces, view.faces[:topCapIdx]...)
	result.faces = append(result.faces, view.faces[topCapIdx+1:]...)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.faces = append(result.faces, brepFace{frame: sp.frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reversed bool) {
		for _, seg := range p.Outer.Segments {
			if reversed {
				seg = reverseSegment(seg)
			}
			result.faces = append(result.faces, brepFace{frame: sp.frame, wall: seg, z0: z0, z1: z1})
		}
	}
	addPlane(inner.profile(), lower.Z0+t, true)
	addWalls(inner.profile(), lower.Z0+t, upper.Z1, true)
	outerRim := outer.profile()
	outerRim.Holes = []loopRecord{}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, inner.profile().Outer)
	if err != nil {
		return nil, true, err
	}
	outerRim.Holes = append(outerRim.Holes, hole)
	addPlane(outerRim, upper.Z1, true)
	pocketRim := rounded
	hole, err = offset2d.ReverseLoopRecordContext(ctx, pocket.profile().Outer)
	if err != nil {
		return nil, true, err
	}
	pocketRim.Holes = []loopRecord{hole}
	addPlane(pocketRim, upper.Z1, true)
	addWalls(rounded, lower.Z1, upper.Z1, false)
	floorFace := addPlane(pocket.profile(), lowerZ, false)
	result.pocketShell = &pocketShellBand{outer: outer, pocket: pocket, inner: inner, rounded: rounded,
		z0: lower.Z0, floorZ: lower.Z1, topZ: upper.Z1, lowerZ: lowerZ, t: t,
		floorFace: floorFace, frame: sp.frame}
	result.assignRoles()
	body, err := evalBrepContext(ctx, source.doc, source.doc.nextProducerID(), result)
	return body, true, err
}

// pocketShellOpenEdges places the rounded side ring and the collapsed square
// floor ring in the walk order the fillet band reads.
func pocketShellOpenEdges(ctx context.Context, bp brepPayload, topo *brepTopology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *Vertex,
	out *brepOpenSet, bi int) error {
	band := bp.pocketShell
	e := topo.embeds[band.floorFace]
	for ring, spec := range []struct {
		loop loopRecord
		z    float64
	}{{band.rounded.Outer, band.floorZ}, {band.pocket.profile().Outer, band.lowerZ}} {
		view := prismPayload{frame: band.frame, xform: bp.xform, z0: spec.z, z1: spec.z}
		for _, seg := range spec.loop.Segments {
			if err := ctx.Err(); err != nil {
				return err
			}
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return err
			}
			key, _ := brepgeom.CurveKey(e, w, spec.z)
			ui, ok := openAt[key]
			if !ok {
				return fmt.Errorf("%w: a blind-pocket shell ring has no open record edge", ErrUnsupported)
			}
			u := topo.uses[ui]
			from, to := e.Canon(w.StartU, w.StartV, spec.z), e.Canon(w.EndU, w.EndV, spec.z)
			start, end := placeVertex(from, 0, 0, 0), placeVertex(to, 0, 0, 0)
			edge, err := brepBandEdge(view, w, true, start, end)
			if err != nil {
				return err
			}
			out.coedge[ui] = coedge{edge: edge, forward: u.DirFrom == from && u.DirTo == to}
			if ring == 0 {
				out.side[bi] = append(out.side[bi], coedge{edge: edge, forward: true})
			} else {
				out.cap[bi] = append(out.cap[bi], coedge{edge: edge, forward: true})
			}
		}
	}
	return nil
}

// attachPocketShellBand reuses LF9's cylinder and sphere patches with a
// virtual upward floor cap. The actual floor is downward-facing and supplies
// the square cap ring; the virtual cap is only the band's frame datum.
func attachPocketShellBand(ctx context.Context, body *Body, ref producerID,
	bp brepPayload, open brepOpenSet) ([]*Face, error) {
	band := bp.pocketShell
	bi := len(bp.loopBands)
	b := brepLoopBand{face: band.floorFace, loop: 0, orig: band.rounded.Outer,
		setback: capSetback{dc: band.t, ds: band.t}, sigma: 1, kind: brepBandFillet}
	capRegion := band.pocket.profile()
	fake := brepFace{frame: band.frame, region: &capRegion,
		z0: band.lowerZ, z1: band.lowerZ, outward: true, role: "pocketShellBand"}
	copyBP := bp
	copyBP.faces = append([]brepFace(nil), bp.faces...)
	copyBP.faces[band.floorFace] = fake
	copyBP.loopBands = append(append([]brepLoopBand(nil), bp.loopBands...), b)
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, body, ref, copyBP, bi, open,
		embeds[band.floorFace], freeform.NewFreeformWork())
	return patches, err
}

// measurePocketShell subtracts the eroded outer box and adds the exact
// Minkowski expansion of the blind pocket within that cavity.
func measurePocketShell(ctx context.Context, bp brepPayload, body *Body) error {
	b := bp.pocketShell
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r bossRect) proofbound.RatInterval {
		return mul(sub(point(r.u1), point(r.u0)), sub(point(r.v1), point(r.v0)))
	}
	centerU := func(r bossRect) proofbound.RatInterval { return scale(add(point(r.u0), point(r.u1)), 1, 2) }
	centerV := func(r bossRect) proofbound.RatInterval { return scale(add(point(r.v0), point(r.v1)), 1, 2) }
	t := point(b.t)
	outerH := sub(point(b.topZ), point(b.z0))
	pocketH := sub(point(b.topZ), point(b.floorZ))
	innerH := sub(point(b.topZ), add(point(b.z0), t))
	w := sub(point(b.pocket.u1), point(b.pocket.u0))
	d := sub(point(b.pocket.v1), point(b.pocket.v0))
	perimHalf := add(w, d)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	outerV := mul(area(b.outer), outerH)
	pocketV := mul(area(b.pocket), pocketH)
	innerV := mul(area(b.inner), innerH)
	roundedArea := add(add(area(b.pocket), scale(mul(perimHalf, t), 2, 1)), mul(pi, t2))
	upperV := mul(roundedArea, pocketH)
	lowerV := add(add(mul(area(b.pocket), t), scale(mul(mul(perimHalf, pi), t2), 1, 2)),
		scale(mul(pi, t3), 2, 3))
	obstacleV := add(upperV, lowerV)
	volume := add(sub(sub(outerV, pocketV), innerV), obstacleV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: blind-pocket shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.volume = Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: exactnessOf(ve)}
	outerCZ := scale(add(point(b.z0), point(b.topZ)), 1, 2)
	pocketCZ := scale(add(point(b.floorZ), point(b.topZ)), 1, 2)
	innerCZ := scale(add(add(point(b.z0), t), point(b.topZ)), 1, 2)
	lowerFirst := sub(mul(point(b.floorZ), lowerV),
		add(add(scale(mul(area(b.pocket), t2), 1, 2), scale(mul(perimHalf, t3), 2, 3)),
			scale(mul(pi, t4), 1, 4)))
	upperFirst := mul(upperV, pocketCZ)
	obstacleZ := add(lowerFirst, upperFirst)
	mu := add(sub(sub(mul(outerV, centerU(b.outer)), mul(pocketV, centerU(b.pocket))),
		mul(innerV, centerU(b.inner))), mul(obstacleV, centerU(b.pocket)))
	mv := add(sub(sub(mul(outerV, centerV(b.outer)), mul(pocketV, centerV(b.pocket))),
		mul(innerV, centerV(b.inner))), mul(obstacleV, centerV(b.pocket)))
	mz := add(sub(sub(mul(outerV, outerCZ), mul(pocketV, pocketCZ)),
		mul(innerV, innerCZ)), obstacleZ)
	ivs := []proofbound.RatInterval{mu, mv, mz}
	var c [3]proofbound.BoundedScalar
	for i, iv := range ivs {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: blind-pocket shell centroid has no finite enclosure", ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.refView()
	cb := prismPointBound(ref, c[0], c[1], c[2])
	body.centroid = VecMeasurement{Value: ref.point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: exactnessOf(cb)}
	var areaTotal proofbound.BoundedScalar
	for _, f := range body.Faces() {
		areaTotal = proofbound.BoundedAdd(areaTotal, proofbound.MeasuredScalar(f.area, f.areaBound))
	}
	body.area = Measurement{Value: units.SquareMillimeters(areaTotal.Value),
		Bound: units.SquareMillimeters(areaTotal.Bound), Exactness: exactnessOf(areaTotal.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.bounds = box
	return validateAnalyticBodyMeasurements(body)
}
