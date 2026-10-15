package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// shellBlindPocket takes only the exact two-slab rectangular blind Cut whose
// top cap is removed inward. A different stack retains route S's SG3 refusal.
func shellBlindPocket(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	lower, upper := sp.Slabs[0], sp.Slabs[1]
	if len(lower.Regions[0].Holes) != 0 || len(upper.Regions[0].Holes) != 1 ||
		len(sp.Interfaces[0].LowerExposed) != 1 || len(sp.Interfaces[0].UpperExposed) != 0 ||
		!brepgeom.LoopsEqual(lower.Regions[0].Outer, upper.Regions[0].Outer) {
		return nil, false, nil
	}
	outer, oOK := prismshell.RecordedBossRect(lower.Regions[0])
	pocketLoop, err := offset2d.ReverseLoopRecordContext(ctx, upper.Regions[0].Holes[0])
	if err != nil {
		return nil, true, err
	}
	pocket, pOK := prismshell.RecordedBossRect(profileRecord{Outer: pocketLoop})
	if !oOK || !pOK || !(pocket.U0 > outer.U0 && pocket.V0 > outer.V0 &&
		pocket.U1 < outer.U1 && pocket.V1 < outer.V1) {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 ||
		call.removed[0] != facesByRole(source)[provenance.RoleCapEnd] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: rectangular blind-pocket shell %s", ErrUnsupported, why)
	}
	if sp.SectionDelta != 0 || lower.Z0Delta != 0 || lower.Z1Delta != 0 ||
		upper.Z0Delta != 0 || upper.Z1Delta != 0 || lower.Z1 != upper.Z0 ||
		!(lower.Z0 < lower.Z1 && upper.Z0 < upper.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.tmm
	if !(lower.Z1-lower.Z0 > 2*t && outer.Width() > 2*t && outer.Height() > 2*t &&
		pocket.Width() > 2*t && pocket.Height() > 2*t &&
		pocket.U0 > outer.U0+2*t && pocket.V0 > outer.V0+2*t &&
		pocket.U1 < outer.U1-2*t && pocket.V1 < outer.V1-2*t) {
		return refuse("has insufficient floor, outer-wall, pocket, or cavity clearance")
	}
	inner := outer.Inset(t)
	lowerZ := lower.Z1 - t
	if proofbound.ExactSumRound(lowerZ, lower.Z1, -t) != 0 ||
		proofbound.ExactSumRound(lower.Z0+t, lower.Z0, t) != 0 ||
		proofbound.ExactSumRound(lower.Z1-lower.Z0, lower.Z1, -lower.Z0) != 0 ||
		proofbound.ExactSumRound(upper.Z1-upper.Z0, upper.Z1, -upper.Z0) != 0 ||
		!bossRectArithmeticExact(outer, t) || !bossRectArithmeticExact(pocket, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	budget := proofbound.NewWorkBudget(ctx)
	rounded, err := offsetProfile(budget, pocket.Profile(), -1, t)
	if err != nil {
		return nil, true, err
	}
	delta, err := offsetSectionDelta(budget, pocket.Profile(), -1, t, 0)
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
	if len(read.walks) != 8 || !brepgeom.LoopsEqual(loopRecord{Segments: collapsed}, pocket.Profile().Outer) {
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
		result.faces = append(result.faces, brepFace{frame: sp.Frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reversed bool) {
		for _, seg := range p.Outer.Segments {
			if reversed {
				seg = brepgeom.ReverseSegment(seg)
			}
			result.faces = append(result.faces, brepFace{frame: sp.Frame, wall: seg, z0: z0, z1: z1})
		}
	}
	addPlane(inner.Profile(), lower.Z0+t, true)
	addWalls(inner.Profile(), lower.Z0+t, upper.Z1, true)
	outerRim := outer.Profile()
	outerRim.Holes = []loopRecord{}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, inner.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	outerRim.Holes = append(outerRim.Holes, hole)
	addPlane(outerRim, upper.Z1, true)
	pocketRim := rounded
	hole, err = offset2d.ReverseLoopRecordContext(ctx, pocket.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	pocketRim.Holes = []loopRecord{hole}
	addPlane(pocketRim, upper.Z1, true)
	addWalls(rounded, lower.Z1, upper.Z1, false)
	floorFace := addPlane(pocket.Profile(), lowerZ, false)
	result.pocketShell = &prismshell.PocketShellBand{Outer: outer, Pocket: pocket, Inner: inner, Rounded: rounded,
		Z0: lower.Z0, FloorZ: lower.Z1, TopZ: upper.Z1, LowerZ: lowerZ, T: t,
		FloorFace: floorFace, Frame: sp.Frame}
	result.assignRoles()
	body, err := evalBrepContext(ctx, documentHandle(source.n.Doc), source.n.Doc.NextProducer, result)
	return body, true, err
}

// pocketShellOpenEdges places the rounded side ring and the collapsed square
// floor ring in the walk order the fillet band reads.
func pocketShellOpenEdges(ctx context.Context, bp brepPayload, btopo *brepTopology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *Vertex,
	out *brepOpenSet, bi int) error {
	band := bp.pocketShell
	e := btopo.embeds[band.FloorFace]
	for ring, spec := range []struct {
		loop loopRecord
		z    float64
	}{{band.Rounded.Outer, band.FloorZ}, {band.CapProfile().Outer, band.LowerZ}} {
		view := prismrecord.Payload{Frame: band.Frame, Xform: bp.xform, Z0: spec.z, Z1: spec.z}
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
			u := btopo.uses[ui]
			from, to := e.Canon(w.StartU, w.StartV, spec.z), e.Canon(w.EndU, w.EndV, spec.z)
			start, end := placeVertex(from, 0, 0, 0), placeVertex(to, 0, 0, 0)
			edge, err := brepBandEdge(view, w, true, start, end)
			if err != nil {
				return err
			}
			out.coedge[ui] = topo.CoEdge{Edge: edgeNode(edge), Forward: u.DirFrom == from && u.DirTo == to}
			if ring == 0 {
				out.side[bi] = append(out.side[bi], topo.CoEdge{Edge: edgeNode(edge), Forward: true})
			} else {
				out.cap[bi] = append(out.cap[bi], topo.CoEdge{Edge: edgeNode(edge), Forward: true})
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
	b := brepLoopBand{face: band.FloorFace, loop: 0, orig: band.Rounded.Outer,
		setback: capSetback{Dc: band.T, Ds: band.T}, sigma: 1, kind: brepBandFillet}
	capRegion := band.CapProfile()
	fake := brepFace{frame: band.Frame, region: &capRegion,
		z0: band.LowerZ, z1: band.LowerZ, outward: band.CapOutward(), role: "pocketShellBand"}
	copyBP := bp
	copyBP.faces = append([]brepFace(nil), bp.faces...)
	copyBP.faces[band.FloorFace] = fake
	copyBP.loopBands = append(append([]brepLoopBand(nil), bp.loopBands...), b)
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, body, ref, copyBP, bi, open,
		embeds[band.FloorFace], freeform.NewFreeformWork())
	return patches, err
}

// measurePocketShell subtracts the eroded outer box and adds the exact
// Minkowski expansion of the blind pocket within that cavity.
func measurePocketShell(ctx context.Context, bp brepPayload, body *Body) error {
	b := bp.pocketShell
	if b.Round != nil {
		return measureCounterboreShell(ctx, bp, body)
	}
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r prismshell.BossRect) proofbound.RatInterval {
		return mul(sub(point(r.U1), point(r.U0)), sub(point(r.V1), point(r.V0)))
	}
	centerU := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.U0), point(r.U1)), 1, 2) }
	centerV := func(r prismshell.BossRect) proofbound.RatInterval { return scale(add(point(r.V0), point(r.V1)), 1, 2) }
	t := point(b.T)
	outerH := sub(point(b.TopZ), point(b.Z0))
	pocketH := sub(point(b.TopZ), point(b.FloorZ))
	innerH := sub(point(b.TopZ), add(point(b.Z0), t))
	w := sub(point(b.Pocket.U1), point(b.Pocket.U0))
	d := sub(point(b.Pocket.V1), point(b.Pocket.V0))
	perimHalf := add(w, d)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	outerV := mul(area(b.Outer), outerH)
	pocketV := mul(area(b.Pocket), pocketH)
	innerV := mul(area(b.Inner), innerH)
	roundedArea := add(add(area(b.Pocket), scale(mul(perimHalf, t), 2, 1)), mul(pi, t2))
	upperV := mul(roundedArea, pocketH)
	lowerV := add(add(mul(area(b.Pocket), t), scale(mul(mul(perimHalf, pi), t2), 1, 2)),
		scale(mul(pi, t3), 2, 3))
	obstacleV := add(upperV, lowerV)
	volume := add(sub(sub(outerV, pocketV), innerV), obstacleV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: blind-pocket shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	outerCZ := scale(add(point(b.Z0), point(b.TopZ)), 1, 2)
	pocketCZ := scale(add(point(b.FloorZ), point(b.TopZ)), 1, 2)
	innerCZ := scale(add(add(point(b.Z0), t), point(b.TopZ)), 1, 2)
	lowerFirst := sub(mul(point(b.FloorZ), lowerV),
		add(add(scale(mul(area(b.Pocket), t2), 1, 2), scale(mul(perimHalf, t3), 2, 3)),
			scale(mul(pi, t4), 1, 4)))
	upperFirst := mul(upperV, pocketCZ)
	obstacleZ := add(lowerFirst, upperFirst)
	mu := add(sub(sub(mul(outerV, centerU(b.Outer)), mul(pocketV, centerU(b.Pocket))),
		mul(innerV, centerU(b.Inner))), mul(obstacleV, centerU(b.Pocket)))
	mv := add(sub(sub(mul(outerV, centerV(b.Outer)), mul(pocketV, centerV(b.Pocket))),
		mul(innerV, centerV(b.Inner))), mul(obstacleV, centerV(b.Pocket)))
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
	cb := ref.PointBound(c[0], c[1], c[2])
	body.n.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var areaTotal proofbound.BoundedScalar
	for _, f := range body.Faces() {
		areaTotal = proofbound.BoundedAdd(areaTotal, proofbound.MeasuredScalar(f.n.Area, f.n.AreaBound))
	}
	body.n.Area = measurement.Measurement{Value: units.SquareMillimeters(areaTotal.Value),
		Bound: units.SquareMillimeters(areaTotal.Bound), Exactness: measurement.ExactnessOf(areaTotal.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.n.Bounds = boxToInternal(box)
	return topobuild.ValidateAnalyticMeasurements(bodyNode(body))
}
