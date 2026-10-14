package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// reverseClosedBossStack makes a narrow lower slab the upper boss of the
// existing certified construction. Flipping the frame's V axis, each section's
// V coordinates, and the recorded levels preserves the same world geometry
// and a right-handed face frame.
func reverseClosedBossStack(ctx context.Context, sp stackedPrismPayload) (stackedPrismPayload, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 ||
		len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	inner, innerOK := prismshell.RecordedBossRect(sp.slabs[0].Regions[0])
	outer, outerOK := prismshell.RecordedBossRect(sp.slabs[1].Regions[0])
	if !innerOK || !outerOK || !(inner.U0 > outer.U0 && inner.V0 > outer.V0 &&
		inner.U1 < outer.U1 && inner.V1 < outer.V1) {
		return stackedPrismPayload{}, false, nil
	}
	frame, err := r3.NewFrame(sp.frame.Origin(), sp.frame.U(), sp.frame.V().Scale(-1))
	if err != nil {
		return stackedPrismPayload{}, true, fmt.Errorf("%w: the closed boss shell cannot reverse its section frame: %v",
			ErrUnsupported, err)
	}
	if frame.Origin() != sp.frame.Origin() || frame.U() != sp.frame.U() ||
		frame.V() != sp.frame.V().Scale(-1) || frame.N() != sp.frame.N().Scale(-1) {
		return stackedPrismPayload{}, true, fmt.Errorf("%w: the closed boss shell needs an exact reversed section frame",
			ErrUnsupported)
	}
	reflectRect := func(r prismshell.BossRect) profileRecord {
		return (prismshell.BossRect{U0: r.U0, V0: -r.V1, U1: r.U1, V1: -r.V0}).Profile()
	}
	exposed, err := stackedrecord.UnionExposed(ctx, reflectRect(outer), reflectRect(inner))
	if err != nil {
		return stackedPrismPayload{}, true, fmt.Errorf("%w: the closed boss shell cannot reverse its interface: %v",
			ErrUnsupported, err)
	}
	a, b := sp.slabs[0], sp.slabs[1]
	return stackedPrismPayload{
		slabs: []stackedrecord.Slab{
			{Regions: []profileRecord{reflectRect(outer)}, Z0: -b.Z1, Z1: -b.Z0, Z0Delta: b.Z1Delta, Z1Delta: b.Z0Delta},
			{Regions: []profileRecord{reflectRect(inner)}, Z0: -a.Z1, Z1: -a.Z0, Z0Delta: a.Z1Delta, Z1Delta: a.Z0Delta},
		},
		interfaces: []stackedrecord.Interface{{
			LowerExposed: exposed,
		}},
		frame: frame, xform: sp.xform, sectionDelta: sp.sectionDelta,
	}, true, nil
}

// shellStackedBoss handles only the exact two-slab, nested rectangular case.
// An unrelated stack falls through to the ordinary Shell route.
func shellStackedBoss(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 || len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	plate, pOK := prismshell.RecordedBossRect(sp.slabs[0].Regions[0])
	boss, bOK := prismshell.RecordedBossRect(sp.slabs[1].Regions[0])
	if !pOK || !bOK || !(boss.U0 > plate.U0 && boss.V0 > plate.V0 && boss.U1 < plate.U1 && boss.V1 < plate.V1) {
		return nil, false, nil
	}
	closed := len(call.removed) == 0
	if call.sense != Inward || (!closed &&
		(len(call.removed) != 1 || call.removed[0] != facesByRole(source)[provenance.RoleCapEnd])) {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: stacked boss shell %s", ErrUnsupported, why)
	}
	a, b := sp.slabs[0], sp.slabs[1]
	if sp.sectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 ||
		a.Z1 != b.Z0 || !(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t := call.tmm
	if !(a.Z1-a.Z0 > 2*t && boss.Width() > 2*t && boss.Height() > 2*t &&
		boss.U0 > plate.U0+t && boss.V0 > plate.V0+t && boss.U1 < plate.U1-t && boss.V1 < plate.V1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	if closed && b.Z1-b.Z0 <= t {
		return refuse("has no cavity below the kept boss roof")
	}
	lower, upper := plate.Inset(t), boss.Inset(t)
	lowerZ := a.Z1 - t
	cavityTopZ := b.Z1
	if closed {
		cavityTopZ = b.Z1 - t
		if proofbound.ExactSumRound(cavityTopZ, b.Z1, -t) != 0 {
			return refuse("needs an exactly represented cavity ceiling")
		}
	}
	if proofbound.ExactSumRound(lowerZ, a.Z1, -t) != 0 ||
		proofbound.ExactSumRound(a.Z0+t, a.Z0, t) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0, a.Z1, -a.Z0) != 0 ||
		proofbound.ExactSumRound(b.Z1-b.Z0, b.Z1, -b.Z0) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0-2*t, a.Z1, -a.Z0, -t, -t) != 0 ||
		!bossRectArithmeticExact(plate, t) || !bossRectArithmeticExact(boss, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	// The source view has a single top cap. All its other faces keep their
	// recorded geometry; the result gets new face(k)/wall(k) roles on rebuild.
	topCapIdx := -1
	for i, f := range view.faces {
		if f.planar() && f.outward && f.z0 == b.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{xform: view.xform, faces: make([]brepFace, 0, len(view.faces)+16)}
	if closed {
		result.faces = append(result.faces, view.faces...)
	} else {
		result.faces = append(result.faces, view.faces[:topCapIdx]...)
		result.faces = append(result.faces, view.faces[topCapIdx+1:]...)
	}
	outerFaceCount := len(result.faces)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.faces = append(result.faces, brepFace{frame: sp.frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(r prismshell.BossRect, z0, z1 float64) {
		for _, seg := range r.Profile().Outer.Segments {
			result.faces = append(result.faces, brepFace{frame: sp.frame, wall: brepgeom.ReverseSegment(seg), z0: z0, z1: z1})
		}
	}
	addPlane(lower.Profile(), a.Z0+t, true)
	addWalls(lower, a.Z0+t, lowerZ)
	ledge := profileRecord{Outer: lower.Profile().Outer}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, boss.Profile().Outer)
	if err != nil {
		return nil, true, err
	}
	ledge.Holes = []loopRecord{hole}
	ledgeFace := addPlane(ledge, lowerZ, false)
	addWalls(upper, b.Z0, cavityTopZ)
	if closed {
		addPlane(upper.Profile(), cavityTopZ, false)
	} else {
		rim := profileRecord{Outer: boss.Profile().Outer}
		hole, err = offset2d.ReverseLoopRecordContext(ctx, upper.Profile().Outer)
		if err != nil {
			return nil, true, err
		}
		rim.Holes = []loopRecord{hole}
		addPlane(rim, b.Z1, true)
	}
	result.bossShell = &prismshell.BossShellBand{Plate: plate, Boss: boss, Lower: lower, Upper: upper,
		Z0: a.Z0, InterfaceZ: a.Z1, TopZ: b.Z1, CavityTopZ: cavityTopZ,
		LowerZ: lowerZ, T: t, LedgeFace: ledgeFace, Frame: sp.frame,
		OuterFaceCount: outerFaceCount, Closed: closed}
	result.assignRoles()
	body, err := evalBrepContext(ctx, source.doc, source.doc.nextProducerID(), result)
	return body, true, err
}

func bossRectArithmeticExact(r prismshell.BossRect, t float64) bool {
	if proofbound.ExactSumRound(r.Width(), r.U1, -r.U0) != 0 ||
		proofbound.ExactSumRound(r.Height(), r.V1, -r.V0) != 0 {
		return false
	}
	for _, x := range [4]float64{r.U0, r.V0, r.U1, r.V1} {
		if proofbound.ExactSumRound(x+t, x, t) != 0 || proofbound.ExactSumRound(x-t, x, -t) != 0 {
			return false
		}
	}
	in := r.Inset(t)
	if proofbound.ExactSumRound(in.Width(), in.U1, -in.U0) != 0 ||
		proofbound.ExactSumRound(in.Height(), in.V1, -in.V0) != 0 {
		return false
	}
	return true
}

// closedBossShellLumps assigns the inner face component to the one void shell.
// shellStackedBoss has already proved that the nested cavity stays clear of
// every exterior wall, the floor and the kept roof. Every source face must
// therefore lie in the one outer component, and all derived cavity faces in
// exactly one other closed component.
func closedBossShellLumps(faces []*Face, outerFaceCount int) ([]*Lump, error) {
	if outerFaceCount <= 0 || outerFaceCount >= len(faces) {
		return nil, fmt.Errorf("%w: a closed boss shell has no distinct exterior and cavity", ErrUnsupported)
	}
	outerFaces := make(map[*Face]struct{}, outerFaceCount)
	for _, face := range faces[:outerFaceCount] {
		outerFaces[face] = struct{}{}
	}
	groups := splitConnectedFaces(faces)
	if len(groups) != 2 {
		return nil, fmt.Errorf("%w: a closed boss shell has %d face components, expected an exterior and one cavity",
			ErrUnsupported, len(groups))
	}
	var outer, cavity []*Face
	for _, group := range groups {
		anchors := 0
		for _, face := range group {
			if _, ok := outerFaces[face]; ok {
				anchors++
			}
		}
		switch {
		case anchors == outerFaceCount && len(group) == outerFaceCount:
			outer = group
		case anchors == 0:
			cavity = group
		default:
			return nil, fmt.Errorf("%w: a closed boss shell mixes exterior and cavity faces", ErrUnsupported)
		}
	}
	if len(outer) != outerFaceCount || len(cavity) != len(faces)-outerFaceCount ||
		shellIsOpen(outer) || shellIsOpen(cavity) ||
		!closedBossShellEdgePairs(outer) || !closedBossShellEdgePairs(cavity) {
		return nil, fmt.Errorf("%w: a closed boss shell needs two closed face components", ErrUnsupported)
	}
	return []*Lump{{shells: []*FaceShell{{faces: outer}, {faces: cavity, n: topo.FaceShell{Void: true}}}}}, nil
}

// Each component must use every edge exactly twice. A free edge or a repeated
// edge cannot pass merely because the face graph has two components. The
// coedge's forward bit records curve direction, not outward face winding.
func closedBossShellEdgePairs(faces []*Face) bool {
	edges := map[*Edge]int{}
	for _, face := range faces {
		for _, loop := range face.n.Loops {
			for _, use := range loop.CoEdges {
				edges[edgeHandle(use.Edge)]++
			}
		}
	}
	if len(edges) == 0 {
		return false
	}
	for edge, uses := range edges {
		if uses != 2 || len(edge.n.Faces) != 2 || edge.n.Faces[0] == edge.n.Faces[1] {
			return false
		}
	}
	return true
}

// bossShellOpenEdges supplies the two four-edge rings in the common
// counter-clockwise walk. Each ordinary face uses its edge in its own sense.
func bossShellOpenEdges(ctx context.Context, bp brepPayload, btopo *brepTopology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *Vertex,
	out *brepOpenSet, bi int) error {
	band := bp.bossShell
	e := btopo.embeds[band.LedgeFace]
	for ring, spec := range []struct {
		profile profileRecord
		z       float64
	}{{band.SideProfile(), band.LowerZ}, {band.CapProfile(), band.InterfaceZ}} {
		view := prismPayload{Payload: prismrecord.Payload{Frame: band.Frame, Xform: bp.xform, Z0: spec.z, Z1: spec.z}}
		for _, seg := range spec.profile.Outer.Segments {
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
				return fmt.Errorf("%w: a boss shell ring has no open record edge", ErrUnsupported)
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
			}
			if ring == 1 {
				out.cap[bi] = append(out.cap[bi], topo.CoEdge{Edge: edgeNode(edge), Forward: true})
			}
		}
	}
	return nil
}

// attachBossShellBand uses the established analytic fillet patch constructor
// with a virtual cap at the upper ring. The cap is only a geometry datum:
// its ring is actually bounded by the upper cavity walls.
func attachBossShellBand(ctx context.Context, body *Body, ref producerID, bp brepPayload, open brepOpenSet) ([]*Face, error) {
	band := bp.bossShell
	bi := len(bp.loopBands)
	b := brepLoopBand{face: band.LedgeFace, loop: 0, orig: band.SideProfile().Outer,
		setback: capSetback{dc: band.T, ds: band.T}, sigma: 1, kind: brepBandFillet}
	upper := band.CapProfile()
	fake := brepFace{frame: band.Frame, region: &upper,
		z0: band.InterfaceZ, z1: band.InterfaceZ, outward: false, role: "bossShellBand"}
	copyBP := bp
	copyBP.faces = append([]brepFace(nil), bp.faces...)
	copyBP.faces[band.LedgeFace] = fake
	copyBP.loopBands = append(append([]brepLoopBand(nil), bp.loopBands...), b)
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, body, ref, copyBP, bi, open, embeds[band.LedgeFace], freeform.NewFreeformWork())
	return patches, err
}

// measureBossShell integrates its three cavity intervals exactly, with pi
// enclosed by proofbound's rational interval. Face areas already carry each
// planar or cylindrical patch's own independent bound.
func measureBossShell(ctx context.Context, bp brepPayload, body *Body) error {
	if bp.bossShell.Round != nil {
		return measureRoundBossShell(ctx, bp, body)
	}
	b := bp.bossShell
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
	h := sub(point(b.InterfaceZ), point(b.Z0))
	hb := sub(point(b.TopZ), point(b.InterfaceZ))
	bw, bh := point(b.Boss.Width()), point(b.Boss.Height())
	plateV, bossV := mul(area(b.Plate), h), mul(area(b.Boss), hb)
	lowerV := mul(area(b.Lower), sub(h, scale(t, 2, 1)))
	upperHeight := sub(point(b.CavityTopZ), point(b.InterfaceZ))
	upperV := mul(area(b.Upper), upperHeight)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	widths := add(bw, bh)
	bandV := add(
		add(sub(mul(area(b.Boss), t), scale(mul(widths, t2), 2, 1)),
			scale(mul(mul(widths, pi), t2), 1, 2)),
		sub(scale(t3, 20, 3), scale(mul(pi, t3), 2, 1)),
	)
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: boss shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: measurement.ExactnessOf(ve)}
	// z is measured from the recorded frame origin, not from the plate base.
	plateCZ := scale(add(point(b.Z0), point(b.InterfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.InterfaceZ), point(b.TopZ)), 1, 2)
	upperCZ := scale(add(point(b.InterfaceZ), point(b.CavityTopZ)), 1, 2)
	bandZ := add(mul(point(b.LowerZ), bandV),
		add(sub(scale(mul(area(b.Boss), t2), 1, 2), scale(mul(widths, t3), 1, 3)), scale(t4, 1, 3)))
	mu := sub(sub(add(mul(plateV, centerU(b.Plate)), mul(bossV, centerU(b.Boss))),
		add(mul(lowerV, centerU(b.Lower)), mul(upperV, centerU(b.Upper)))), mul(bandV, centerU(b.Boss)))
	mv := sub(sub(add(mul(plateV, centerV(b.Plate)), mul(bossV, centerV(b.Boss))),
		add(mul(lowerV, centerV(b.Lower)), mul(upperV, centerV(b.Upper)))), mul(bandV, centerV(b.Boss)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, plateCZ), mul(upperV, upperCZ))), bandZ)
	ivs := []proofbound.RatInterval{mu, mv, mz}
	var c [3]proofbound.BoundedScalar
	for i, iv := range ivs {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: boss shell centroid has no finite enclosure", ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.refView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.n.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value), Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var a proofbound.BoundedScalar
	for _, f := range body.Faces() {
		a = proofbound.BoundedAdd(a, proofbound.MeasuredScalar(f.n.Area, f.n.AreaBound))
	}
	body.n.Area = measurement.Measurement{Value: units.SquareMillimeters(a.Value), Bound: units.SquareMillimeters(a.Bound), Exactness: measurement.ExactnessOf(a.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.n.Bounds = boxToInternal(box)
	return validateAnalyticBodyMeasurements(body)
}
