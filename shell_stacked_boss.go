package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// bossShellBand records the one rounded cavity transition of an inward shell
// on a two-slab rectangular boss. The two open rings belong to the inner
// ledge at lowerZ and the upper cavity walls at interfaceZ.
type bossShellBand struct {
	plate, boss, lower, upper    bossRect
	round                        *roundBossCircle
	z0, interfaceZ, topZ, lowerZ float64
	t                            float64
	ledgeFace                    int
	frame                        r3.Frame
}

func (b *bossShellBand) sideProfile() profileRecord {
	if b.round != nil {
		return b.round.outer
	}
	return b.boss.profile()
}

func (b *bossShellBand) capProfile() profileRecord {
	if b.round != nil {
		return b.round.inner
	}
	return b.upper.profile()
}

type bossRect struct{ u0, v0, u1, v1 float64 }

func (r bossRect) width() float64  { return r.u1 - r.u0 }
func (r bossRect) height() float64 { return r.v1 - r.v0 }
func (r bossRect) inset(t float64) bossRect {
	return bossRect{r.u0 + t, r.v0 + t, r.u1 - t, r.v1 - t}
}
func (r bossRect) profile() profileRecord {
	pts := [4]Point2{{U: r.u0, V: r.v0}, {U: r.u1, V: r.v0}, {U: r.u1, V: r.v1}, {U: r.u0, V: r.v1}}
	segs := make([]curveSegment, 4)
	for i := range segs {
		segs[i] = lineSeg{Start: pts[i], End: pts[(i+1)%4], TEnd: 1}
	}
	return profileRecord{Outer: loopRecord{Segments: segs}}
}

// recordedBossRect accepts a complete, counter-clockwise axis-aligned
// rectangle only. It reads the producer's segments, never a sampled outline.
func recordedBossRect(p profileRecord) (bossRect, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 4 {
		return bossRect{}, false
	}
	var r bossRect
	var pts [4]Point2
	for i, seg := range p.Outer.Segments {
		line, ok := seg.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return bossRect{}, false
		}
		if (line.Start.U == line.End.U) == (line.Start.V == line.End.V) {
			return bossRect{}, false
		}
		pts[i] = line.Start
		if i > 0 {
			prev, ok := p.Outer.Segments[i-1].(lineSeg)
			if !ok || prev.End != line.Start {
				return bossRect{}, false
			}
		}
		if i == 0 {
			r = bossRect{line.Start.U, line.Start.V, line.Start.U, line.Start.V}
		}
		r.u0, r.u1 = math.Min(r.u0, line.Start.U), math.Max(r.u1, line.Start.U)
		r.v0, r.v1 = math.Min(r.v0, line.Start.V), math.Max(r.v1, line.Start.V)
		if i == 3 && line.End != pts[0] {
			return bossRect{}, false
		}
	}
	if !(r.width() > 0 && r.height() > 0) {
		return bossRect{}, false
	}
	seen := map[Point2]bool{}
	for _, p := range pts {
		if seen[p] || !((p.U == r.u0 || p.U == r.u1) && (p.V == r.v0 || p.V == r.v1)) {
			return bossRect{}, false
		}
		seen[p] = true
	}
	a, b := pts[0], pts[1]
	c := pts[2]
	cross := (b.U-a.U)*(c.V-b.V) - (b.V-a.V)*(c.U-b.U)
	return r, cross > 0 && !proofbound.IsNonFinite(r.width()) && !proofbound.IsNonFinite(r.height())
}

// shellStackedBoss handles only the exact two-slab, nested rectangular case.
// An unrelated stack falls through to the ordinary Shell route.
func shellStackedBoss(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 || len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	plate, pOK := recordedBossRect(sp.slabs[0].Regions[0])
	boss, bOK := recordedBossRect(sp.slabs[1].Regions[0])
	if !pOK || !bOK || !(boss.u0 > plate.u0 && boss.v0 > plate.v0 && boss.u1 < plate.u1 && boss.v1 < plate.v1) {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 || call.removed[0] != facesByRole(source)[roleCapEnd] {
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
	if !(a.Z1-a.Z0 > 2*t && boss.width() > 2*t && boss.height() > 2*t &&
		boss.u0 > plate.u0+t && boss.v0 > plate.v0+t && boss.u1 < plate.u1-t && boss.v1 < plate.v1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	lower, upper := plate.inset(t), boss.inset(t)
	lowerZ := a.Z1 - t
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
	result := brepPayload{xform: view.xform, faces: make([]brepFace, 0, len(view.faces)+15)}
	result.faces = append(result.faces, view.faces[:topCapIdx]...)
	result.faces = append(result.faces, view.faces[topCapIdx+1:]...)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.faces = append(result.faces, brepFace{frame: sp.frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(r bossRect, z0, z1 float64) {
		for _, seg := range r.profile().Outer.Segments {
			result.faces = append(result.faces, brepFace{frame: sp.frame, wall: reverseSegment(seg), z0: z0, z1: z1})
		}
	}
	addPlane(lower.profile(), a.Z0+t, true)
	addWalls(lower, a.Z0+t, lowerZ)
	ledge := profileRecord{Outer: lower.profile().Outer}
	hole, err := offset2d.ReverseLoopRecordContext(ctx, boss.profile().Outer)
	if err != nil {
		return nil, true, err
	}
	ledge.Holes = []loopRecord{hole}
	ledgeFace := addPlane(ledge, lowerZ, false)
	addWalls(upper, b.Z0, b.Z1)
	rim := profileRecord{Outer: boss.profile().Outer}
	hole, err = offset2d.ReverseLoopRecordContext(ctx, upper.profile().Outer)
	if err != nil {
		return nil, true, err
	}
	rim.Holes = []loopRecord{hole}
	addPlane(rim, b.Z1, true)
	result.bossShell = &bossShellBand{plate: plate, boss: boss, lower: lower, upper: upper,
		z0: a.Z0, interfaceZ: a.Z1, topZ: b.Z1, lowerZ: lowerZ, t: t, ledgeFace: ledgeFace, frame: sp.frame}
	result.assignRoles()
	body, err := evalBrepContext(ctx, source.doc, source.doc.nextProducerID(), result)
	return body, true, err
}

func bossRectArithmeticExact(r bossRect, t float64) bool {
	if proofbound.ExactSumRound(r.width(), r.u1, -r.u0) != 0 ||
		proofbound.ExactSumRound(r.height(), r.v1, -r.v0) != 0 {
		return false
	}
	for _, x := range [4]float64{r.u0, r.v0, r.u1, r.v1} {
		if proofbound.ExactSumRound(x+t, x, t) != 0 || proofbound.ExactSumRound(x-t, x, -t) != 0 {
			return false
		}
	}
	in := r.inset(t)
	if proofbound.ExactSumRound(in.width(), in.u1, -in.u0) != 0 ||
		proofbound.ExactSumRound(in.height(), in.v1, -in.v0) != 0 {
		return false
	}
	return true
}

// bossShellOpenEdges supplies the two four-edge rings in the common
// counter-clockwise walk. Each ordinary face uses its edge in its own sense.
func bossShellOpenEdges(ctx context.Context, bp brepPayload, topo *brepTopology,
	openAt map[brepgeom.EdgeKey]int,
	placeVertex func([3]float64, float64, float64, float64) *Vertex,
	out *brepOpenSet, bi int) error {
	band := bp.bossShell
	e := topo.embeds[band.ledgeFace]
	for ring, spec := range []struct {
		profile profileRecord
		z       float64
	}{{band.sideProfile(), band.lowerZ}, {band.capProfile(), band.interfaceZ}} {
		view := prismPayload{frame: band.frame, xform: bp.xform, z0: spec.z, z1: spec.z}
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
			}
			if ring == 1 {
				out.cap[bi] = append(out.cap[bi], coedge{edge: edge, forward: true})
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
	b := brepLoopBand{face: band.ledgeFace, loop: 0, orig: band.sideProfile().Outer,
		setback: capSetback{dc: band.t, ds: band.t}, sigma: 1, kind: brepBandFillet}
	upper := band.capProfile()
	fake := brepFace{frame: band.frame, region: &upper,
		z0: band.interfaceZ, z1: band.interfaceZ, outward: false, role: "bossShellBand"}
	copyBP := bp
	copyBP.faces = append([]brepFace(nil), bp.faces...)
	copyBP.faces[band.ledgeFace] = fake
	copyBP.loopBands = append(append([]brepLoopBand(nil), bp.loopBands...), b)
	embeds, err := brepEmbeds(bp.faces)
	if err != nil {
		return nil, err
	}
	patches, _, err := attachFilletBand(ctx, body, ref, copyBP, bi, open, embeds[band.ledgeFace], freeform.NewFreeformWork())
	return patches, err
}

// measureBossShell integrates its three cavity intervals exactly, with pi
// enclosed by proofbound's rational interval. Face areas already carry each
// planar or cylindrical patch's own independent bound.
func measureBossShell(ctx context.Context, bp brepPayload, body *Body) error {
	if bp.bossShell.round != nil {
		return measureRoundBossShell(ctx, bp, body)
	}
	b := bp.bossShell
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
	h := sub(point(b.interfaceZ), point(b.z0))
	hb := sub(point(b.topZ), point(b.interfaceZ))
	bw, bh := point(b.boss.width()), point(b.boss.height())
	plateV, bossV := mul(area(b.plate), h), mul(area(b.boss), hb)
	lowerV := mul(area(b.lower), sub(h, scale(t, 2, 1)))
	upperV := mul(area(b.upper), hb)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	widths := add(bw, bh)
	bandV := add(
		add(sub(mul(area(b.boss), t), scale(mul(widths, t2), 2, 1)),
			scale(mul(mul(widths, pi), t2), 1, 2)),
		sub(scale(t3, 20, 3), scale(mul(pi, t3), 2, 1)),
	)
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: boss shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.volume = Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: exactnessOf(ve)}
	// z is measured from the recorded frame origin, not from the plate base.
	plateCZ := scale(add(point(b.z0), point(b.interfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.interfaceZ), point(b.topZ)), 1, 2)
	bandZ := add(mul(point(b.lowerZ), bandV),
		add(sub(scale(mul(area(b.boss), t2), 1, 2), scale(mul(widths, t3), 1, 3)), scale(t4, 1, 3)))
	mu := sub(sub(add(mul(plateV, centerU(b.plate)), mul(bossV, centerU(b.boss))),
		add(mul(lowerV, centerU(b.lower)), mul(upperV, centerU(b.upper)))), mul(bandV, centerU(b.boss)))
	mv := sub(sub(add(mul(plateV, centerV(b.plate)), mul(bossV, centerV(b.boss))),
		add(mul(lowerV, centerV(b.lower)), mul(upperV, centerV(b.upper)))), mul(bandV, centerV(b.boss)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, plateCZ), mul(upperV, bossCZ))), bandZ)
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
	cb := prismPointBound(ref, c[0], c[1], c[2])
	body.centroid = VecMeasurement{Value: ref.point(c[0].Value, c[1].Value, c[2].Value), Bound: units.Millimeters(cb), Exactness: exactnessOf(cb)}
	var a proofbound.BoundedScalar
	for _, f := range body.Faces() {
		a = proofbound.BoundedAdd(a, proofbound.MeasuredScalar(f.area, f.areaBound))
	}
	body.area = Measurement{Value: units.SquareMillimeters(a.Value), Bound: units.SquareMillimeters(a.Bound), Exactness: exactnessOf(a.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.bounds = box
	return validateAnalyticBodyMeasurements(body)
}
