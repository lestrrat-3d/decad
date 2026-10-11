package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// roundBossCircle is the exact whole-circle variant of bossShellBand's two rings.
// The plate remains rectangular; the single transition is a quarter torus.
type roundBossCircle struct {
	outer, inner profileRecord
	center       Point2
	radius       float64
}

func recordedBossCircle(p profileRecord) (roundBossCircle, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 1 {
		return roundBossCircle{}, false
	}
	c, ok := p.Outer.Segments[0].(circleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 {
		return roundBossCircle{}, false
	}
	r, delta, err := extent.MagnitudeInBounded(c.Radius, units.Length, units.Millimeter, "boss radius")
	if err != nil || delta != 0 || !(r > 0) || proofbound.IsNonFinite(r) ||
		proofbound.IsNonFinite(c.Center.U) || proofbound.IsNonFinite(c.Center.V) {
		return roundBossCircle{}, false
	}
	return roundBossCircle{outer: p, center: c.Center, radius: r}, true
}

// shellRoundBoss admits one nested, concentric whole-circle boss over a
// rectangular plate. Every offset and level must be exactly represented.
func shellRoundBoss(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 ||
		len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	plate, pOK := recordedBossRect(sp.slabs[0].Regions[0])
	circle, cOK := recordedBossCircle(sp.slabs[1].Regions[0])
	if !pOK || !cOK {
		return nil, false, nil
	}
	if call.sense != Inward || len(call.removed) != 1 ||
		call.removed[0] != facesByRole(source)[roleCapEnd] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: round stacked-boss shell %s", ErrUnsupported, why)
	}
	a, b := sp.slabs[0], sp.slabs[1]
	if sp.sectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 ||
		a.Z1 != b.Z0 || !(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	t, r, c := call.tmm, circle.radius, circle.center
	if !(t > 0 && a.Z1-a.Z0 > 2*t && r > t &&
		c.U-r > plate.u0+t && c.U+r < plate.u1-t &&
		c.V-r > plate.v0+t && c.V+r < plate.v1-t) {
		return refuse("has insufficient floor, wall, or root clearance")
	}
	lowerZ, innerR := a.Z1-t, r-t
	if proofbound.ExactSumRound(lowerZ, a.Z1, -t) != 0 ||
		proofbound.ExactSumRound(a.Z0+t, a.Z0, t) != 0 ||
		proofbound.ExactSumRound(innerR, r, -t) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0, a.Z1, -a.Z0) != 0 ||
		proofbound.ExactSumRound(b.Z1-b.Z0, b.Z1, -b.Z0) != 0 ||
		proofbound.ExactSumRound(a.Z1-a.Z0-2*t, a.Z1, -a.Z0, -t, -t) != 0 ||
		!bossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, x := range []float64{c.U, c.V} {
		if proofbound.ExactSumRound(x-r, x, -r) != 0 ||
			proofbound.ExactSumRound(x+r, x, r) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	inner, ok := circle.outer.Outer.Segments[0].(circleSeg)
	if !ok {
		return refuse("has no recorded whole-circle boundary")
	}
	inner.Radius = units.Millimeters(innerR)
	circle.inner = profileRecord{Outer: loopRecord{Segments: []curveSegment{inner}}}
	topCapIdx := -1
	for i, face := range view.faces {
		if face.planar() && face.outward && face.z0 == b.Z1 {
			if topCapIdx >= 0 {
				return refuse("has more than one top cap")
			}
			topCapIdx = i
		}
	}
	if topCapIdx < 0 {
		return refuse("has no top cap")
	}
	result := brepPayload{xform: view.xform, faces: make([]brepFace, 0, len(view.faces)+12)}
	result.faces = append(result.faces, view.faces[:topCapIdx]...)
	result.faces = append(result.faces, view.faces[topCapIdx+1:]...)
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.faces = append(result.faces, brepFace{frame: sp.frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64) {
		for _, seg := range p.Outer.Segments {
			result.faces = append(result.faces, brepFace{frame: sp.frame, wall: reverseSegment(seg), z0: z0, z1: z1})
		}
	}
	lower := plate.inset(t)
	addPlane(lower.profile(), a.Z0+t, true)
	addWalls(lower.profile(), a.Z0+t, lowerZ)
	ledge := lower.profile()
	hole, err := offset2d.ReverseLoopRecordContext(ctx, circle.outer.Outer)
	if err != nil {
		return nil, true, err
	}
	ledge.Holes = []loopRecord{hole}
	ledgeFace := addPlane(ledge, lowerZ, false)
	addWalls(circle.inner, b.Z0, b.Z1)
	rim := circle.outer
	hole, err = offset2d.ReverseLoopRecordContext(ctx, circle.inner.Outer)
	if err != nil {
		return nil, true, err
	}
	rim.Holes = []loopRecord{hole}
	addPlane(rim, b.Z1, true)
	result.bossShell = &bossShellBand{plate: plate, lower: lower, round: &circle,
		z0: a.Z0, interfaceZ: a.Z1, topZ: b.Z1, lowerZ: lowerZ,
		t: t, ledgeFace: ledgeFace, frame: sp.frame}
	result.assignRoles()
	body, err := evalBrepContext(ctx, source.doc, source.doc.nextProducerID(), result)
	return body, true, err
}

// The void below the circular boss's inset wall has radius
// (r-t)+sqrt(t²-u²) at height u above lowerZ. Its area and first height
// moment integrate to the intervals below; pi is enclosed rationally.
func measureRoundBossShell(ctx context.Context, bp brepPayload, body *Body) error {
	b, circle := bp.bossShell, bp.bossShell.round
	point := func(x float64) proofbound.RatInterval { return proofbound.PointInterval(proofarith.FloatRat(x)) }
	add, sub, mul := proofbound.IntervalAdd, proofbound.IntervalSub, proofbound.IntervalMul
	scale := func(a proofbound.RatInterval, n, d int64) proofbound.RatInterval {
		return proofbound.IntervalScale(a, big.NewRat(n, d))
	}
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	area := func(r bossRect) proofbound.RatInterval {
		return mul(sub(point(r.u1), point(r.u0)), sub(point(r.v1), point(r.v0)))
	}
	h := sub(point(b.interfaceZ), point(b.z0))
	hb := sub(point(b.topZ), point(b.interfaceZ))
	t, r := point(b.t), point(circle.radius)
	ri := sub(r, t)
	t2, t3, t4 := mul(t, t), mul(mul(t, t), t), mul(mul(t, t), mul(t, t))
	plateV := mul(area(b.plate), h)
	bossV := mul(pi, mul(mul(r, r), hb))
	lowerV := mul(area(b.lower), sub(h, scale(t, 2, 1)))
	upperV := mul(pi, mul(mul(ri, ri), hb))
	bandAreaIntegral := add(add(mul(mul(ri, ri), t), scale(mul(mul(ri, pi), t2), 1, 2)), scale(t3, 2, 3))
	bandV := mul(pi, bandAreaIntegral)
	volume := sub(sub(add(plateV, bossV), add(lowerV, upperV)), bandV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: round boss shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.volume = Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve), Exactness: exactnessOf(ve)}
	plateCZ := scale(add(point(b.z0), point(b.interfaceZ)), 1, 2)
	bossCZ := scale(add(point(b.interfaceZ), point(b.topZ)), 1, 2)
	bandFirst := add(add(scale(mul(mul(ri, ri), t2), 1, 2), scale(mul(ri, t3), 2, 3)), scale(t4, 1, 4))
	bandZ := add(mul(point(b.lowerZ), bandV), mul(pi, bandFirst))
	plateNet := sub(plateV, lowerV)
	bossNet := sub(sub(bossV, upperV), bandV)
	plateU := scale(add(point(b.plate.u0), point(b.plate.u1)), 1, 2)
	plateVCenter := scale(add(point(b.plate.v0), point(b.plate.v1)), 1, 2)
	mu := add(mul(plateNet, plateU), mul(bossNet, point(circle.center.U)))
	mv := add(mul(plateNet, plateVCenter), mul(bossNet, point(circle.center.V)))
	mz := sub(sub(add(mul(plateV, plateCZ), mul(bossV, bossCZ)),
		add(mul(lowerV, plateCZ), mul(upperV, bossCZ))), bandZ)
	var c [3]proofbound.BoundedScalar
	for i, iv := range []proofbound.RatInterval{mu, mv, mz} {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: round boss centroid has no finite enclosure", ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.refView()
	cb := prismPointBound(ref, c[0], c[1], c[2])
	body.centroid = VecMeasurement{Value: ref.point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: exactnessOf(cb)}
	var surfaceArea proofbound.BoundedScalar
	for _, face := range body.Faces() {
		surfaceArea = proofbound.BoundedAdd(surfaceArea, proofbound.MeasuredScalar(face.area, face.areaBound))
	}
	body.area = Measurement{Value: units.SquareMillimeters(surfaceArea.Value),
		Bound: units.SquareMillimeters(surfaceArea.Bound), Exactness: exactnessOf(surfaceArea.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.bounds = box
	return validateAnalyticBodyMeasurements(body)
}
