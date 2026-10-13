package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// counterboreShellRound is the source-authenticated two-level through bore.
// Its shell uses pocketShellBand's whole-circle fillet between the larger
// bore's shoulder and the new cavity wall. sign points from the kept far cap
// toward the removed cap in the producer's frame.
type counterboreShellRound struct {
	small, large       roundBossCircle
	plate, inset       bossRect
	farZ, stepZ, nearZ float64
	sign, t            float64
	center             Point2
	frame              r3.Frame
}

func counterboreHoleCircle(ctx context.Context, p profileRecord) (roundBossCircle, bool, error) {
	if len(p.Holes) != 1 {
		return roundBossCircle{}, false, nil
	}
	loop, err := offset2d.ReverseLoopRecordContext(ctx, p.Holes[0])
	if err != nil {
		return roundBossCircle{}, false, err
	}
	c, ok := recordedBossCircle(profileRecord{Outer: loop})
	return c, ok, nil
}

// shellCounterbore accepts exactly two coaxial whole-circle holes in one
// rectangular plate, with the larger hole at the removed end. It reads only
// the stacked Cut record and refuses any offset it cannot represent exactly.
func shellCounterbore(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.slabs) != 2 || len(sp.interfaces) != 1 ||
		len(sp.slabs[0].Regions) != 1 || len(sp.slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	a, b := sp.slabs[0], sp.slabs[1]
	lo, hi := a.Regions[0], b.Regions[0]
	if !brepgeom.LoopsEqual(lo.Outer, hi.Outer) {
		return nil, false, nil
	}
	plate, ok := recordedBossRect(profileRecord{Outer: lo.Outer})
	if !ok {
		return nil, false, nil
	}
	lower, ok, err := counterboreHoleCircle(ctx, lo)
	if err != nil || !ok {
		return nil, false, err
	}
	upper, ok, err := counterboreHoleCircle(ctx, hi)
	if err != nil || !ok {
		return nil, false, err
	}
	if lower.center != upper.center || lower.radius == upper.radius ||
		call.sense != Inward || len(call.removed) != 1 {
		return nil, false, nil
	}
	top := upper.radius > lower.radius
	roles := facesByRole(source)
	if top && call.removed[0] != roles[roleCapEnd] ||
		!top && call.removed[0] != roles[roleCapStart] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: coaxial counterbore shell %s", ErrUnsupported, why)
	}
	if sp.sectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 ||
		b.Z0Delta != 0 || b.Z1Delta != 0 || a.Z1 != b.Z0 ||
		!(a.Z0 < a.Z1 && b.Z0 < b.Z1) || call.tDelta != 0 {
		return refuse("requires exact slab sections, levels, and thickness")
	}
	small, large := lower, upper
	farZ, nearZ, sign := a.Z0, b.Z1, 1.0
	if !top {
		small, large = upper, lower
		farZ, nearZ, sign = b.Z1, a.Z0, -1
	}
	stepZ, t := a.Z1, call.tmm
	farH, nearH := sign*(stepZ-farZ), sign*(nearZ-stepZ)
	if !(t > 0 && farH > 2*t && nearH > 0 && plate.width() > 2*t && plate.height() > 2*t) {
		return refuse("has insufficient floor, counterbore depth, or outer wall")
	}
	inset := plate.inset(t)
	smallR, largeR := small.radius+t, large.radius+t
	c := small.center
	if !(smallR < large.radius &&
		c.U-largeR > inset.u0 && c.U+largeR < inset.u1 &&
		c.V-largeR > inset.v0 && c.V+largeR < inset.v1) {
		return refuse("has insufficient shoulder or bore-to-wall clearance")
	}
	farInnerZ, capZ := farZ+sign*t, stepZ-sign*t
	if proofbound.ExactSumRound(smallR, small.radius, t) != 0 ||
		proofbound.ExactSumRound(largeR, large.radius, t) != 0 ||
		proofbound.ExactSumRound(farInnerZ, farZ, sign*t) != 0 ||
		proofbound.ExactSumRound(capZ, stepZ, -sign*t) != 0 ||
		proofbound.ExactSumRound(farH, stepZ*sign, -farZ*sign) != 0 ||
		proofbound.ExactSumRound(nearH, nearZ*sign, -stepZ*sign) != 0 ||
		!bossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, radius := range []float64{small.radius, large.radius, smallR, largeR} {
		if proofbound.ExactSumRound(c.U-radius, c.U, -radius) != 0 ||
			proofbound.ExactSumRound(c.U+radius, c.U, radius) != 0 ||
			proofbound.ExactSumRound(c.V-radius, c.V, -radius) != 0 ||
			proofbound.ExactSumRound(c.V+radius, c.V, radius) != 0 {
			return refuse("needs exactly represented circle extents")
		}
	}
	openingZ := nearZ
	openingOutward := sign > 0
	openingIndex := -1
	for i, face := range view.faces {
		if face.planar() && face.outward == openingOutward && face.z0 == openingZ {
			if openingIndex >= 0 {
				return refuse("has more than one opening cap")
			}
			openingIndex = i
		}
	}
	if openingIndex < 0 {
		return refuse("has no opening cap")
	}
	makeCircle := func(orig roundBossCircle, radius float64) (profileRecord, bool) {
		circle, ok := orig.outer.Outer.Segments[0].(circleSeg)
		if !ok {
			return profileRecord{}, false
		}
		circle.Radius = units.Millimeters(radius)
		return profileRecord{Outer: loopRecord{Segments: []curveSegment{circle}}}, true
	}
	smallExpanded, ok := makeCircle(small, smallR)
	if !ok {
		return refuse("has no authenticated smaller circle")
	}
	largeExpanded, ok := makeCircle(large, largeR)
	if !ok {
		return refuse("has no authenticated larger circle")
	}
	result := brepPayload{xform: view.xform, faces: make([]brepFace, 0, len(view.faces)+15)}
	result.faces = append(result.faces, view.faces[:openingIndex]...)
	result.faces = append(result.faces, view.faces[openingIndex+1:]...)
	annulus := func(outer, hole profileRecord) (profileRecord, error) {
		p := outer
		reversed, err := offset2d.ReverseLoopRecordContext(ctx, hole.Outer)
		if err != nil {
			return profileRecord{}, err
		}
		p.Holes = []loopRecord{reversed}
		return p, nil
	}
	addPlane := func(p profileRecord, z float64, outward bool) int {
		result.faces = append(result.faces, brepFace{frame: sp.frame, region: &p, z0: z, z1: z, outward: outward})
		return len(result.faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reverse bool) {
		if z0 > z1 {
			z0, z1 = z1, z0
		}
		for _, seg := range p.Outer.Segments {
			if reverse {
				seg = reverseSegment(seg)
			}
			result.faces = append(result.faces, brepFace{frame: sp.frame, wall: seg, z0: z0, z1: z1})
		}
	}
	floor, err := annulus(inset.profile(), smallExpanded)
	if err != nil {
		return nil, true, err
	}
	addPlane(floor, farInnerZ, openingOutward)
	addWalls(inset.profile(), farInnerZ, nearZ, true)
	outerRim, err := annulus(plate.profile(), inset.profile())
	if err != nil {
		return nil, true, err
	}
	addPlane(outerRim, nearZ, openingOutward)
	addWalls(smallExpanded, farInnerZ, capZ, false)
	under, err := annulus(large.outer, smallExpanded)
	if err != nil {
		return nil, true, err
	}
	floorFace := addPlane(under, capZ, !openingOutward)
	addWalls(largeExpanded, stepZ, nearZ, false)
	innerRim, err := annulus(largeExpanded, large.outer)
	if err != nil {
		return nil, true, err
	}
	addPlane(innerRim, nearZ, openingOutward)
	result.pocketShell = &pocketShellBand{outer: plate, inner: inset, rounded: largeExpanded,
		floorZ: stepZ, lowerZ: capZ, topZ: nearZ, z0: farZ, t: t, floorFace: floorFace, frame: sp.frame,
		round: &counterboreShellRound{small: small, large: large, plate: plate, inset: inset,
			farZ: farZ, stepZ: stepZ, nearZ: nearZ, sign: sign, t: t, center: c, frame: sp.frame}}
	result.assignRoles()
	body, err := evalBrepContext(ctx, source.doc, source.doc.nextProducerID(), result)
	return body, true, err
}

// measureCounterboreShell subtracts the inset rectangular cavity and adds
// the dilation of the source's stepped through bore. The quarter-torus
// transition is integrated as a circular radius over its axial interval.
func measureCounterboreShell(ctx context.Context, bp brepPayload, body *Body) error {
	b := bp.pocketShell.round
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
	square := func(r proofbound.RatInterval) proofbound.RatInterval { return mul(r, r) }
	t, rs, rl := point(b.t), point(b.small.radius), point(b.large.radius)
	rsExpanded, rlExpanded := add(rs, t), add(rl, t)
	farH := point(math.Abs(b.stepZ - b.farZ))
	nearH := point(math.Abs(b.nearZ - b.stepZ))
	h := add(farH, nearH)
	outerV := mul(area(b.plate), h)
	smallHoleV := mul(pi, mul(square(rs), farH))
	largeHoleV := mul(pi, mul(square(rl), nearH))
	sourceV := sub(sub(outerV, smallHoleV), largeHoleV)
	innerV := mul(area(b.inset), sub(h, t))
	lowerH := sub(farH, scale(t, 2, 1))
	lowerV := mul(pi, mul(square(rsExpanded), lowerH))
	upperV := mul(pi, mul(square(rlExpanded), nearH))
	t2, t3, t4 := square(t), mul(square(t), t), square(square(t))
	// At distance v below the shoulder, the expanded hole radius is
	// rl + sqrt(t²-v²), 0 <= v <= t.
	transitionArea := add(add(mul(square(rl), t), scale(mul(mul(rl, pi), t2), 1, 2)), scale(t3, 2, 3))
	transitionV := mul(pi, transitionArea)
	obstacleV := add(add(lowerV, transitionV), upperV)
	volume := add(sub(sourceV, innerV), obstacleV)
	if volume.Lo.Sign() <= 0 {
		return fmt.Errorf("%w: counterbore shell has no proven positive volume", ErrUnsupported)
	}
	v, ve := heldOf(volume)
	body.volume = Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve),
		Exactness: exactnessOf(ve)}
	outerW := mul(outerV, scale(h, 1, 2))
	smallHoleW := mul(smallHoleV, scale(farH, 1, 2))
	largeHoleW := mul(largeHoleV, add(farH, scale(nearH, 1, 2)))
	sourceW := sub(sub(outerW, smallHoleW), largeHoleW)
	innerW := mul(innerV, scale(add(h, t), 1, 2))
	lowerW := mul(lowerV, scale(farH, 1, 2))
	transitionVFirst := add(add(scale(mul(square(rl), t2), 1, 2),
		scale(mul(rl, t3), 2, 3)), scale(t4, 1, 4))
	transitionW := sub(mul(transitionV, farH), mul(pi, transitionVFirst))
	upperW := mul(upperV, add(farH, scale(nearH, 1, 2)))
	mw := add(sub(sourceW, innerW), add(add(lowerW, transitionW), upperW))
	mu := sub(mul(outerV, centerU(b.plate)), mul(innerV, centerU(b.inset)))
	mv := sub(mul(outerV, centerV(b.plate)), mul(innerV, centerV(b.inset)))
	holesV := add(smallHoleV, largeHoleV)
	mu = add(sub(mu, mul(holesV, point(b.center.U))), mul(obstacleV, point(b.center.U)))
	mv = add(sub(mv, mul(holesV, point(b.center.V))), mul(obstacleV, point(b.center.V)))
	mz := add(mul(point(b.farZ), volume), mul(point(b.sign), mw))
	var c [3]proofbound.BoundedScalar
	for i, iv := range []proofbound.RatInterval{mu, mv, mz} {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: counterbore shell centroid has no finite enclosure", ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.refView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.centroid = VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
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
