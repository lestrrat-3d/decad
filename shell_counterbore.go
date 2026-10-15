package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// shellCounterbore accepts exactly two coaxial whole-circle holes in one
// rectangular plate, with the larger hole at the removed end. It reads only
// the stacked Cut record and refuses any offset it cannot represent exactly.
func shellCounterbore(ctx context.Context, source *Body, sp stackedPrismPayload, view brepPayload,
	call brepShellCall) (*Body, bool, error) {
	if len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 ||
		len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return nil, false, nil
	}
	a, b := sp.Slabs[0], sp.Slabs[1]
	lo, hi := a.Regions[0], b.Regions[0]
	if !brepgeom.LoopsEqual(lo.Outer, hi.Outer) {
		return nil, false, nil
	}
	plate, ok := prismshell.RecordedBossRect(profileRecord{Outer: lo.Outer})
	if !ok {
		return nil, false, nil
	}
	lower, ok, err := prismshell.CounterboreHoleCircle(ctx, lo)
	if err != nil || !ok {
		return nil, false, err
	}
	upper, ok, err := prismshell.CounterboreHoleCircle(ctx, hi)
	if err != nil || !ok {
		return nil, false, err
	}
	if lower.Center != upper.Center || lower.Radius == upper.Radius ||
		call.sense != Inward || len(call.removed) != 1 {
		return nil, false, nil
	}
	top := upper.Radius > lower.Radius
	roles := facesByRole(source)
	if top && call.removed[0] != roles[provenance.RoleCapEnd] ||
		!top && call.removed[0] != roles[provenance.RoleCapStart] {
		return nil, false, nil
	}
	refuse := func(why string) (*Body, bool, error) {
		return nil, true, fmt.Errorf("%w: coaxial counterbore shell %s", ErrUnsupported, why)
	}
	if sp.SectionDelta != 0 || a.Z0Delta != 0 || a.Z1Delta != 0 ||
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
	if !(t > 0 && farH > 2*t && nearH > 0 && plate.Width() > 2*t && plate.Height() > 2*t) {
		return refuse("has insufficient floor, counterbore depth, or outer wall")
	}
	inset := plate.Inset(t)
	smallR, largeR := small.Radius+t, large.Radius+t
	c := small.Center
	if !(smallR < large.Radius &&
		c.U-largeR > inset.U0 && c.U+largeR < inset.U1 &&
		c.V-largeR > inset.V0 && c.V+largeR < inset.V1) {
		return refuse("has insufficient shoulder or bore-to-wall clearance")
	}
	farInnerZ, capZ := farZ+sign*t, stepZ-sign*t
	if proofbound.ExactSumRound(smallR, small.Radius, t) != 0 ||
		proofbound.ExactSumRound(largeR, large.Radius, t) != 0 ||
		proofbound.ExactSumRound(farInnerZ, farZ, sign*t) != 0 ||
		proofbound.ExactSumRound(capZ, stepZ, -sign*t) != 0 ||
		proofbound.ExactSumRound(farH, stepZ*sign, -farZ*sign) != 0 ||
		proofbound.ExactSumRound(nearH, nearZ*sign, -stepZ*sign) != 0 ||
		!bossRectArithmeticExact(plate, t) {
		return refuse("needs exactly represented offset coordinates")
	}
	for _, radius := range []float64{small.Radius, large.Radius, smallR, largeR} {
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
	for i, face := range view.Faces {
		if face.Planar() && face.Outward == openingOutward && face.Z0 == openingZ {
			if openingIndex >= 0 {
				return refuse("has more than one opening cap")
			}
			openingIndex = i
		}
	}
	if openingIndex < 0 {
		return refuse("has no opening cap")
	}
	makeCircle := func(orig prismshell.RoundBossCircle, radius float64) (profileRecord, bool) {
		circle, ok := orig.Outer.Outer.Segments[0].(circleSeg)
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
	result := brepPayload{Xform: view.Xform, Faces: make([]brepFace, 0, len(view.Faces)+15)}
	result.Faces = append(result.Faces, view.Faces[:openingIndex]...)
	result.Faces = append(result.Faces, view.Faces[openingIndex+1:]...)
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
		result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Region: &p, Z0: z, Z1: z, Outward: outward})
		return len(result.Faces) - 1
	}
	addWalls := func(p profileRecord, z0, z1 float64, reverse bool) {
		if z0 > z1 {
			z0, z1 = z1, z0
		}
		for _, seg := range p.Outer.Segments {
			if reverse {
				seg = brepgeom.ReverseSegment(seg)
			}
			result.Faces = append(result.Faces, brepFace{Frame: sp.Frame, Wall: seg, Z0: z0, Z1: z1})
		}
	}
	floor, err := annulus(inset.Profile(), smallExpanded)
	if err != nil {
		return nil, true, err
	}
	addPlane(floor, farInnerZ, openingOutward)
	addWalls(inset.Profile(), farInnerZ, nearZ, true)
	outerRim, err := annulus(plate.Profile(), inset.Profile())
	if err != nil {
		return nil, true, err
	}
	addPlane(outerRim, nearZ, openingOutward)
	addWalls(smallExpanded, farInnerZ, capZ, false)
	under, err := annulus(large.Outer, smallExpanded)
	if err != nil {
		return nil, true, err
	}
	floorFace := addPlane(under, capZ, !openingOutward)
	addWalls(largeExpanded, stepZ, nearZ, false)
	innerRim, err := annulus(largeExpanded, large.Outer)
	if err != nil {
		return nil, true, err
	}
	addPlane(innerRim, nearZ, openingOutward)
	result.PocketShell = &prismshell.PocketShellBand{Outer: plate, Inner: inset, Rounded: largeExpanded,
		FloorZ: stepZ, LowerZ: capZ, TopZ: nearZ, Z0: farZ, T: t, FloorFace: floorFace, Frame: sp.Frame,
		Round: &prismshell.CounterboreShellRound{Small: small, Large: large, Plate: plate, Inset: inset,
			FarZ: farZ, StepZ: stepZ, NearZ: nearZ, Sign: sign, T: t, Center: c, Frame: sp.Frame}}
	result.AssignRoles()
	body, err := evalBrepContext(ctx, documentHandle(source.n.Doc), source.n.Doc.NextProducer, result)
	return body, true, err
}

// measureCounterboreShell subtracts the inset rectangular cavity and adds
// the dilation of the source's stepped through bore. The quarter-torus
// transition is integrated as a circular radius over its axial interval.
func measureCounterboreShell(ctx context.Context, bp brepPayload, body *Body) error {
	b := bp.PocketShell.Round
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
	square := func(r proofbound.RatInterval) proofbound.RatInterval { return mul(r, r) }
	t, rs, rl := point(b.T), point(b.Small.Radius), point(b.Large.Radius)
	rsExpanded, rlExpanded := add(rs, t), add(rl, t)
	farH := point(math.Abs(b.StepZ - b.FarZ))
	nearH := point(math.Abs(b.NearZ - b.StepZ))
	h := add(farH, nearH)
	outerV := mul(area(b.Plate), h)
	smallHoleV := mul(pi, mul(square(rs), farH))
	largeHoleV := mul(pi, mul(square(rl), nearH))
	sourceV := sub(sub(outerV, smallHoleV), largeHoleV)
	innerV := mul(area(b.Inset), sub(h, t))
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
	body.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(ve),
		Exactness: measurement.ExactnessOf(ve)}
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
	mu := sub(mul(outerV, centerU(b.Plate)), mul(innerV, centerU(b.Inset)))
	mv := sub(mul(outerV, centerV(b.Plate)), mul(innerV, centerV(b.Inset)))
	holesV := add(smallHoleV, largeHoleV)
	mu = add(sub(mu, mul(holesV, point(b.Center.U))), mul(obstacleV, point(b.Center.U)))
	mv = add(sub(mv, mul(holesV, point(b.Center.V))), mul(obstacleV, point(b.Center.V)))
	mz := add(mul(point(b.FarZ), volume), mul(point(b.Sign), mw))
	var c [3]proofbound.BoundedScalar
	for i, iv := range []proofbound.RatInterval{mu, mv, mz} {
		q, ok := proofbound.IntervalQuo(iv, volume)
		if !ok {
			return fmt.Errorf("%w: counterbore shell centroid has no finite enclosure", ErrUnsupported)
		}
		value, bound := heldOf(q)
		c[i] = proofbound.MeasuredScalar(value, bound)
	}
	ref := bp.RefView()
	cb := ref.PointBound(c[0], c[1], c[2])
	body.n.Centroid = measurement.VecMeasurement{Value: ref.Point(c[0].Value, c[1].Value, c[2].Value),
		Bound: units.Millimeters(cb), Exactness: measurement.ExactnessOf(cb)}
	var surfaceArea proofbound.BoundedScalar
	for _, face := range body.Faces() {
		surfaceArea = proofbound.BoundedAdd(surfaceArea, proofbound.MeasuredScalar(face.n.Area, face.n.AreaBound))
	}
	body.n.Area = measurement.Measurement{Value: units.SquareMillimeters(surfaceArea.Value),
		Bound: units.SquareMillimeters(surfaceArea.Bound), Exactness: measurement.ExactnessOf(surfaceArea.Bound)}
	box, err := brepBoundsContext(ctx, bp)
	if err != nil {
		return err
	}
	body.n.Bounds = boxToInternal(box)
	return topobuild.ValidateAnalyticMeasurements(bodyNode(body))
}
