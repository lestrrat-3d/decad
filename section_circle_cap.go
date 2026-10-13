package decad

import (
	"context"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// obliqueCircleCapClip admits one strictly crossed cap only after Sketch has
// certified the source circle-line trim. Inner and outer rational polygons
// enclose the clipped disk before its affine map into the requested plane.
func obliqueCircleCapClip(ctx context.Context, pp prismPayload, plane r3.Frame) ([]Point2, float64, bool) {
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, 0, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || !obliqueCircleFinite(radius, circle.Center.U, circle.Center.V,
		pp.Z0, pp.Z1) {
		return nil, 0, false
	}
	sourceOrigin := proofarith.DyVec(pp.Frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.Xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.Xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.Frame.U()), image(pp.Frame.V()), image(pp.Frame.N())
	centerLocal := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
		proofarith.DvScale(proofarith.DyVec(pp.Frame.U()), proofarith.DyOfFinite(circle.Center.U)),
		proofarith.DvScale(proofarith.DyVec(pp.Frame.V()), proofarith.DyOfFinite(circle.Center.V))))
	centerWorld := proofarith.DvTransform(pp.Xform, centerLocal)
	requestedOrigin := proofarith.DyVec(plane.Origin())
	normal := proofarith.DvCross(proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()))
	denominator := proofarith.DvDot(normal, axis).Rat()
	if denominator.Sign() == 0 {
		return nil, 0, false
	}
	r := proofarith.FloatRat(radius)
	zCenter := sectionRatQuo(new(big.Rat).Neg(
		proofarith.DvDot(normal, proofarith.DvSub(centerWorld, requestedOrigin)).Rat()), denominator)
	zCos := sectionRatQuo(new(big.Rat).Neg(sectionRatMul(r,
		proofarith.DvDot(normal, radialU).Rat())), denominator)
	zSin := sectionRatQuo(new(big.Rat).Neg(sectionRatMul(r,
		proofarith.DvDot(normal, radialV).Rat())), denominator)
	reach2 := sectionRatAdd(sectionRatMul(zCos, zCos), sectionRatMul(zSin, zSin))
	if reach2.Sign() <= 0 {
		return nil, 0, false
	}
	z0, z1 := proofarith.FloatRat(pp.Z0), proofarith.FloatRat(pp.Z1)
	low, high := sectionRatSub(z0, zCenter), sectionRatSub(z1, zCenter)
	lowCross := sectionRatMul(low, low).Cmp(reach2) < 0
	highCross := sectionRatMul(high, high).Cmp(reach2) < 0
	if lowCross == highCross {
		return nil, 0, false
	}
	// The uncrossed cap must miss even the conservative L1 disk extent.
	reach1 := sectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if highCross && sectionRatSub(zCenter, reach1).Cmp(z0) <= 0 ||
		lowCross && sectionRatAdd(zCenter, reach1).Cmp(z1) >= 0 {
		return nil, 0, false
	}
	capLevel := high
	if lowCross {
		capLevel = low
	}
	if !sectionCircleCapSketch(ctx, circle, r, zCos, zSin, capLevel, highCross) {
		return nil, 0, false
	}
	center := obliqueCircleWorldAdd(obliqueCircleWorld(centerWorld),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zCenter))
	cosVector := obliqueCircleWorldAdd(obliqueCircleWorldScale(obliqueCircleWorld(radialU), r),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zCos))
	sinVector := obliqueCircleWorldAdd(obliqueCircleWorldScale(obliqueCircleWorld(radialV), r),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zSin))
	localCenter, ok := sectionPlaneCoordinates(center, plane)
	if !ok {
		return nil, 0, false
	}
	localCos, ok := sectionPlaneCoordinates(obliqueCircleWorldAdd(center, cosVector), plane)
	if !ok {
		return nil, 0, false
	}
	localSin, ok := sectionPlaneCoordinates(obliqueCircleWorldAdd(center, sinVector), plane)
	if !ok {
		return nil, 0, false
	}
	a, b := sectionRatVector(localCenter, localCos), sectionRatVector(localCenter, localSin)
	orientation := sectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		if err := ctx.Err(); err != nil {
			return nil, 0, false
		}
		polygon, delta, ok := sectionCircleCapPolygon(localCenter, a, b, zCos, zSin,
			capLevel, highCross, orientation, plane, perQuadrant)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}

// sectionCapFloat accepts only coordinates whose binary64 value is the exact
// rational source coordinate, so Sketch sees the same line as the proof.
func sectionCapFloat(value *big.Rat) (float64, bool) {
	f, _ := value.Float64()
	return f, obliqueCircleFinite(f) && proofarith.FloatRat(f).Cmp(value) == 0
}

func sectionCircleCapSketch(ctx context.Context, circle circleSeg, r, a, b, capLevel *big.Rat, keepBelow bool) bool {
	// Choose an intercept on the stronger coordinate axis. A strict secant
	// places it within sqrt(2) radii of the centre. The endpoints at +/-4
	// direction units therefore span both crossings.
	var anchor, direction sectionRatPoint
	if new(big.Rat).Abs(a).Cmp(new(big.Rat).Abs(b)) >= 0 {
		if a.Sign() == 0 {
			return false
		}
		anchor = sectionRatPoint{sectionRatQuo(capLevel, a), new(big.Rat)}
		direction = sectionRatPoint{new(big.Rat).Neg(sectionRatQuo(b, a)), big.NewRat(1, 1)}
	} else {
		anchor = sectionRatPoint{new(big.Rat), sectionRatQuo(capLevel, b)}
		direction = sectionRatPoint{big.NewRat(1, 1), new(big.Rat).Neg(sectionRatQuo(a, b))}
	}
	points := [2]sectionRatPoint{}
	for i, sign := range [...]int64{-4, 4} {
		coordinate := sectionRatPoint{
			u: sectionRatAdd(anchor.u, sectionRatMul(big.NewRat(sign, 1), direction.u)),
			v: sectionRatAdd(anchor.v, sectionRatMul(big.NewRat(sign, 1), direction.v)),
		}
		points[i] = sectionRatPoint{
			u: sectionRatAdd(proofarith.FloatRat(circle.Center.U), sectionRatMul(r, coordinate.u)),
			v: sectionRatAdd(proofarith.FloatRat(circle.Center.V), sectionRatMul(r, coordinate.v)),
		}
	}
	x0, ok0 := sectionCapFloat(points[0].u)
	y0, ok1 := sectionCapFloat(points[0].v)
	x1, ok2 := sectionCapFloat(points[1].u)
	y1, ok3 := sectionCapFloat(points[1].v)
	if !ok0 || !ok1 || !ok2 || !ok3 {
		return false
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return false
	}
	center := s.CreateReferencePoint(circle.Center.U, circle.Center.V, "decad.section.source")
	radius, exact := sectionCapFloat(r)
	if !exact {
		return false
	}
	sourceCircle, err := s.CreateReferenceCircle(center, radius, "decad.section.source")
	if err != nil {
		return false
	}
	p0 := s.CreateReferencePoint(x0, y0, "decad.section.cap")
	p1 := s.CreateReferencePoint(x1, y1, "decad.section.cap")
	line, err := s.CreateReferenceLine(p0, p1, "decad.section.cap")
	if err != nil {
		return false
	}
	dx, dy := sectionRatSub(points[1].u, points[0].u), sectionRatSub(points[1].v, points[0].v)
	leftSide := sectionRatSub(sectionRatMul(b, dx), sectionRatMul(a, dy)).Sign()
	if leftSide == 0 {
		return false
	}
	selected := 0
	for _, profile := range s.Profiles() {
		if err := ctx.Err(); err != nil {
			return false
		}
		if !profile.Valid || len(profile.Holes) != 0 {
			continue
		}
		lineCount, circleCount, valid := 0, 0, true
		for _, edge := range profile.Outer {
			if !edge.Partial || !edge.TExact {
				valid = false
				break
			}
			switch edge.Entity {
			case line:
				lineCount++
				side := leftSide
				if edge.Reversed {
					side = -side
				}
				if (side < 0) != keepBelow {
					valid = false
				}
			case sourceCircle:
				circleCount++
			default:
				valid = false
			}
		}
		if valid && lineCount == 1 && circleCount >= 1 {
			selected++
		}
	}
	return selected == 1
}

func sectionCircleCapPolygon(center, a, b sectionRatPoint, zCos, zSin, capLevel *big.Rat,
	keepBelow bool, orientation int, plane r3.Frame, perQuadrant int) ([]Point2, float64, bool) {
	unit := make([]sectionRatPoint, 0, 4*perQuadrant)
	for quadrant := range 4 {
		for j := range perQuadrant {
			unit = append(unit, obliqueCircleUnit(quadrant, j, perQuadrant))
		}
	}
	outer := make([]sectionRatPoint, len(unit))
	for i, p := range unit {
		q := unit[(i+1)%len(unit)]
		dot := sectionRatAdd(sectionRatMul(p.u, q.u), sectionRatMul(p.v, q.v))
		den := sectionRatAdd(big.NewRat(1, 1), dot)
		if den.Sign() <= 0 {
			return nil, 0, false
		}
		outer[i] = sectionRatPoint{
			u: sectionRatQuo(sectionRatAdd(p.u, q.u), den),
			v: sectionRatQuo(sectionRatAdd(p.v, q.v), den),
		}
	}
	inner := sectionCapClipPolygon(unit, zCos, zSin, capLevel, keepBelow)
	outer = sectionCapClipPolygon(outer, zCos, zSin, capLevel, keepBelow)
	if len(inner) < 3 || len(outer) < 3 {
		return nil, 0, false
	}
	if orientation < 0 {
		slices.Reverse(inner)
		slices.Reverse(outer)
	}
	inner = sectionCapMapPolygon(inner, center, a, b)
	outer = sectionCapMapPolygon(outer, center, a, b)
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	// Inner is contained in the true clipped ellipse, which is contained in
	// outer. Convexity makes the largest outer-vertex distance a bound for
	// every point of outer, in both the local and held world norms.
	delta := float64(0)
	for _, point := range outer {
		best := math.Inf(1)
		for i := range inner {
			foot := sectionCapSegmentFoot(point, inner[i], inner[(i+1)%len(inner)])
			uGap := proofbound.RatFloatUp(new(big.Rat).Abs(sectionRatSub(point.u, foot.u)))
			vGap := proofbound.RatFloatUp(new(big.Rat).Abs(sectionRatSub(point.v, foot.v)))
			candidate := max(proofbound.AbsSumUpper(uGap, vGap),
				proofbound.AbsSumUpper(proofbound.ProductUpper(uGap, uNorm),
					proofbound.ProductUpper(vGap, vNorm)))
			best = min(best, candidate)
		}
		delta = max(delta, best)
	}
	if !obliqueCircleFinite(delta) || delta > 1.0/20 {
		return nil, 0, false
	}
	gap := delta
	proxy := make([]Point2, len(inner))
	for i, point := range inner {
		u, _ := point.u.Float64()
		v, _ := point.v.Float64()
		if !obliqueCircleFinite(u, v) {
			return nil, 0, false
		}
		proxy[i] = Point2{U: u, V: v}
	}
	// The outer-to-inner gap and the inner-to-held rounding are additive.
	rounding := float64(0)
	for i, point := range inner {
		uGap := proofarith.RationalFloatError(point.u, proxy[i].U)
		vGap := proofarith.RationalFloatError(point.v, proxy[i].V)
		rounding = max(rounding, proofbound.AbsSumUpper(uGap, vGap),
			proofbound.AbsSumUpper(proofbound.ProductUpper(uGap, uNorm),
				proofbound.ProductUpper(vGap, vNorm)))
	}
	delta = proofbound.AbsSumUpper(gap, rounding)
	if delta <= 0 || delta > 1.0/20 || !obliqueCircleFinite(delta) ||
		!sectionMatchedConvex(inner, proxy) {
		return nil, 0, false
	}
	for i, point := range inner {
		next := inner[(i+1)%len(inner)]
		edgeL1 := sectionRatAdd(new(big.Rat).Abs(sectionRatSub(next.u, point.u)),
			new(big.Rat).Abs(sectionRatSub(next.v, point.v)))
		if edgeL1.Sign() <= 0 ||
			sectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}

func sectionCapMapPolygon(points []sectionRatPoint, center, a, b sectionRatPoint) []sectionRatPoint {
	result := make([]sectionRatPoint, len(points))
	for i, point := range points {
		result[i] = sectionRatPoint{
			u: sectionRatAdd(center.u, sectionRatAdd(sectionRatMul(a.u, point.u), sectionRatMul(b.u, point.v))),
			v: sectionRatAdd(center.v, sectionRatAdd(sectionRatMul(a.v, point.u), sectionRatMul(b.v, point.v))),
		}
	}
	return result
}

// sectionCapClipPolygon clips a CCW rational convex polygon by one exact
// half-plane. Intersections and duplicate removal are exact.
func sectionCapClipPolygon(points []sectionRatPoint, a, b, capLevel *big.Rat, keepBelow bool) []sectionRatPoint {
	value := func(p sectionRatPoint) *big.Rat {
		v := sectionRatSub(sectionRatAdd(sectionRatMul(a, p.u), sectionRatMul(b, p.v)), capLevel)
		if !keepBelow {
			v.Neg(v)
		}
		return v
	}
	result := make([]sectionRatPoint, 0, len(points)+2)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		vp, vq := value(p), value(q)
		if vp.Sign() <= 0 {
			result = append(result, p)
		}
		if vp.Sign()*vq.Sign() < 0 {
			t := sectionRatQuo(vp, sectionRatSub(vp, vq))
			result = append(result, sectionRatPoint{
				u: sectionRatAdd(p.u, sectionRatMul(t, sectionRatSub(q.u, p.u))),
				v: sectionRatAdd(p.v, sectionRatMul(t, sectionRatSub(q.v, p.v))),
			})
		}
	}
	unique := result[:0]
	for _, point := range result {
		if len(unique) == 0 || !sectionCapSamePoint(unique[len(unique)-1], point) {
			unique = append(unique, point)
		}
	}
	if len(unique) > 1 && sectionCapSamePoint(unique[0], unique[len(unique)-1]) {
		unique = unique[:len(unique)-1]
	}
	return unique
}

func sectionCapSamePoint(a, b sectionRatPoint) bool {
	return a.u.Cmp(b.u) == 0 && a.v.Cmp(b.v) == 0
}

func sectionCapSegmentFoot(p, a, b sectionRatPoint) sectionRatPoint {
	d := sectionRatVector(a, b)
	length2 := sectionRatAdd(sectionRatMul(d.u, d.u), sectionRatMul(d.v, d.v))
	if length2.Sign() == 0 {
		return a
	}
	fromA := sectionRatVector(a, p)
	t := sectionRatQuo(sectionRatAdd(sectionRatMul(fromA.u, d.u), sectionRatMul(fromA.v, d.v)), length2)
	if t.Sign() <= 0 {
		return a
	}
	if t.Cmp(big.NewRat(1, 1)) >= 0 {
		return b
	}
	return sectionRatPoint{
		u: sectionRatAdd(a.u, sectionRatMul(t, d.u)),
		v: sectionRatAdd(a.v, sectionRatMul(t, d.v)),
	}
}
