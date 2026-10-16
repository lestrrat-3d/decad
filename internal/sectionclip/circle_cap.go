package sectionclip

import (
	"context"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

type circleSeg = sectionrecord.CircleSeg

// planePoint is the evaluator's plane-local coordinate. Point2 is its public
// spelling; both share one underlying struct, so a value assigns either way.
type planePoint = sectionrecord.Point2

// sectionCapFloat accepts only coordinates whose binary64 value is the exact
// rational source coordinate, so Sketch sees the same line as the proof.
func sectionCapFloat(value *big.Rat) (float64, bool) {
	f, _ := value.Float64()
	return f, ObliqueCircleFinite(f) && proofarith.FloatRat(f).Cmp(value) == 0
}

// SectionCircleCapSketch reports whether a private Sketch scene of the source
// circle and the cap line yields exactly one valid trimmed profile on the
// kept side, every edge of it a certified Partial fragment.
func SectionCircleCapSketch(ctx context.Context, circle circleSeg, r, a, b, capLevel *big.Rat, keepBelow bool) bool {
	// Choose an intercept on the stronger coordinate axis. A strict secant
	// places it within sqrt(2) radii of the centre. The endpoints at +/-4
	// direction units therefore span both crossings.
	var anchor, direction SectionRatPoint
	if new(big.Rat).Abs(a).Cmp(new(big.Rat).Abs(b)) >= 0 {
		if a.Sign() == 0 {
			return false
		}
		anchor = SectionRatPoint{SectionRatQuo(capLevel, a), new(big.Rat)}
		direction = SectionRatPoint{new(big.Rat).Neg(SectionRatQuo(b, a)), big.NewRat(1, 1)}
	} else {
		anchor = SectionRatPoint{new(big.Rat), SectionRatQuo(capLevel, b)}
		direction = SectionRatPoint{big.NewRat(1, 1), new(big.Rat).Neg(SectionRatQuo(a, b))}
	}
	points := [2]SectionRatPoint{}
	for i, sign := range [...]int64{-4, 4} {
		coordinate := SectionRatPoint{
			U: SectionRatAdd(anchor.U, SectionRatMul(big.NewRat(sign, 1), direction.U)),
			V: SectionRatAdd(anchor.V, SectionRatMul(big.NewRat(sign, 1), direction.V)),
		}
		points[i] = SectionRatPoint{
			U: SectionRatAdd(proofarith.FloatRat(circle.Center.U), SectionRatMul(r, coordinate.U)),
			V: SectionRatAdd(proofarith.FloatRat(circle.Center.V), SectionRatMul(r, coordinate.V)),
		}
	}
	x0, ok0 := sectionCapFloat(points[0].U)
	y0, ok1 := sectionCapFloat(points[0].V)
	x1, ok2 := sectionCapFloat(points[1].U)
	y1, ok3 := sectionCapFloat(points[1].V)
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
	dx, dy := SectionRatSub(points[1].U, points[0].U), SectionRatSub(points[1].V, points[0].V)
	leftSide := SectionRatSub(SectionRatMul(b, dx), SectionRatMul(a, dy)).Sign()
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

// SectionCircleCapPolygon builds the bounded one-cap oblique section polygon
// of a source circle trimmed by the cap line.
func SectionCircleCapPolygon(center, a, b SectionRatPoint, zCos, zSin, capLevel *big.Rat,
	keepBelow bool, orientation int, plane r3.Frame, perQuadrant int) ([]planePoint, float64, bool) {
	unit := make([]SectionRatPoint, 0, 4*perQuadrant)
	for quadrant := range 4 {
		for j := range perQuadrant {
			unit = append(unit, obliqueCircleUnit(quadrant, j, perQuadrant))
		}
	}
	outer := make([]SectionRatPoint, len(unit))
	for i, p := range unit {
		q := unit[(i+1)%len(unit)]
		dot := SectionRatAdd(SectionRatMul(p.U, q.U), SectionRatMul(p.V, q.V))
		den := SectionRatAdd(big.NewRat(1, 1), dot)
		if den.Sign() <= 0 {
			return nil, 0, false
		}
		outer[i] = SectionRatPoint{
			U: SectionRatQuo(SectionRatAdd(p.U, q.U), den),
			V: SectionRatQuo(SectionRatAdd(p.V, q.V), den),
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
			uGap := proofbound.RatFloatUp(new(big.Rat).Abs(SectionRatSub(point.U, foot.U)))
			vGap := proofbound.RatFloatUp(new(big.Rat).Abs(SectionRatSub(point.V, foot.V)))
			candidate := max(proofbound.AbsSumUpper(uGap, vGap),
				proofbound.AbsSumUpper(proofbound.ProductUpper(uGap, uNorm),
					proofbound.ProductUpper(vGap, vNorm)))
			best = min(best, candidate)
		}
		delta = max(delta, best)
	}
	if !ObliqueCircleFinite(delta) || delta > 1.0/20 {
		return nil, 0, false
	}
	gap := delta
	proxy := make([]planePoint, len(inner))
	for i, point := range inner {
		u, _ := point.U.Float64()
		v, _ := point.V.Float64()
		if !ObliqueCircleFinite(u, v) {
			return nil, 0, false
		}
		proxy[i] = planePoint{U: u, V: v}
	}
	// The outer-to-inner gap and the inner-to-held rounding are additive.
	rounding := float64(0)
	for i, point := range inner {
		uGap := proofarith.RationalFloatError(point.U, proxy[i].U)
		vGap := proofarith.RationalFloatError(point.V, proxy[i].V)
		rounding = max(rounding, proofbound.AbsSumUpper(uGap, vGap),
			proofbound.AbsSumUpper(proofbound.ProductUpper(uGap, uNorm),
				proofbound.ProductUpper(vGap, vNorm)))
	}
	delta = proofbound.AbsSumUpper(gap, rounding)
	if delta <= 0 || delta > 1.0/20 || !ObliqueCircleFinite(delta) ||
		!SectionMatchedConvex(inner, proxy) {
		return nil, 0, false
	}
	for i, point := range inner {
		next := inner[(i+1)%len(inner)]
		edgeL1 := SectionRatAdd(new(big.Rat).Abs(SectionRatSub(next.U, point.U)),
			new(big.Rat).Abs(SectionRatSub(next.V, point.V)))
		if edgeL1.Sign() <= 0 ||
			SectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}

func sectionCapMapPolygon(points []SectionRatPoint, center, a, b SectionRatPoint) []SectionRatPoint {
	result := make([]SectionRatPoint, len(points))
	for i, point := range points {
		result[i] = SectionRatPoint{
			U: SectionRatAdd(center.U, SectionRatAdd(SectionRatMul(a.U, point.U), SectionRatMul(b.U, point.V))),
			V: SectionRatAdd(center.V, SectionRatAdd(SectionRatMul(a.V, point.U), SectionRatMul(b.V, point.V))),
		}
	}
	return result
}

// sectionCapClipPolygon clips a CCW rational convex polygon by one exact
// half-plane. Intersections and duplicate removal are exact.
func sectionCapClipPolygon(points []SectionRatPoint, a, b, capLevel *big.Rat, keepBelow bool) []SectionRatPoint {
	value := func(p SectionRatPoint) *big.Rat {
		v := SectionRatSub(SectionRatAdd(SectionRatMul(a, p.U), SectionRatMul(b, p.V)), capLevel)
		if !keepBelow {
			v.Neg(v)
		}
		return v
	}
	result := make([]SectionRatPoint, 0, len(points)+2)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		vp, vq := value(p), value(q)
		if vp.Sign() <= 0 {
			result = append(result, p)
		}
		if vp.Sign()*vq.Sign() < 0 {
			t := SectionRatQuo(vp, SectionRatSub(vp, vq))
			result = append(result, SectionRatPoint{
				U: SectionRatAdd(p.U, SectionRatMul(t, SectionRatSub(q.U, p.U))),
				V: SectionRatAdd(p.V, SectionRatMul(t, SectionRatSub(q.V, p.V))),
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

func sectionCapSamePoint(a, b SectionRatPoint) bool {
	return a.U.Cmp(b.U) == 0 && a.V.Cmp(b.V) == 0
}

func sectionCapSegmentFoot(p, a, b SectionRatPoint) SectionRatPoint {
	d := SectionRatVector(a, b)
	length2 := SectionRatAdd(SectionRatMul(d.U, d.U), SectionRatMul(d.V, d.V))
	if length2.Sign() == 0 {
		return a
	}
	fromA := SectionRatVector(a, p)
	t := SectionRatQuo(SectionRatAdd(SectionRatMul(fromA.U, d.U), SectionRatMul(fromA.V, d.V)), length2)
	if t.Sign() <= 0 {
		return a
	}
	if t.Cmp(big.NewRat(1, 1)) >= 0 {
		return b
	}
	return SectionRatPoint{
		U: SectionRatAdd(a.U, SectionRatMul(t, d.U)),
		V: SectionRatAdd(a.V, SectionRatMul(t, d.V)),
	}
}

// ObliqueCircleCapClip admits one strictly crossed cap only after Sketch has
// certified the source circle-line trim. Inner and outer rational polygons
// enclose the clipped disk before its affine map into the requested plane.
func ObliqueCircleCapClip(ctx context.Context, pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, 0, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || !ObliqueCircleFinite(radius, circle.Center.U, circle.Center.V,
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
	zCenter := SectionRatQuo(new(big.Rat).Neg(
		proofarith.DvDot(normal, proofarith.DvSub(centerWorld, requestedOrigin)).Rat()), denominator)
	zCos := SectionRatQuo(new(big.Rat).Neg(SectionRatMul(r,
		proofarith.DvDot(normal, radialU).Rat())), denominator)
	zSin := SectionRatQuo(new(big.Rat).Neg(SectionRatMul(r,
		proofarith.DvDot(normal, radialV).Rat())), denominator)
	reach2 := SectionRatAdd(SectionRatMul(zCos, zCos), SectionRatMul(zSin, zSin))
	if reach2.Sign() <= 0 {
		return nil, 0, false
	}
	z0, z1 := proofarith.FloatRat(pp.Z0), proofarith.FloatRat(pp.Z1)
	low, high := SectionRatSub(z0, zCenter), SectionRatSub(z1, zCenter)
	lowCross := SectionRatMul(low, low).Cmp(reach2) < 0
	highCross := SectionRatMul(high, high).Cmp(reach2) < 0
	if lowCross == highCross {
		return nil, 0, false
	}
	// The uncrossed cap must miss even the conservative L1 disk extent.
	reach1 := SectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if highCross && SectionRatSub(zCenter, reach1).Cmp(z0) <= 0 ||
		lowCross && SectionRatAdd(zCenter, reach1).Cmp(z1) >= 0 {
		return nil, 0, false
	}
	capLevel := high
	if lowCross {
		capLevel = low
	}
	if !SectionCircleCapSketch(ctx, circle, r, zCos, zSin, capLevel, highCross) {
		return nil, 0, false
	}
	center := ObliqueCircleWorldAdd(ObliqueCircleWorld(centerWorld),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zCenter))
	cosVector := ObliqueCircleWorldAdd(ObliqueCircleWorldScale(ObliqueCircleWorld(radialU), r),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zCos))
	sinVector := ObliqueCircleWorldAdd(ObliqueCircleWorldScale(ObliqueCircleWorld(radialV), r),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zSin))
	localCenter, ok := SectionPlaneCoordinates(center, plane)
	if !ok {
		return nil, 0, false
	}
	localCos, ok := SectionPlaneCoordinates(ObliqueCircleWorldAdd(center, cosVector), plane)
	if !ok {
		return nil, 0, false
	}
	localSin, ok := SectionPlaneCoordinates(ObliqueCircleWorldAdd(center, sinVector), plane)
	if !ok {
		return nil, 0, false
	}
	a, b := SectionRatVector(localCenter, localCos), SectionRatVector(localCenter, localSin)
	orientation := SectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		if err := ctx.Err(); err != nil {
			return nil, 0, false
		}
		polygon, delta, ok := SectionCircleCapPolygon(localCenter, a, b, zCos, zSin,
			capLevel, highCross, orientation, plane, perQuadrant)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}
