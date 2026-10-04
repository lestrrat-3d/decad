package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// orientedSourceBox is the exact parallelotope obtained by applying the read
// placement and pose entries to a source-certified rectangular prism.
type orientedSourceBox struct {
	corner [8]dyV3
	edge   [3]dyV3
}

func sourceOrientedBoxAtPose(body *Body, pose r3.Transform) (orientedSourceBox, bool) {
	pp, ok := body.payload.(prismPayload)
	if !ok || !body.solid || body.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!rectangularProfile(pp.profile) ||
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !pose.IsValid() || !finiteVec(pose.Translation()) ||
		!finiteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) {
		return orientedSourceBox{}, false
	}
	var umin, umax, vmin, vmax float64
	for i, segment := range pp.profile.Outer.Segments {
		line, ok := segment.(LineSeg)
		if !ok {
			return orientedSourceBox{}, false
		}
		point := line.Start
		if i == 0 {
			umin, umax, vmin, vmax = point.U, point.U, point.V, point.V
		}
		umin, umax = math.Min(umin, point.U), math.Max(umax, point.U)
		vmin, vmax = math.Min(vmin, point.V), math.Max(vmax, point.V)
	}
	if !finiteMeasurementValues(umin, umax, vmin, vmax) ||
		umin >= umax || vmin >= vmax || pp.z0 >= pp.z1 {
		return orientedSourceBox{}, false
	}
	values := [3][2]dyadic{{mustDyOf(umin), mustDyOf(umax)},
		{mustDyOf(vmin), mustDyOf(vmax)}, {mustDyOf(pp.z0), mustDyOf(pp.z1)}}
	frame := [3]dyV3{dyVec(pp.frame.U()), dyVec(pp.frame.V()), dyVec(pp.frame.N())}
	origin := dyVec(pp.frame.Origin())
	var box orientedSourceBox
	for index := range box.corner {
		point := origin
		for axis := range 3 {
			point = dvAdd(point, dyScaleVec(frame[axis], values[axis][(index>>axis)&1]))
		}
		box.corner[index] = exactContactTransform(pose, exactContactTransform(pp.xform, point))
	}
	box.edge = [3]dyV3{dvSub(box.corner[1], box.corner[0]),
		dvSub(box.corner[2], box.corner[0]), dvSub(box.corner[4], box.corner[0])}
	for axis := range 3 {
		if dvIsZero(box.edge[axis]) {
			return orientedSourceBox{}, false
		}
	}
	return box, true
}

// orientedBoxRelation applies the complete separating-axis test. Every axis
// and projection is a polynomial of held float entries, hence exact dyadic.
func orientedBoxRelation(a, b orientedSourceBox) (ContactRelation, dyadic, dyadic) {
	faceAxes := func(box orientedSourceBox) [3]dyV3 {
		return [3]dyV3{dvCross(box.edge[1], box.edge[2]),
			dvCross(box.edge[2], box.edge[0]), dvCross(box.edge[0], box.edge[1])}
	}
	axisA, axisB := faceAxes(a), faceAxes(b)
	axes := make([]dyV3, 0, 15)
	axes = append(axes, axisA[:]...)
	axes = append(axes, axisB[:]...)
	for _, ea := range a.edge {
		for _, eb := range b.edge {
			axes = append(axes, dvCross(ea, eb))
		}
	}
	touch := false
	bestGap, bestNormSquared := dyZero(), dyZero()
	for _, axis := range axes {
		if dvIsZero(axis) {
			continue
		}
		alo, ahi := orientedProjection(a, axis)
		blo, bhi := orientedProjection(b, axis)
		gap := dyZero()
		if dyCmp(ahi, blo) < 0 {
			gap = dySubScalar(blo, ahi)
		} else if dyCmp(bhi, alo) < 0 {
			gap = dySubScalar(alo, bhi)
		} else if dyCmp(ahi, blo) == 0 || dyCmp(bhi, alo) == 0 {
			touch = true
		}
		if gap.sign() <= 0 {
			continue
		}
		normSquared := dvDot(axis, axis)
		if bestGap.sign() == 0 ||
			new(big.Rat).Quo(dyMul(gap, gap).rat(), normSquared.rat()).Cmp(
				new(big.Rat).Quo(dyMul(bestGap, bestGap).rat(), bestNormSquared.rat())) > 0 {
			bestGap, bestNormSquared = gap, normSquared
		}
	}
	if bestGap.sign() > 0 {
		return ContactSeparated, bestGap, bestNormSquared
	}
	if touch {
		return ContactTouching, dyZero(), dyZero()
	}
	return ContactOverlapping, dyZero(), dyZero()
}

func orientedProjection(box orientedSourceBox, axis dyV3) (dyadic, dyadic) {
	lo, hi := dvDot(box.corner[0], axis), dvDot(box.corner[0], axis)
	for _, point := range box.corner[1:] {
		value := dvDot(point, axis)
		lo, hi = dyMin(lo, value), dyMax(hi, value)
	}
	return lo, hi
}

func classifyOrientedSourceBoxes(report *ContactReport, a, b orientedSourceBox) {
	relation, gap, normSquared := orientedBoxRelation(a, b)
	switch relation {
	case ContactTouching:
		report.Relation = relation
		report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Reason = ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, a, b)
		}
	case ContactOverlapping:
		report.Relation, report.Reason = relation, ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, a, b)
		}
	case ContactSeparated:
		reading, ok := orientedBoxGap(a, b, gap, normSquared)
		if !ok {
			report.Reason = ContactNoGapProof
			return
		}
		report.Relation, report.Gap = relation, &reading
	}
}

// publishContainedHorizontalPatch handles a complete rotating box face
// inside an axis-aligned box face. Exact corner and side-plane comparisons
// certify the four-point patch, including a uniquely shallow penetration.
func publishContainedHorizontalPatch(report *ContactReport, a, b orientedSourceBox) {
	if base, ok := sourceBoxAtPose(report.A, report.PoseA); ok &&
		publishHorizontalPatchOrder(report, base, b, report.B, report.PoseB, true) {
		return
	}
	if base, ok := sourceBoxAtPose(report.B, report.PoseB); ok {
		publishHorizontalPatchOrder(report, base, a, report.A, report.PoseA, false)
	}
}

func publishHorizontalPatchOrder(report *ContactReport, base sourceBoxContactProof,
	rotated orientedSourceBox, body *Body, pose r3.Transform, baseIsA bool) bool {
	basis := pose.Basis()
	if basis.EX.Z != 0 || basis.EY.Z != 0 || basis.EZ != (r3.Vec{Z: 1}) {
		return false
	}
	source, ok := sourceBoxAtPose(body, r3.Identity())
	if !ok {
		return false
	}
	minZ, maxZ := orientedProjection(rotated, dyV3{dyZero(), dyZero(), mustDyOf(1)})
	baseBelow := dyCmp(base.lo[2], minZ) < 0 && dyCmp(base.hi[2], maxZ) < 0
	baseAbove := dyCmp(base.lo[2], minZ) > 0 && dyCmp(base.hi[2], maxZ) > 0
	if !baseBelow && !baseAbove {
		return false
	}
	var faceZ, supportZ, separation dyadic
	var baseSide, rotatedSide int
	if baseBelow {
		faceZ, supportZ = minZ, base.hi[2]
		separation = dySubScalar(faceZ, supportZ)
		baseSide, rotatedSide = 1, 0
	} else {
		faceZ, supportZ = maxZ, base.lo[2]
		separation = dySubScalar(supportZ, faceZ)
		baseSide, rotatedSide = 0, 1
	}
	if separation.sign() > 0 ||
		(report.Relation == ContactTouching && separation.sign() != 0) ||
		(report.Relation == ContactOverlapping && separation.sign() >= 0) {
		return false
	}
	depth := dyNeg(separation)
	vertical := -1
	for axis, edge := range rotated.edge {
		if edge[0].sign() == 0 && edge[1].sign() == 0 && edge[2].sign() != 0 {
			if vertical >= 0 {
				return false
			}
			vertical = axis
		}
	}
	if vertical < 0 {
		return false
	}
	start := 0
	if dyCmp(rotated.corner[0][2], faceZ) != 0 {
		start = 1 << vertical
	}
	i, j := (vertical+1)%3, (vertical+2)%3
	indices := [4]int{start, start | (1 << i), start | (1 << i) | (1 << j), start | (1 << j)}
	var corners [4]dyV3
	for n, index := range indices {
		corner := rotated.corner[index]
		if dyCmp(corner[2], faceZ) != 0 {
			return false
		}
		for axis := range 2 {
			if dyCmp(corner[axis], dyAdd(base.lo[axis], depth)) <= 0 ||
				dyCmp(corner[axis], dySubScalar(base.hi[axis], depth)) >= 0 {
				return false
			}
		}
		corners[n] = corner
	}
	if report.Relation == ContactOverlapping &&
		((baseBelow && dyCmp(maxZ, base.hi[2]) <= 0) ||
			(baseAbove && dyCmp(minZ, base.lo[2]) >= 0)) {
		return false
	}
	reading, ok := sourceBoxSignedReading(separation)
	if !ok {
		return false
	}
	normalZ := 1.0
	if baseAbove {
		normalZ = -1
	}
	if !baseIsA {
		normalZ = -normalZ
	}
	faceBase, faceRotated := base.faces[2][baseSide], source.faces[2][rotatedSide]
	points := make([]ContactPoint, 0, len(corners))
	for _, corner := range corners {
		basePoint := corner
		basePoint[2] = supportZ
		baseReading, okBase := sourceBoxPoint(basePoint)
		rotatedReading, okRotated := sourceBoxPoint(corner)
		if !okBase || !okRotated ||
			baseReading.Bound.Base() > report.Request.PointResolution.Base() ||
			rotatedReading.Bound.Base() > report.Request.PointResolution.Base() {
			return false
		}
		point := ContactPoint{Normal: VecMeasurement{Value: r3.Vec{Z: normalZ},
			Exactness: Exact, Bound: units.Scalar(0)}, NormalAngle: units.Radians(0),
			Separation: reading}
		if baseIsA {
			point.OnA, point.OnB = baseReading, rotatedReading
			point.FaceA, point.FaceB = faceBase, faceRotated
		} else {
			point.OnA, point.OnB = rotatedReading, baseReading
			point.FaceA, point.FaceB = faceRotated, faceBase
		}
		point.FeatureA, point.FeatureB = ContactFeature{Face: point.FaceA}, ContactFeature{Face: point.FaceB}
		points = append(points, point)
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
	return true
}

// publishOrientedAxisPatch reduces two opposed axis-normal faces to one
// certified interior witness. It leaves other oriented contacts without a
// manifold, including edge and corner contacts.
func publishOrientedAxisPatch(report *ContactReport, a, b orientedSourceBox) {
	var chosen *ContactPoint
	best := dyZero()
	for axis := range 3 {
		for _, sign := range []int{1, -1} {
			sideA, sideB := 1, 0
			if sign < 0 {
				sideA, sideB = 0, 1
			}
			faceA, okA := orientedAxisFace(a, axis, sideA)
			faceB, okB := orientedAxisFace(b, axis, sideB)
			if !okA || !okB {
				continue
			}
			candidate := orientedFaceCenter(faceA)
			if !orientedFaceContainsProjection(faceB, candidate, axis) {
				candidate = orientedFaceCenter(faceB)
				if !orientedFaceContainsProjection(faceA, candidate, axis) {
					continue
				}
			}
			pointA, pointB := candidate, candidate
			pointA[axis], pointB[axis] = faceA.origin[axis], faceB.origin[axis]
			onA, readA := sourceBoxPoint(pointA)
			onB, readB := sourceBoxPoint(pointB)
			if !readA || !readB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
				onB.Bound.Base() > report.Request.PointResolution.Base() {
				continue
			}
			separation := dySubScalar(pointB[axis], pointA[axis])
			if sign < 0 {
				separation = dyNeg(separation)
			}
			if separation.sign() > 0 && report.Relation != ContactSeparated {
				continue
			}
			if report.Relation == ContactTouching && separation.sign() != 0 {
				continue
			}
			if chosen != nil && dyCmp(dyAbs(separation), dyAbs(best)) >= 0 {
				continue
			}
			reading, ok := sourceBoxSignedReading(separation)
			if !ok || reading.Bound.Base() > report.Request.PointResolution.Base() {
				continue
			}
			normal := r3.Vec{}
			switch axis {
			case 0:
				normal.X = float64(sign)
			case 1:
				normal.Y = float64(sign)
			case 2:
				normal.Z = float64(sign)
			}
			fa, fb := orientedSourceFace(report.A, report.PoseA, axis, sideA),
				orientedSourceFace(report.B, report.PoseB, axis, sideB)
			if fa == nil || fb == nil {
				continue
			}
			point := ContactPoint{
				OnA: onA, OnB: onB,
				Normal:      VecMeasurement{Value: normal, Exactness: Exact, Bound: units.Scalar(0)},
				NormalAngle: units.Radians(0), Separation: reading,
				FaceA: fa, FaceB: fb,
				FeatureA: ContactFeature{Face: fa}, FeatureB: ContactFeature{Face: fb},
			}
			chosen, best = &point, separation
		}
	}
	if chosen != nil {
		report.Manifold = &ContactManifold{Points: []ContactPoint{*chosen}}
		report.Reason = ContactNoReason
	}
}

type orientedFace struct {
	origin, u, v dyV3
}

func orientedAxisFace(box orientedSourceBox, axis, side int) (orientedFace, bool) {
	for edgeAxis, edge := range box.edge {
		if edge[axis].sign() == 0 {
			continue
		}
		for other := range 3 {
			if other != axis && edge[other].sign() != 0 {
				return orientedFace{}, false
			}
		}
		others := [2]int{}
		count := 0
		for i := range 3 {
			if i != edgeAxis {
				if box.edge[i][axis].sign() != 0 {
					return orientedFace{}, false
				}
				others[count] = i
				count++
			}
		}
		origin := box.corner[0]
		if (side == 1 && edge[axis].sign() > 0) || (side == 0 && edge[axis].sign() < 0) {
			origin = dvAdd(origin, edge)
		}
		return orientedFace{origin: origin, u: box.edge[others[0]], v: box.edge[others[1]]}, true
	}
	return orientedFace{}, false
}

func orientedFaceCenter(face orientedFace) dyV3 {
	var point dyV3
	for i := range 3 {
		point[i] = dyAdd(face.origin[i], dyShift(dyAdd(face.u[i], face.v[i]), -1))
	}
	return point
}

func orientedFaceContainsProjection(face orientedFace, point dyV3, axis int) bool {
	i, j := (axis+1)%3, (axis+2)%3
	det := dySubScalar(dyMul(face.u[i], face.v[j]), dyMul(face.u[j], face.v[i]))
	if det.sign() == 0 {
		return false
	}
	pi, pj := dySubScalar(point[i], face.origin[i]), dySubScalar(point[j], face.origin[j])
	u := dySubScalar(dyMul(pi, face.v[j]), dyMul(pj, face.v[i]))
	v := dySubScalar(dyMul(face.u[i], pj), dyMul(face.u[j], pi))
	if det.sign() < 0 {
		det, u, v = dyNeg(det), dyNeg(u), dyNeg(v)
	}
	return u.sign() > 0 && v.sign() > 0 && dyCmp(u, det) < 0 && dyCmp(v, det) < 0
}

func orientedSourceFace(body *Body, pose r3.Transform, axis, side int) *Face {
	for _, face := range body.Faces() {
		plane, ok := face.surface.(Plane)
		if !ok || face.normalBound != 0 {
			continue
		}
		normal := plane.Frame.N()
		if face.reversed {
			normal = normal.Scale(-1)
		}
		normal = pose.ApplyDir(normal)
		foundAxis, foundSide, ok := signedAxis(normal)
		if ok && foundAxis == axis && foundSide == side {
			return face
		}
	}
	return nil
}

// orientedBoxGap encloses the true minimum distance. SAT supplies a lower
// bound; actual vertex/face point pairs supply upper bounds.
func orientedBoxGap(a, b orientedSourceBox, gap, normSquared dyadic) (Measurement, bool) {
	normUp := ratSqrtUp(normSquared.rat())
	if !finiteMeasurementValues(normUp) || normUp <= 0 {
		return Measurement{}, false
	}
	lower := new(big.Rat).Quo(gap.rat(), floatRat(normUp))
	if lower.Sign() <= 0 {
		return Measurement{}, false
	}
	var upperSquared *big.Rat
	consider := func(candidate *big.Rat) {
		if candidate != nil && (upperSquared == nil || candidate.Cmp(upperSquared) < 0) {
			upperSquared = candidate
		}
	}
	for _, va := range a.corner {
		for _, vb := range b.corner {
			delta := dvSub(va, vb)
			consider(dvDot(delta, delta).rat())
		}
		for axis := range 3 {
			for side := range 2 {
				consider(orientedVertexFaceDistanceSquared(va, b, axis, side))
			}
		}
	}
	for _, vb := range b.corner {
		for axis := range 3 {
			for side := range 2 {
				consider(orientedVertexFaceDistanceSquared(vb, a, axis, side))
			}
		}
	}
	if upperSquared == nil {
		return Measurement{}, false
	}
	upperFloat := ratSqrtUp(upperSquared)
	if !finiteMeasurementValues(upperFloat) {
		return Measurement{}, false
	}
	upper := floatRat(upperFloat)
	if upper.Cmp(lower) < 0 {
		return Measurement{}, false
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(lower, upper), big.NewRat(2, 1))
	value := ratFloatNearest(mid)
	if !finiteMeasurementValues(value) {
		return Measurement{}, false
	}
	held := floatRat(value)
	deviation := new(big.Rat).Sub(held, lower)
	deviation.Abs(deviation)
	other := new(big.Rat).Sub(upper, held)
	other.Abs(other)
	if other.Cmp(deviation) > 0 {
		deviation = other
	}
	bound := ratFloatUp(deviation)
	if !finiteMeasurementValues(bound) || new(big.Rat).Sub(held, floatRat(bound)).Sign() <= 0 {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessOf(bound)}, true
}

func orientedVertexFaceDistanceSquared(vertex dyV3, box orientedSourceBox, axis, side int) *big.Rat {
	_, distance := orientedVertexFaceFoot(vertex, box, axis, side)
	return distance
}

func orientedVertexFaceFoot(vertex dyV3, box orientedSourceBox, axis, side int) ([3]*big.Rat, *big.Rat) {
	i, j := (axis+1)%3, (axis+2)%3
	face := box.corner[0]
	if side == 1 {
		face = dvAdd(face, box.edge[axis])
	}
	a, b := box.edge[i], box.edge[j]
	normal := dvCross(a, b)
	normSquared := dvDot(normal, normal).rat()
	if normSquared.Sign() == 0 {
		return [3]*big.Rat{}, nil
	}
	w := dvSub(vertex, face)
	distanceNumerator := dvDot(w, normal).rat()
	point := [3]*big.Rat{}
	for k := range 3 {
		point[k] = new(big.Rat).Sub(w[k].rat(),
			new(big.Rat).Quo(new(big.Rat).Mul(normal[k].rat(), distanceNumerator), normSquared))
	}
	dot := func(u, v [3]*big.Rat) *big.Rat {
		result := new(big.Rat)
		for k := range 3 {
			result.Add(result, new(big.Rat).Mul(u[k], v[k]))
		}
		return result
	}
	ar, br := [3]*big.Rat{a[0].rat(), a[1].rat(), a[2].rat()},
		[3]*big.Rat{b[0].rat(), b[1].rat(), b[2].rat()}
	aa, bb, ab := dot(ar, ar), dot(br, br), dot(ar, br)
	det := new(big.Rat).Sub(new(big.Rat).Mul(aa, bb), new(big.Rat).Mul(ab, ab))
	if det.Sign() <= 0 {
		return [3]*big.Rat{}, nil
	}
	pa, pb := dot(point, ar), dot(point, br)
	u := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(pa, bb),
		new(big.Rat).Mul(pb, ab)), det)
	v := new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(pb, aa),
		new(big.Rat).Mul(pa, ab)), det)
	if u.Sign() < 0 || v.Sign() < 0 || u.Cmp(big.NewRat(1, 1)) > 0 || v.Cmp(big.NewRat(1, 1)) > 0 {
		return [3]*big.Rat{}, nil
	}
	var foot [3]*big.Rat
	for k := range 3 {
		foot[k] = new(big.Rat).Add(face[k].rat(), point[k])
	}
	return foot, new(big.Rat).Quo(new(big.Rat).Mul(distanceNumerator, distanceNumerator), normSquared)
}

// An actual point strictly inside both η-eroded read boxes is also inside
// both ideal boxes: every support plane moves by at most its body's η.
func orientedInteriorWitness(a, b orientedSourceBox, etaA, etaB *big.Rat) bool {
	try := func(point [3]*big.Rat) bool {
		return orientedPointInside(a, point, etaA) && orientedPointInside(b, point, etaB)
	}
	center := func(box orientedSourceBox) [3]*big.Rat {
		var result [3]*big.Rat
		for k := range 3 {
			result[k] = new(big.Rat).Add(box.corner[0][k].rat(),
				new(big.Rat).Quo(new(big.Rat).Add(box.edge[0][k].rat(),
					new(big.Rat).Add(box.edge[1][k].rat(), box.edge[2][k].rat())), big.NewRat(2, 1)))
		}
		return result
	}
	ca, cb := center(a), center(b)
	if try(ca) || try(cb) {
		return true
	}
	for _, pair := range [][2]orientedSourceBox{{a, b}, {b, a}} {
		for _, vertex := range orientedWitnessSamples(pair[0]) {
			for axis := range 3 {
				for side := range 2 {
					foot, distance := orientedVertexFaceFoot(vertex, pair[1], axis, side)
					if distance == nil {
						continue
					}
					var candidate [3]*big.Rat
					for k := range 3 {
						candidate[k] = new(big.Rat).Quo(new(big.Rat).Add(vertex[k].rat(), foot[k]),
							big.NewRat(2, 1))
					}
					if try(candidate) {
						return true
					}
				}
			}
		}
	}
	return false
}

func orientedWitnessSamples(box orientedSourceBox) []dyV3 {
	samples := make([]dyV3, 0, 26)
	samples = append(samples, box.corner[:]...)
	average := func(indices ...int) dyV3 {
		var point dyV3
		for axis := range 3 {
			sum := dyZero()
			for _, index := range indices {
				sum = dyAdd(sum, box.corner[index][axis])
			}
			point[axis], _ = dyOfRat(new(big.Rat).Quo(sum.rat(), big.NewRat(int64(len(indices)), 1)))
		}
		return point
	}
	for axis := range 3 {
		for index := range 8 {
			if index&(1<<axis) == 0 {
				samples = append(samples, average(index, index|(1<<axis)))
			}
		}
		for side := range 2 {
			indices := make([]int, 0, 4)
			for index := range 8 {
				if (index>>axis)&1 == side {
					indices = append(indices, index)
				}
			}
			samples = append(samples, average(indices...))
		}
	}
	return samples
}

func orientedPointInside(box orientedSourceBox, point [3]*big.Rat, eta *big.Rat) bool {
	for axis := range 3 {
		i, j := (axis+1)%3, (axis+2)%3
		normal := dvCross(box.edge[i], box.edge[j])
		if dvDot(normal, box.edge[axis]).sign() < 0 {
			for k := range 3 {
				normal[k] = dyNeg(normal[k])
			}
		}
		normUp := ratSqrtUp(dvDot(normal, normal).rat())
		if !finiteMeasurementValues(normUp) || normUp <= 0 {
			return false
		}
		margin := new(big.Rat).Mul(eta, floatRat(normUp))
		for side := range 2 {
			face := box.corner[0]
			signed := normal
			if side == 1 {
				face = dvAdd(face, box.edge[axis])
			} else {
				for k := range 3 {
					signed[k] = dyNeg(signed[k])
				}
			}
			bound := dvDot(signed, face).rat()
			projected := new(big.Rat)
			for k := range 3 {
				projected.Add(projected, new(big.Rat).Mul(signed[k].rat(), point[k]))
			}
			if new(big.Rat).Sub(bound, projected).Cmp(margin) <= 0 {
				return false
			}
		}
	}
	return true
}
