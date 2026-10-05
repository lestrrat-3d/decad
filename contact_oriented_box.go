package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// orientedSourceBox is the exact parallelotope obtained by applying the read
// placement and pose entries to a source-certified rectangular prism.
type orientedSourceBox struct {
	corner [8]proofarith.DyV3
	edge   [3]proofarith.DyV3
	faces  [3][2]*Face
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
	values := [3][2]proofarith.Dyadic{{proofarith.MustDyOf(umin), proofarith.MustDyOf(umax)},
		{proofarith.MustDyOf(vmin), proofarith.MustDyOf(vmax)}, {proofarith.MustDyOf(pp.z0), proofarith.MustDyOf(pp.z1)}}
	frame := [3]proofarith.DyV3{proofarith.DyVec(pp.frame.U()), proofarith.DyVec(pp.frame.V()), proofarith.DyVec(pp.frame.N())}
	origin := proofarith.DyVec(pp.frame.Origin())
	var box orientedSourceBox
	for index := range box.corner {
		point := origin
		for axis := range 3 {
			point = proofarith.DvAdd(point, dyScaleVec(frame[axis], values[axis][(index>>axis)&1]))
		}
		box.corner[index] = exactContactTransform(pose, exactContactTransform(pp.xform, point))
	}
	box.edge = [3]proofarith.DyV3{proofarith.DvSub(box.corner[1], box.corner[0]),
		proofarith.DvSub(box.corner[2], box.corner[0]), proofarith.DvSub(box.corner[4], box.corner[0])}
	for axis := range 3 {
		if proofarith.DvIsZero(box.edge[axis]) {
			return orientedSourceBox{}, false
		}
	}
	// The original planar faces retain their source identities after the read
	// pose. Match them in the body's cardinal placed frame, before rotation.
	placedEdge := [3]proofarith.DyV3{}
	for axis := range 3 {
		placedEdge[axis] = exactContactTransform(pp.xform,
			dyScaleVec(frame[axis], proofarith.DySubScalar(values[axis][1], values[axis][0])))
		placedEdge[axis] = proofarith.DvSub(placedEdge[axis], proofarith.DyVec(pp.xform.Translation()))
	}
	faces := body.Faces()
	if len(faces) != 6 {
		return orientedSourceBox{}, false
	}
	for _, face := range faces {
		plane, ok := face.surface.(Plane)
		if !ok || face.normalBound != 0 {
			return orientedSourceBox{}, false
		}
		normal := plane.Frame.N()
		if face.reversed {
			normal = normal.Scale(-1)
		}
		mapped := false
		for axis := range 3 {
			projection := proofarith.DvDot(proofarith.DyVec(normal), placedEdge[axis]).Sign()
			if projection == 0 {
				continue
			}
			side := 0
			if projection > 0 {
				side = 1
			}
			if mapped || box.faces[axis][side] != nil {
				return orientedSourceBox{}, false
			}
			box.faces[axis][side], mapped = face, true
		}
		if !mapped {
			return orientedSourceBox{}, false
		}
	}
	for axis := range 3 {
		if box.faces[axis][0] == nil || box.faces[axis][1] == nil {
			return orientedSourceBox{}, false
		}
	}
	return box, true
}

// orientedBoxRelation applies the complete separating-axis test. Every axis
// and projection is a polynomial of held float entries, hence exact dyadic.
func orientedBoxRelation(a, b orientedSourceBox) (ContactRelation, proofarith.Dyadic, proofarith.Dyadic) {
	faceAxes := func(box orientedSourceBox) [3]proofarith.DyV3 {
		return [3]proofarith.DyV3{proofarith.DvCross(box.edge[1], box.edge[2]),
			proofarith.DvCross(box.edge[2], box.edge[0]), proofarith.DvCross(box.edge[0], box.edge[1])}
	}
	axisA, axisB := faceAxes(a), faceAxes(b)
	axes := make([]proofarith.DyV3, 0, 15)
	axes = append(axes, axisA[:]...)
	axes = append(axes, axisB[:]...)
	for _, ea := range a.edge {
		for _, eb := range b.edge {
			axes = append(axes, proofarith.DvCross(ea, eb))
		}
	}
	touch := false
	bestGap, bestNormSquared := proofarith.DyZero(), proofarith.DyZero()
	for _, axis := range axes {
		if proofarith.DvIsZero(axis) {
			continue
		}
		alo, ahi := orientedProjection(a, axis)
		blo, bhi := orientedProjection(b, axis)
		gap := proofarith.DyZero()
		if proofarith.DyCmp(ahi, blo) < 0 {
			gap = proofarith.DySubScalar(blo, ahi)
		} else if proofarith.DyCmp(bhi, alo) < 0 {
			gap = proofarith.DySubScalar(alo, bhi)
		} else if proofarith.DyCmp(ahi, blo) == 0 || proofarith.DyCmp(bhi, alo) == 0 {
			touch = true
		}
		if gap.Sign() <= 0 {
			continue
		}
		normSquared := proofarith.DvDot(axis, axis)
		if bestGap.Sign() == 0 ||
			new(big.Rat).Quo(proofarith.DyMul(gap, gap).Rat(), normSquared.Rat()).Cmp(
				new(big.Rat).Quo(proofarith.DyMul(bestGap, bestGap).Rat(), bestNormSquared.Rat())) > 0 {
			bestGap, bestNormSquared = gap, normSquared
		}
	}
	if bestGap.Sign() > 0 {
		return ContactSeparated, bestGap, bestNormSquared
	}
	if touch {
		return ContactTouching, proofarith.DyZero(), proofarith.DyZero()
	}
	return ContactOverlapping, proofarith.DyZero(), proofarith.DyZero()
}

func orientedProjection(box orientedSourceBox, axis proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
	lo, hi := proofarith.DvDot(box.corner[0], axis), proofarith.DvDot(box.corner[0], axis)
	for _, point := range box.corner[1:] {
		value := proofarith.DvDot(point, axis)
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
			publishClippedHorizontalPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != ContactPointTooCoarse {
			publishOrientedBoxPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != ContactPointTooCoarse {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case ContactOverlapping:
		report.Relation, report.Reason = relation, ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, &a, &b)
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
	minZ, maxZ := orientedProjection(rotated, proofarith.DyV3{proofarith.DyZero(), proofarith.DyZero(), proofarith.MustDyOf(1)})
	baseBelow := proofarith.DyCmp(base.lo[2], minZ) < 0 && proofarith.DyCmp(base.hi[2], maxZ) < 0
	baseAbove := proofarith.DyCmp(base.lo[2], minZ) > 0 && proofarith.DyCmp(base.hi[2], maxZ) > 0
	if !baseBelow && !baseAbove {
		return false
	}
	var faceZ, supportZ, separation proofarith.Dyadic
	var baseSide, rotatedSide int
	if baseBelow {
		faceZ, supportZ = minZ, base.hi[2]
		separation = proofarith.DySubScalar(faceZ, supportZ)
		baseSide, rotatedSide = 1, 0
	} else {
		faceZ, supportZ = maxZ, base.lo[2]
		separation = proofarith.DySubScalar(supportZ, faceZ)
		baseSide, rotatedSide = 0, 1
	}
	if separation.Sign() > 0 ||
		(report.Relation == ContactTouching && separation.Sign() != 0) ||
		(report.Relation == ContactOverlapping && separation.Sign() >= 0) {
		return false
	}
	depth := proofarith.DyNeg(separation)
	vertical := -1
	for axis, edge := range rotated.edge {
		if edge[0].Sign() == 0 && edge[1].Sign() == 0 && edge[2].Sign() != 0 {
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
	if proofarith.DyCmp(rotated.corner[0][2], faceZ) != 0 {
		start = 1 << vertical
	}
	i, j := (vertical+1)%3, (vertical+2)%3
	indices := [4]int{start, start | (1 << i), start | (1 << i) | (1 << j), start | (1 << j)}
	var corners [4]proofarith.DyV3
	for n, index := range indices {
		corner := rotated.corner[index]
		if proofarith.DyCmp(corner[2], faceZ) != 0 {
			return false
		}
		for axis := range 2 {
			if proofarith.DyCmp(corner[axis], proofarith.DyAdd(base.lo[axis], depth)) <= 0 ||
				proofarith.DyCmp(corner[axis], proofarith.DySubScalar(base.hi[axis], depth)) >= 0 {
				return false
			}
		}
		corners[n] = corner
	}
	if report.Relation == ContactOverlapping &&
		((baseBelow && proofarith.DyCmp(maxZ, base.hi[2]) <= 0) ||
			(baseAbove && proofarith.DyCmp(minZ, base.lo[2]) >= 0)) {
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
func publishOrientedAxisPatch(report *ContactReport, a, b *orientedSourceBox) {
	var chosen *ContactPoint
	best := proofarith.DyZero()
	_, alignedA := sourceBoxAtPose(report.A, report.PoseA)
	_, alignedB := sourceBoxAtPose(report.B, report.PoseB)
	for axis := range 3 {
		// A horizontal patch against a signed-axis box needs the complete
		// contained four-point proof above; one interior point cannot replace it.
		if axis == 2 && (alignedA || alignedB) {
			continue
		}
		for _, sign := range []int{1, -1} {
			sideA, sideB := 1, 0
			if sign < 0 {
				sideA, sideB = 0, 1
			}
			var faceA, faceB orientedFace
			if !orientedAxisFace(a, axis, sideA, &faceA) ||
				!orientedAxisFace(b, axis, sideB, &faceB) {
				continue
			}
			var candidate proofarith.DyV3
			orientedFaceCenter(&faceA, &candidate)
			if !orientedFaceContainsProjection(&faceB, &candidate, axis) {
				orientedFaceCenter(&faceB, &candidate)
				if !orientedFaceContainsProjection(&faceA, &candidate, axis) {
					continue
				}
			}
			// Separation depends only on the two support planes. Read it before
			// constructing and rounding the witness points, so the exact proof
			// does not depend on a copied point surviving float publication.
			separation := proofarith.DySubScalar(faceB.origin[axis], faceA.origin[axis])
			if sign < 0 {
				separation = proofarith.DyNeg(separation)
			}
			if separation.Sign() > 0 && report.Relation != ContactSeparated {
				continue
			}
			if report.Relation == ContactTouching && separation.Sign() != 0 {
				continue
			}
			if chosen != nil && proofarith.DyCmp(proofarith.DyAbs(separation), proofarith.DyAbs(best)) >= 0 {
				continue
			}
			reading, ok := sourceBoxSignedReading(separation)
			if !ok || reading.Bound.Base() > report.Request.PointResolution.Base() {
				continue
			}
			var pointA, pointB proofarith.DyV3
			for coordinate := range 3 {
				pointA[coordinate] = candidate[coordinate]
				pointB[coordinate] = candidate[coordinate]
			}
			pointA[axis] = faceA.origin[axis]
			pointB[axis] = faceB.origin[axis]
			onA, readA := sourceBoxPointAt(&pointA)
			onB, readB := sourceBoxPointAt(&pointB)
			if !readA || !readB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
				onB.Bound.Base() > report.Request.PointResolution.Base() {
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
	origin, u, v proofarith.DyV3
}

func orientedAxisFace(box *orientedSourceBox, axis, side int, face *orientedFace) bool {
	for edgeAxis := range box.edge {
		edge := &box.edge[edgeAxis]
		if edge[axis].Sign() == 0 {
			continue
		}
		for other := range 3 {
			if other != axis && edge[other].Sign() != 0 {
				return false
			}
		}
		others := [2]int{}
		count := 0
		for i := range 3 {
			if i != edgeAxis {
				if box.edge[i][axis].Sign() != 0 {
					return false
				}
				others[count] = i
				count++
			}
		}
		shift := (side == 1 && edge[axis].Sign() > 0) ||
			(side == 0 && edge[axis].Sign() < 0)
		for coordinate := range 3 {
			face.origin[coordinate] = box.corner[0][coordinate]
			if shift {
				face.origin[coordinate] = proofarith.DyAdd(face.origin[coordinate], edge[coordinate])
			}
			face.u[coordinate] = box.edge[others[0]][coordinate]
			face.v[coordinate] = box.edge[others[1]][coordinate]
		}
		return true
	}
	return false
}

func orientedFaceCenter(face *orientedFace, point *proofarith.DyV3) {
	for i := range 3 {
		point[i] = proofarith.DyAdd(face.origin[i], proofarith.DyShift(proofarith.DyAdd(face.u[i], face.v[i]), -1))
	}
}

func orientedFaceContainsProjection(face *orientedFace, point *proofarith.DyV3, axis int) bool {
	i, j := (axis+1)%3, (axis+2)%3
	det := proofarith.DySubScalar(proofarith.DyMul(face.u[i], face.v[j]), proofarith.DyMul(face.u[j], face.v[i]))
	if det.Sign() == 0 {
		return false
	}
	pi, pj := proofarith.DySubScalar(point[i], face.origin[i]), proofarith.DySubScalar(point[j], face.origin[j])
	u := proofarith.DySubScalar(proofarith.DyMul(pi, face.v[j]), proofarith.DyMul(pj, face.v[i]))
	v := proofarith.DySubScalar(proofarith.DyMul(face.u[i], pj), proofarith.DyMul(face.u[j], pi))
	if det.Sign() < 0 {
		det, u, v = proofarith.DyNeg(det), proofarith.DyNeg(u), proofarith.DyNeg(v)
	}
	return u.Sign() > 0 && v.Sign() > 0 && proofarith.DyCmp(u, det) < 0 && proofarith.DyCmp(v, det) < 0
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
func orientedBoxGap(a, b orientedSourceBox, gap, normSquared proofarith.Dyadic) (Measurement, bool) {
	normUp := ratSqrtUp(normSquared.Rat())
	if !finiteMeasurementValues(normUp) || normUp <= 0 {
		return Measurement{}, false
	}
	lower := new(big.Rat).Quo(gap.Rat(), proofarith.FloatRat(normUp))
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
			delta := proofarith.DvSub(va, vb)
			consider(proofarith.DvDot(delta, delta).Rat())
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
	upper := proofarith.FloatRat(upperFloat)
	if upper.Cmp(lower) < 0 {
		return Measurement{}, false
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(lower, upper), big.NewRat(2, 1))
	value := ratFloatNearest(mid)
	if !finiteMeasurementValues(value) {
		return Measurement{}, false
	}
	held := proofarith.FloatRat(value)
	deviation := new(big.Rat).Sub(held, lower)
	deviation.Abs(deviation)
	other := new(big.Rat).Sub(upper, held)
	other.Abs(other)
	if other.Cmp(deviation) > 0 {
		deviation = other
	}
	bound := ratFloatUp(deviation)
	if !finiteMeasurementValues(bound) || new(big.Rat).Sub(held, proofarith.FloatRat(bound)).Sign() <= 0 {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessOf(bound)}, true
}

func orientedVertexFaceDistanceSquared(vertex proofarith.DyV3, box orientedSourceBox, axis, side int) *big.Rat {
	_, distance := orientedVertexFaceFoot(vertex, box, axis, side)
	return distance
}

func orientedVertexFaceFoot(vertex proofarith.DyV3, box orientedSourceBox, axis, side int) ([3]*big.Rat, *big.Rat) {
	i, j := (axis+1)%3, (axis+2)%3
	face := box.corner[0]
	if side == 1 {
		face = proofarith.DvAdd(face, box.edge[axis])
	}
	a, b := box.edge[i], box.edge[j]
	normal := proofarith.DvCross(a, b)
	normSquared := proofarith.DvDot(normal, normal).Rat()
	if normSquared.Sign() == 0 {
		return [3]*big.Rat{}, nil
	}
	w := proofarith.DvSub(vertex, face)
	distanceNumerator := proofarith.DvDot(w, normal).Rat()
	point := [3]*big.Rat{}
	for k := range 3 {
		point[k] = new(big.Rat).Sub(w[k].Rat(),
			new(big.Rat).Quo(new(big.Rat).Mul(normal[k].Rat(), distanceNumerator), normSquared))
	}
	dot := func(u, v [3]*big.Rat) *big.Rat {
		result := new(big.Rat)
		for k := range 3 {
			result.Add(result, new(big.Rat).Mul(u[k], v[k]))
		}
		return result
	}
	ar, br := [3]*big.Rat{a[0].Rat(), a[1].Rat(), a[2].Rat()},
		[3]*big.Rat{b[0].Rat(), b[1].Rat(), b[2].Rat()}
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
		foot[k] = new(big.Rat).Add(face[k].Rat(), point[k])
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
			result[k] = new(big.Rat).Add(box.corner[0][k].Rat(),
				new(big.Rat).Quo(new(big.Rat).Add(box.edge[0][k].Rat(),
					new(big.Rat).Add(box.edge[1][k].Rat(), box.edge[2][k].Rat())), big.NewRat(2, 1)))
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
						candidate[k] = new(big.Rat).Quo(new(big.Rat).Add(vertex[k].Rat(), foot[k]),
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

func orientedWitnessSamples(box orientedSourceBox) []proofarith.DyV3 {
	samples := make([]proofarith.DyV3, 0, 26)
	samples = append(samples, box.corner[:]...)
	average := func(indices ...int) proofarith.DyV3 {
		var point proofarith.DyV3
		for axis := range 3 {
			sum := proofarith.DyZero()
			for _, index := range indices {
				sum = proofarith.DyAdd(sum, box.corner[index][axis])
			}
			point[axis], _ = proofarith.DyOfRat(new(big.Rat).Quo(sum.Rat(), big.NewRat(int64(len(indices)), 1)))
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
		normal := proofarith.DvCross(box.edge[i], box.edge[j])
		if proofarith.DvDot(normal, box.edge[axis]).Sign() < 0 {
			for k := range 3 {
				normal[k] = proofarith.DyNeg(normal[k])
			}
		}
		normUp := ratSqrtUp(proofarith.DvDot(normal, normal).Rat())
		if !finiteMeasurementValues(normUp) || normUp <= 0 {
			return false
		}
		margin := new(big.Rat).Mul(eta, proofarith.FloatRat(normUp))
		for side := range 2 {
			face := box.corner[0]
			signed := normal
			if side == 1 {
				face = proofarith.DvAdd(face, box.edge[axis])
			} else {
				for k := range 3 {
					signed[k] = proofarith.DyNeg(signed[k])
				}
			}
			bound := proofarith.DvDot(signed, face).Rat()
			projected := new(big.Rat)
			for k := range 3 {
				projected.Add(projected, new(big.Rat).Mul(signed[k].Rat(), point[k]))
			}
			if new(big.Rat).Sub(bound, projected).Cmp(margin) <= 0 {
				return false
			}
		}
	}
	return true
}
