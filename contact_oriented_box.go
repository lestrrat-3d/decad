package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/pair"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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

func (box orientedSourceBox) pairBox() pair.OrientedBox {
	return pair.OrientedBox{Corner: box.corner, Edge: box.edge}
}

func sourceOrientedBoxAtPose(body *Body, pose r3.Transform) (orientedSourceBox, bool) {
	pp, ok := body.payload.(prismPayload)
	if !ok || !body.solid || body.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!rectangularProfile(pp.profile) ||
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !pose.IsValid() || !proofbound.FiniteVec(pose.Translation()) ||
		!proofbound.FiniteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) {
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

// orientedBoxRelation keeps the contact report's relation type at the root.
func orientedBoxRelation(a, b orientedSourceBox) (ContactRelation, proofarith.Dyadic, proofarith.Dyadic) {
	relation, gap, normSquared := pair.OrientedBoxRelation(a.pairBox(), b.pairBox())
	switch relation {
	case pair.Separated:
		return ContactSeparated, gap, normSquared
	case pair.Touching:
		return ContactTouching, gap, normSquared
	case pair.Overlapping:
		return ContactOverlapping, gap, normSquared
	default:
		return ContactUndecided, gap, normSquared
	}
}

func orientedProjection(box orientedSourceBox, axis proofarith.DyV3) (proofarith.Dyadic, proofarith.Dyadic) {
	return pair.OrientedProjection(box.pairBox(), axis)
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

func (face orientedFace) pairFace() pair.OrientedFace {
	return pair.OrientedFace{Origin: face.origin, U: face.u, V: face.v}
}

func orientedAxisFace(box *orientedSourceBox, axis, side int, face *orientedFace) bool {
	geom := box.pairBox()
	var found pair.OrientedFace
	if !pair.OrientedAxisFace(&geom, axis, side, &found) {
		return false
	}
	*face = orientedFace{origin: found.Origin, u: found.U, v: found.V}
	return true
}

func orientedFaceCenter(face *orientedFace, point *proofarith.DyV3) {
	geom := face.pairFace()
	pair.OrientedFaceCenter(&geom, point)
}

func orientedFaceContainsProjection(face *orientedFace, point *proofarith.DyV3, axis int) bool {
	geom := face.pairFace()
	return pair.OrientedFaceContainsProjection(&geom, point, axis)
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
		foundAxis, foundSide, ok := clearance.SignedAxis(normal)
		if ok && foundAxis == axis && foundSide == side {
			return face
		}
	}
	return nil
}

// orientedBoxGap publishes the neutral gap enclosure as a public reading.
func orientedBoxGap(a, b orientedSourceBox, gap, normSquared proofarith.Dyadic) (Measurement, bool) {
	reading, ok := pair.OrientedBoxGap(a.pairBox(), b.pairBox(), gap, normSquared)
	if !ok {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(reading.ValueMM), Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}, true
}

func orientedVertexFaceDistanceSquared(vertex proofarith.DyV3, box orientedSourceBox, axis, side int) *big.Rat {
	return pair.OrientedVertexFaceDistanceSquared(vertex, box.pairBox(), axis, side)
}

func orientedVertexFaceFoot(vertex proofarith.DyV3, box orientedSourceBox, axis, side int) ([3]*big.Rat, *big.Rat) {
	return pair.OrientedVertexFaceFoot(vertex, box.pairBox(), axis, side)
}

// An actual point strictly inside both eroded read boxes is inside both ideal boxes.
func orientedInteriorWitness(a, b orientedSourceBox, etaA, etaB *big.Rat) bool {
	return pair.OrientedInteriorWitness(a.pairBox(), b.pairBox(), etaA, etaB)
}

func orientedWitnessSamples(box orientedSourceBox) []proofarith.DyV3 {
	return pair.OrientedWitnessSamples(box.pairBox())
}
