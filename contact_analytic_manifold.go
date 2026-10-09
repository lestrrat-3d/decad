package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/placedruling"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rulingSide is one body's share of a certified ruling contact: its original
// face, the exact ruling interval its own occupied set puts on the
// separating plane, and the cylinder radius that scales its normal tilt.
type rulingSide struct {
	face   *Face
	ends   [2]proofarith.DyV3
	radius proofarith.Dyadic // zero for a plane side
	curved bool
}

// publishRulingManifold publishes the manifold of
// docs/contact-geometry-design.md §4.5 from the clearance kernel's ruling
// certificate. Each cylinder side must be a full source cylinder, so its
// occupied set meets the separating plane in exactly its tangent ruling; a
// plane side holds the whole certified ruling inside its trim. The complete
// contact set is then one segment, published as its two ends.
func publishRulingManifold(report *ContactReport, ruling *clearance.RulingContact) {
	outward := ruling.Normal
	sideA, reason := rulingSideOf(report.A, ruling.FaceA, outward, ruling.Offset)
	if reason != ContactNoReason {
		report.Reason = reason
		return
	}
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	sideB, reason := rulingSideOf(report.B, ruling.FaceB, inward, proofarith.DyNeg(ruling.Offset))
	if reason != ContactNoReason {
		report.Reason = reason
		return
	}
	ends, ok := rulingContactSet(sideA, sideB)
	if !ok || !rulingEndsEqual(ends, ruling.Ends) {
		report.Reason = ContactAmbiguousFeature
		return
	}
	var witnesses [2]VecMeasurement
	for i := range ends {
		witness, ok := sourceBoxPointAt(&ends[i])
		if !ok || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return
		}
		witnesses[i] = witness
	}
	points := make([]ContactPoint, 0, len(ends))
	for _, witness := range witnesses {
		normal, angle, ok := rulingNormal(sideA, sideB, witness, outward)
		if !ok || angle.Base() > report.Request.NormalResolution.Base() {
			report.Reason = ContactNoNormalProof
			return
		}
		points = append(points, ContactPoint{
			OnA: witness, OnB: witness, Normal: normal, NormalAngle: angle,
			Separation: Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact},
			FaceA:      sideA.face, FaceB: sideB.face,
			FeatureA: ContactFeature{Face: sideA.face}, FeatureB: ContactFeature{Face: sideB.face},
		})
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
}

// rulingSideOf maps one certified carrier back to its body's original face.
// outward is the side's exact outward normal at the contact and offset the
// separating plane's offset along it.
func rulingSideOf(b *Body, carrier *clearance.CFace, outward proofarith.DyV3,
	offset proofarith.Dyadic) (rulingSide, ContactReason) {
	if carrier.Kind == clearance.CkPlane {
		face, ok := uniquePlaneFace(b, clearance.DyAxisVec(outward), offset)
		if !ok {
			return rulingSide{}, ContactAmbiguousFeature
		}
		return rulingSide{face: face}, ContactNoReason
	}
	cylinder, ok := sourceCylinderAtPose(b, r3.Identity())
	if !ok {
		return rulingSide{}, ContactPayloadUnsupported
	}
	normalAxis, side, ok := clearance.SignedAxis(clearance.DyAxisVec(outward))
	if !ok || normalAxis == cylinder.axis {
		return rulingSide{}, ContactAmbiguousFeature
	}
	var wall *Face
	for _, face := range b.Faces() {
		if _, isCylinder := face.surface.(Cylinder); isCylinder {
			wall = face
		}
	}
	if wall == nil {
		return rulingSide{}, ContactAmbiguousFeature
	}
	// The source box is the exact disk-by-interval box: its transverse
	// extents are the axis coordinate plus or minus the radius.
	box := cylinder.box
	half := proofarith.MustDyOf(.5)
	var point proofarith.DyV3
	var radius proofarith.Dyadic
	for i := range 3 {
		switch i {
		case cylinder.axis:
			continue
		case normalAxis:
			radius = proofarith.DyMul(proofarith.DySubScalar(box.hi[i], box.lo[i]), half)
			point[i] = box.lo[i]
			if side == 1 {
				point[i] = box.hi[i]
			}
		default:
			point[i] = proofarith.DyMul(proofarith.DyAdd(box.lo[i], box.hi[i]), half)
		}
	}
	if proofarith.DyCmp(proofarith.DvDot(outward, point), offset) != 0 {
		return rulingSide{}, ContactAmbiguousFeature
	}
	ends := [2]proofarith.DyV3{point, point}
	ends[0][cylinder.axis], ends[1][cylinder.axis] = box.lo[cylinder.axis], box.hi[cylinder.axis]
	return rulingSide{face: wall, ends: ends, radius: radius, curved: true}, ContactNoReason
}

// uniquePlaneFace finds the one original planar face whose outward normal is
// exactly normal and whose plane lies exactly at offset along it.
func uniquePlaneFace(b *Body, normal r3.Vec, offset proofarith.Dyadic) (*Face, bool) {
	var found *Face
	for _, face := range b.Faces() {
		plane, ok := face.surface.(Plane)
		if !ok {
			continue
		}
		faceNormal := plane.Frame.N()
		if face.reversed {
			faceNormal = faceNormal.Scale(-1)
		}
		origin, okOrigin := clearance.DyVecOf(plane.Frame.Origin())
		if faceNormal != normal || !okOrigin ||
			proofarith.DyCmp(proofarith.DvDot(proofarith.DyVec(normal), origin), offset) != 0 {
			continue
		}
		if found != nil {
			return nil, false
		}
		found = face
	}
	return found, found != nil
}

// rulingContactSet intersects the two sides' rulings. A plane side states no
// ruling of its own: the certificate already put the whole cylinder ruling
// inside its trim.
func rulingContactSet(a, b rulingSide) ([2]proofarith.DyV3, bool) {
	switch {
	case a.curved && b.curved:
		var lo, hi proofarith.DyV3
		for i := range 3 {
			lo[i] = dyMax(dyMin(a.ends[0][i], a.ends[1][i]), dyMin(b.ends[0][i], b.ends[1][i]))
			hi[i] = dyMin(dyMax(a.ends[0][i], a.ends[1][i]), dyMax(b.ends[0][i], b.ends[1][i]))
			if proofarith.DyCmp(lo[i], hi[i]) > 0 {
				return [2]proofarith.DyV3{}, false
			}
		}
		if rulingEndsEqual([2]proofarith.DyV3{lo, lo}, [2]proofarith.DyV3{hi, hi}) {
			return [2]proofarith.DyV3{}, false
		}
		return clearance.OrderedRulingEnds([2]proofarith.DyV3{lo, hi}), true
	case a.curved:
		return clearance.OrderedRulingEnds(a.ends), true
	case b.curved:
		return clearance.OrderedRulingEnds(b.ends), true
	default:
		return [2]proofarith.DyV3{}, false
	}
}

func rulingEndsEqual(a, b [2]proofarith.DyV3) bool {
	for i := range a {
		for k := range 3 {
			if proofarith.DyCmp(a[i][k], b[i][k]) != 0 {
				return false
			}
		}
	}
	return true
}

// rulingNormal reads each side's Face.NormalAt at the witness and publishes
// the tighter ball as the A-to-B normal. A cylinder reading taken at a
// rounded witness also charges the radial tilt to the true contact point:
// two radial vectors whose difference is at most e, the longer of length r,
// have unit directions at most 2e/r apart. The exact separating normal must
// lie in both balls; a ball that misses it refuses the entry.
func rulingNormal(a, b rulingSide, witness VecMeasurement,
	outward proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	normalA, okA := rulingSideNormal(a, witness, outward)
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	normalB, okB := rulingSideNormal(b, witness, inward)
	if !okA || !okB {
		return VecMeasurement{}, units.Value{}, false
	}
	normal := normalA
	if normalB.Bound.Base() < normalA.Bound.Base() {
		normal = VecMeasurement{Value: normalB.Value.Scale(-1), Bound: normalB.Bound,
			Exactness: normalB.Exactness}
	}
	bound := normal.Bound.Base()
	if bound == 0 {
		return normal, units.Radians(0), true
	}
	// A unit vector within bound of the published one is at most 4·bound
	// radians away while bound stays below a half (orientedBoxNormal's rule).
	if bound >= .5 {
		return VecMeasurement{}, units.Value{}, false
	}
	return normal, units.Radians(proofbound.UpRound(4 * bound)), true
}

// rulingSideNormal is one side's outward normal ball at the witness, checked
// to contain the exact outward normal of the certificate.
func rulingSideNormal(side rulingSide, witness VecMeasurement,
	outward proofarith.DyV3) (VecMeasurement, bool) {
	reading, err := side.face.NormalAt(witness.Value)
	if err != nil {
		return VecMeasurement{}, false
	}
	bound := reading.Bound.Base()
	if side.curved && witness.Bound.Base() > 0 {
		tilt := proofbound.DivUpper(2*witness.Bound.Base(), proofarith.DyFloatDown(side.radius))
		bound = proofbound.AbsSumUpper(bound, tilt)
	}
	if !finiteMeasurementValues(bound) {
		return VecMeasurement{}, false
	}
	value, ok := clearance.DyVecOf(reading.Value)
	if !ok {
		return VecMeasurement{}, false
	}
	residual := proofarith.DvSub(value, outward)
	ball := proofarith.MustDyOf(bound)
	if proofarith.DyCmp(proofarith.DvDot(residual, residual), proofarith.DyMul(ball, ball)) > 0 {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Scalar(bound), Exactness: exactnessOf(bound)}, true
}

// placedCylinder is a full source cylinder staged through a query pose
// (docs/contact-geometry-design.md §4.5, placed poses). Its identity record
// gives the body-frame axis, radius, axial length and end-disk centers; the
// pose maps each center exactly. The pose's float basis B is orthonormal only
// to rounding, so the occupied set is the identity cylinder's image under the
// exact float map, whose section is the disk's image under B. gram, the
// largest absolute row sum of BᵀB − I, bounds that departure: every unit m
// has |Bᵀm|² in [1 − gram, 1 + gram].
type placedCylinder struct {
	placedruling.Cylinder
	box    sourceBoxContactProof // the identity disk-by-interval box
	source [2]proofarith.DyV3    // end-disk centers at the identity pose
	wall   *Face
}

// placedCylinderAt reads a full source cylinder at its identity pose and
// stages it through any pose with a positive determinant.
func placedCylinderAt(b *Body, pose r3.Transform) (placedCylinder, bool) {
	source, ok := sourceCylinderAtPose(b, r3.Identity())
	if !ok || !positiveAffine(pose) {
		return placedCylinder{}, false
	}
	var wall *Face
	for _, face := range b.Faces() {
		if _, curved := face.surface.(Cylinder); curved {
			wall = face
		}
	}
	if wall == nil {
		return placedCylinder{}, false
	}
	ends := rollingEnds(source)
	staged := placedruling.Stage(source.axis, source.box.lo, source.box.hi, ends, pose)
	c := placedCylinder{Cylinder: staged, box: source.box, wall: wall, source: ends}
	return c, true
}

// The placed-ruling proof requires gram at most 1/16; its support selection
// checks |α| at most 1/4, under which RimDrift holds.
var rulingGramLimit = proofarith.MustDyOf(1.0 / 16)

// rulingSupport adapts a sweep path to the placed-ruling support proof.
func rulingSupport(c *placedCylinder, S *rotationalSweepPath, poll func() error) (placedruling.Plane, bool, error) {
	return placedruling.Support(c.Cylinder, S.solid, S.startPoints, poll)
}

// classifyPlacedRuling is docs/contact-geometry-design.md §4.5 at placed
// query poses: a full source cylinder M, at any pose with a positive
// determinant, against an exact planar body S with zero held displacement,
// at its own pose. It reports false, leaving report untouched, when the pair
// is not admitted or no relation is proven.
//
// On a plane with all of S behind it, S lies in its vertices' hull behind the
// plane. M's least height above it is the lesser of its end disks', since
// height is affine along the axis, and each disk's is its center's height
// less r·|P·Bᵀn̂|: exactly the lesser H± when the pose is a signed-axis
// permutation, which proves a touch or a gap; otherwise within sectionDrift
// of it. With δ = r·gram + r·α² + |α|·L, L the axial length, a least height
// above δ is a gap and one within δ of zero is a ContactBand of half-width
// c₀ = max|H±| + sectionDrift. The lowest rim points lie within rimDrift of
// c± − r·n̂, whose feet must stay inside S's face with that margin: the
// support plane then holds the whole contact ruling, and the gap is the least
// height.
//
// On a face-local plane (docs/multibody-dynamics-design.md §10.6) S's
// material in front of the plane lies at least the lateral clearance m away
// from the cylinder, so a gap's lower end is the lesser of the least height's
// and m, its upper end unchanged; a touch or band publishes only when m
// exceeds its half-width (clearsBand), and otherwise the pair is Undecided.
func classifyPlacedRuling(ctx context.Context, report *ContactReport) (bool, error) {
	cylinderFirst := true
	bodyM, bodyS, poseS := report.A, report.B, report.PoseB
	cylinder, ok := placedCylinderAt(bodyM, report.PoseA)
	if !ok {
		cylinderFirst, bodyM, bodyS, poseS = false, report.B, report.A, report.PoseA
		if cylinder, ok = placedCylinderAt(bodyM, report.PoseB); !ok {
			return false, nil
		}
	}
	if proofarith.DyCmp(cylinder.Gram, rulingGramLimit) > 0 {
		return false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	solid, delta, ok, err := planarSolidAtPose(ctx, budget, bodyS, poseS, heldChordOf(report.Request))
	if err != nil || !ok || delta.Sign() != 0 {
		return false, err
	}
	pathS := &rotationalSweepPath{body: bodyS, solid: &solid, startPoints: solid.Verts}
	plane, ok, err := rulingSupport(&cylinder, pathS, budget.Step)
	if err != nil || !ok {
		return false, err
	}
	classification, ok := placedruling.Classify(cylinder.Cylinder, plane)
	if !ok {
		return false, nil
	}
	trial := ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	switch classification.Relation {
	case placedruling.Separated:
		gap, ok := ratIntervalMeasurement(classification.GapLo, classification.GapHi)
		if !ok {
			return false, nil
		}
		trial.Relation, trial.Gap = ContactSeparated, &gap
	case placedruling.Touching:
		trial.Relation = ContactTouching
		trial.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		publishPlacedRulingManifold(&trial, &cylinder, &plane, cylinderFirst,
			classification.Lateral, new(big.Rat))
	case placedruling.Band:
		width := classification.BandWidth
		trial.Relation = ContactBand
		trial.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(width),
			Exactness: exactnessFromBound(width)}
		publishPlacedRulingManifold(&trial, &cylinder, &plane, cylinderFirst,
			classification.Lateral, proofarith.FloatRat(width))
	}
	*report = trial
	return true, nil
}

// publishPlacedRulingManifold publishes the two lowest rim points of a placed
// ruling, in the exact coordinate order of c± − r·n̂: each M point is that
// point with a lateral ball, each S point its foot on the plane with the same
// ball, and the normal S's exact face normal oriented from A toward B, which
// is M's own normal at its true lowest point. separation is the band's
// half-width, zero for an exact touch. A point beyond the request withholds
// the manifold with ContactPointTooCoarse.
func publishPlacedRulingManifold(report *ContactReport, c *placedCylinder, plane *placedruling.Plane,
	cylinderFirst bool, lateral, separation *big.Rat) {
	resolution, ok := sweeppath.ExactBaseValue(report.Request.PointResolution)
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	var rims [2]proofarith.DyV3
	for i, center := range c.Centers {
		rims[i] = proofarith.DvSub(center, proofarith.DvScale(plane.Normal, c.Radius))
	}
	if ordered := clearance.OrderedRulingEnds(rims); !proofarith.DvEqual(ordered[0], rims[0]) {
		rims = ordered
	}
	direction := plane.Normal
	if cylinderFirst {
		direction = proofarith.DvSub(proofarith.DyV3{}, direction)
	}
	normal, angle, ok := planarNormal(direction)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = ContactNoNormalProof
		return
	}
	faces := report.A.Faces()
	if cylinderFirst {
		faces = report.B.Faces()
	}
	if plane.Face.ID < 0 || plane.Face.ID >= len(faces) {
		report.Reason = ContactAmbiguousFeature
		return
	}
	faceS := faces[plane.Face.ID]
	bound := proofbound.RatFloatUp(separation)
	points := make([]ContactPoint, 0, len(rims))
	for _, rim := range rims {
		foot := rim
		foot[plane.Axis] = plane.Offset
		onM, okM := planarTrackPoint([3]*big.Rat(ratOfDyV3(rim)), lateral, resolution)
		onS, okS := planarTrackPoint([3]*big.Rat(ratOfDyV3(foot)), lateral, resolution)
		if !okM || !okS {
			report.Reason = ContactPointTooCoarse
			return
		}
		point := ContactPoint{Normal: normal, NormalAngle: angle,
			Separation: Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(bound),
				Exactness: exactnessFromBound(bound)}}
		if cylinderFirst {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = ContactFeature{Face: c.wall}, ContactFeature{Face: faceS}
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = ContactFeature{Face: faceS}, ContactFeature{Face: c.wall}
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
}

// ratIntervalMeasurement publishes an exact interval [lo, hi] as its nearest
// midpoint float and an outward bound covering both ends.
func ratIntervalMeasurement(lo, hi *big.Rat) (Measurement, bool) {
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	value := sweeppath.RatFloatNearest(mid)
	held := proofarith.FloatRat(value)
	spread := proofbound.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(lo, held)), new(big.Rat).Abs(new(big.Rat).Sub(hi, held)))
	bound := proofbound.RatFloatUp(spread)
	if !finiteMeasurementValues(value, bound) {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessFromBound(bound)}, true
}
