package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/pair/planar"

	"github.com/lestrrat-3d/decad/internal/survey2d"

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
	axis    int // the identity-frame axis index
	radius  proofarith.Dyadic
	length  proofarith.Dyadic
	box     sourceBoxContactProof // the identity disk-by-interval box
	source  [2]proofarith.DyV3    // end-disk centers at the identity pose
	centers [2]proofarith.DyV3    // the same centers under the pose
	columns [3]proofarith.DyV3    // the pose's basis columns
	gram    proofarith.Dyadic
	wall    *Face
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
	transverse := (source.axis + 1) % 3
	c := placedCylinder{axis: source.axis, box: source.box, wall: wall, source: rollingEnds(source),
		radius: proofarith.DyShift(proofarith.DySubScalar(source.box.hi[transverse], source.box.lo[transverse]), -1),
		length: proofarith.DySubScalar(source.box.hi[source.axis], source.box.lo[source.axis])}
	basis := pose.Basis()
	c.columns = [3]proofarith.DyV3{proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)}
	for i := range c.source {
		c.centers[i] = exactContactTransform(pose, c.source[i])
	}
	one := proofarith.DyInt(1)
	for i := range 3 {
		row := proofarith.DyZero()
		for j := range 3 {
			entry := proofarith.DvDot(c.columns[i], c.columns[j])
			if i == j {
				entry = proofarith.DySubScalar(entry, one)
			}
			row = proofarith.DyAdd(row, proofarith.DyAbs(entry))
		}
		c.gram = dyMax(c.gram, row)
	}
	return c, true
}

// sectionDrift is r·gram + r·α², the bound on how far the staged section's
// least height along a unit normal n̂ departs from its center's height less
// r, with α = n̂·Bâ the normal component of the staged axis column: that least
// height is r·|P·Bᵀn̂| below the center, P removing the identity axis, and
// |P·Bᵀn̂|² = |Bᵀn̂|² − α² lies in [1 − gram − α², 1 + gram], so
// |r − r·|P·Bᵀn̂|| <= r·(gram + α²).
func (c *placedCylinder) sectionDrift(alpha proofarith.Dyadic) proofarith.Dyadic {
	return proofarith.DyMul(c.radius, proofarith.DyAdd(c.gram, proofarith.DyMul(alpha, alpha)))
}

// rimDrift is r·(3·gram + (3/2)·|α|): how far the true lowest point of a
// staged end disk lies from its center less r·n̂, while gram <= 1/16 and
// |α| <= 1/4. With A the staged basis, y = P·Aᵀn̂ and AAᵀ = I + E, ‖E‖ <= gram,
// the lowest point is c − r·Ay/|y| and Ay = n̂ + E·n̂ − α·Aâ, so
// |Ay − |y|·n̂| <= (gram + α²) + gram + |α|·√(1 + gram), and dividing by
// |y| >= √(1 − gram − α²) >= √(7/8) leaves at most 3·gram + (3/2)·|α|.
func (c *placedCylinder) rimDrift(alpha *big.Rat) *big.Rat {
	return proofbound.RatMul(c.radius.Rat(), proofbound.RatAdd(proofbound.RatMul(big.NewRat(3, 1), c.gram.Rat()),
		proofbound.RatMul(big.NewRat(3, 2), new(big.Rat).Abs(alpha))))
}

// stagedCorners are the eight corners of the cylinder's identity
// disk-by-interval box under the pose, whose hull holds the staged cylinder.
// The box's center maps to the midpoint of the staged end centers and each
// half extent along a basis column, so every corner is exact.
func (c *placedCylinder) stagedCorners() [8]proofarith.DyV3 {
	var mid proofarith.DyV3
	var extent [3]proofarith.Dyadic
	for k := range 3 {
		mid[k] = proofarith.DyShift(proofarith.DyAdd(c.centers[0][k], c.centers[1][k]), -1)
		extent[k] = proofarith.DyShift(proofarith.DySubScalar(c.box.hi[k], c.box.lo[k]), -1)
	}
	var out [8]proofarith.DyV3
	for i := range out {
		corner := mid
		for k := range 3 {
			step := extent[k]
			if i&(1<<k) == 0 {
				step = proofarith.DyNeg(step)
			}
			corner = proofarith.DvAdd(corner, dyScaleVec(c.columns[k], step))
		}
		out[i] = corner
	}
	return out
}

// rulingPlane is the face plane of an exact planar body S that a placed
// cylinder rests on. A face-local plane (docs/multibody-dynamics-design.md
// §10.6) has S material in front of it; clearance is then the lateral
// clearance m of the staged corner box at f = 0, a lower bound on the
// distance from the cylinder to that material, and nil for a plane with all
// of S behind it.
type rulingPlane struct {
	normal    proofarith.DyV3   // n̂, S's unit outward normal there, a signed axis
	axis      int               // n̂'s axis
	origin    int               // an S vertex on the plane
	offset    proofarith.Dyadic // the plane's coordinate on n̂'s axis
	face      planarFace
	alpha     proofarith.Dyadic    // α = n̂·Bâ, the staged axis column's normal component
	heights   [2]proofarith.Dyadic // H± = n̂·(c± − q) − r, each end center's height less r
	local     bool                 // an S vertex lies strictly in front of the plane
	clearance *big.Rat
}

// rulingPlaneLimits are the gates every placed ruling reads: gram at most
// 1/16 and |α| at most 1/4, under which rimDrift holds.
var (
	rulingGramLimit  = proofarith.MustDyOf(1.0 / 16)
	rulingAlphaLimit = proofarith.MustDyOf(1.0 / 4)
)

// rulingSupport picks the plane a placed cylinder rests on: among S's face
// planes whose outward normal is a signed axis, with |α| <= 1/4 and S's
// triangles on it belonging to one face, the one whose larger |H±| is least,
// the first in S's triangle order on a tie. A plane with an S vertex strictly
// in front of it is face-local (§10.6): it is admitted only when the column
// test holds at f = 0 over the coordinate box of the staged corners, and it
// records that box's lateral clearance. S is a path whose startPoints are its
// staged vertices.
func rulingSupport(c *placedCylinder, S *rotationalSweepPath, poll func() error) (rulingPlane, bool, error) {
	var best rulingPlane
	var bestScore proofarith.Dyadic
	found := false
	var tried []planarSupport
	for _, tri := range S.solid.Tris {
		if err := poll(); err != nil {
			return rulingPlane{}, false, err
		}
		origin := S.startPoints[tri[0]]
		n := proofarith.DvCross(proofarith.DvSub(S.startPoints[tri[1]], origin),
			proofarith.DvSub(S.startPoints[tri[2]], origin))
		axis, unit, ok := dyUnitAxis(n)
		if !ok || planarPlaneTried(tried, unit, origin) {
			continue
		}
		tried = append(tried, planarSupport{normal: unit, origin: origin})
		alpha := proofarith.DvDot(unit, c.columns[c.axis])
		if proofarith.DyCmp(proofarith.DyAbs(alpha), rulingAlphaLimit) > 0 {
			continue
		}
		plane := rulingPlane{normal: unit, axis: axis, origin: tri[0], offset: origin[axis], alpha: alpha}
		score := proofarith.DyZero()
		for i, center := range c.centers {
			plane.heights[i] = proofarith.DySubScalar(proofarith.DvDot(unit, proofarith.DvSub(center, origin)), c.radius)
			score = dyMax(score, proofarith.DyAbs(plane.heights[i]))
		}
		if found && proofarith.DyCmp(score, bestScore) >= 0 {
			continue
		}
		face, ok := planarSupportFace(&planarSupport{normal: unit, origin: origin, pathS: S})
		if !ok {
			continue
		}
		plane.face = face
		for _, v := range S.startPoints {
			if proofarith.DvDot(unit, proofarith.DvSub(v, origin)).Sign() > 0 {
				plane.local = true
				break
			}
		}
		if plane.local {
			clearance, apart, err := rulingColumn(c, S, unit, origin, poll)
			if err != nil {
				return rulingPlane{}, false, err
			}
			if !apart {
				continue
			}
			plane.clearance = clearance
		}
		best, bestScore, found = plane, score, true
	}
	return best, found, nil
}

// rulingColumn is §10.6's column test at f = 0 for a placed cylinder: every
// S triangle with a vertex strictly in front of the plane through origin
// must project apart from the coordinate box of the staged corners, whose
// hull holds the cylinder. clearance is that box's lateral clearance, nil for
// an unbounded one (planar.PlanarColumnClear).
func rulingColumn(c *placedCylinder, S *rotationalSweepPath, unit, origin proofarith.DyV3,
	poll func() error) (*big.Rat, bool, error) {
	var lo, hi [3]*big.Rat
	for i, corner := range c.stagedCorners() {
		for k := range 3 {
			v := corner[k].Rat()
			if i == 0 || v.Cmp(lo[k]) < 0 {
				lo[k] = v
			}
			if i == 0 || v.Cmp(hi[k]) > 0 {
				hi[k] = v
			}
		}
	}
	solid := planar.PlanarSolid{Verts: S.startPoints, Tris: S.solid.Tris}
	return planar.PlanarColumnClear(&solid, unit, origin, lo, hi, poll)
}

// holdsBox reports whether a closed box in the face's projected coordinates
// lies inside the face: it meets no bounding edge and has a corner inside one
// of the face's triangles.
func (face *planarFace) holdsBox(lo, hi [2]*big.Rat) bool {
	inside := false
	for _, tri := range face.tris {
		if planarPointInTriangle(lo, tri) {
			inside = true
			break
		}
	}
	if !inside {
		return false
	}
	for _, edge := range face.edges {
		if planarSegmentMeetsBox(edge[0], edge[1], lo, hi) {
			return false
		}
	}
	return true
}

// clearsBand reports whether a touch or band of half-width w on the plane is
// the pair's only contact: on a face-local plane the lateral clearance must
// exceed w, so no material of S in front of the plane lies within the band; a
// plane with all of S behind it, or an unbounded clearance, always clears.
func (p *rulingPlane) clearsBand(w *big.Rat) bool {
	return p.clearance == nil || p.clearance.Cmp(w) > 0
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
	if proofarith.DyCmp(cylinder.gram, rulingGramLimit) > 0 {
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
	lateral := new(big.Rat)
	exact := cylinder.gram.Sign() == 0 && plane.alpha.Sign() == 0
	if !exact {
		lateral = cylinder.rimDrift(plane.alpha.Rat())
	}
	if !placedRulingFootInside(&cylinder, &plane, lateral) {
		return false, nil
	}
	low := dyMin(plane.heights[0], plane.heights[1])
	// The least height is low + r·(1 − ρ), ρ = |P·Bᵀn̂|, exactly low when the
	// pose is exact.
	sigmaLo, sigmaHi := low.Rat(), low.Rat()
	slack := new(big.Rat)
	if !exact {
		var rhoSquared proofarith.Dyadic
		for k := range 3 {
			if k != cylinder.axis {
				component := proofarith.DvDot(plane.normal, cylinder.columns[k])
				rhoSquared = proofarith.DyAdd(rhoSquared, proofarith.DyMul(component, component))
			}
		}
		rhoLo, rhoHi := proofarith.DySqrtDown(rhoSquared), proofarith.DySqrtUp(rhoSquared)
		if !finiteMeasurementValues(rhoLo, rhoHi) {
			return false, nil
		}
		r := cylinder.radius.Rat()
		sigmaLo = new(big.Rat).Sub(proofbound.RatAdd(sigmaLo, r), proofbound.RatMul(r, proofarith.FloatRat(rhoHi)))
		sigmaHi = new(big.Rat).Sub(proofbound.RatAdd(sigmaHi, r), proofbound.RatMul(r, proofarith.FloatRat(rhoLo)))
		slack = proofbound.RatAdd(cylinder.sectionDrift(plane.alpha).Rat(),
			proofbound.RatMul(new(big.Rat).Abs(plane.alpha.Rat()), cylinder.length.Rat()))
	}
	trial := ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	switch {
	case sigmaLo.Cmp(slack) > 0:
		// On a face-local plane the material in front lies at least the
		// clearance away, so the gap's lower end is the lesser of the two;
		// its upper end stays the least height, since the ruling's feet lie
		// inside the face and S has material under the cylinder there.
		if plane.clearance != nil {
			sigmaLo = survey2d.RatMin(sigmaLo, plane.clearance)
		}
		gap, ok := ratIntervalMeasurement(sigmaLo, sigmaHi)
		if !ok {
			return false, nil
		}
		trial.Relation, trial.Gap = ContactSeparated, &gap
	case sigmaHi.Cmp(new(big.Rat).Neg(slack)) < 0:
		return false, nil
	case exact:
		if !plane.clearsBand(new(big.Rat)) {
			return false, nil
		}
		trial.Relation = ContactTouching
		trial.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		publishPlacedRulingManifold(&trial, &cylinder, &plane, cylinderFirst, lateral, new(big.Rat))
	default:
		band := proofbound.RatAdd(dyMax(proofarith.DyAbs(plane.heights[0]), proofarith.DyAbs(plane.heights[1])).Rat(),
			cylinder.sectionDrift(plane.alpha).Rat())
		width := proofbound.RatFloatUp(band)
		if !finiteMeasurementValues(width) || !plane.clearsBand(proofarith.FloatRat(width)) {
			return false, nil
		}
		trial.Relation = ContactBand
		trial.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(width),
			Exactness: exactnessFromBound(width)}
		publishPlacedRulingManifold(&trial, &cylinder, &plane, cylinderFirst, lateral, proofarith.FloatRat(width))
	}
	*report = trial
	return true, nil
}

// placedRulingFootInside checks that the feet of both rims' lowest points, the
// points c± − r·n̂ grown by lateral, lie inside S's face. n̂ is a signed axis,
// so dropping its axis projects onto the plane exactly.
func placedRulingFootInside(c *placedCylinder, plane *rulingPlane, lateral *big.Rat) bool {
	var lo, hi [2]*big.Rat
	for slot, axis := range [2]int{(plane.face.drop + 1) % 3, (plane.face.drop + 2) % 3} {
		a, b := c.centers[0][axis].Rat(), c.centers[1][axis].Rat()
		lo[slot] = new(big.Rat).Sub(survey2d.RatMin(a, b), lateral)
		hi[slot] = new(big.Rat).Add(survey2d.RatMax(a, b), lateral)
	}
	return plane.face.holdsBox(lo, hi)
}

// publishPlacedRulingManifold publishes the two lowest rim points of a placed
// ruling, in the exact coordinate order of c± − r·n̂: each M point is that
// point with a lateral ball, each S point its foot on the plane with the same
// ball, and the normal S's exact face normal oriented from A toward B, which
// is M's own normal at its true lowest point. separation is the band's
// half-width, zero for an exact touch. A point beyond the request withholds
// the manifold with ContactPointTooCoarse.
func publishPlacedRulingManifold(report *ContactReport, c *placedCylinder, plane *rulingPlane,
	cylinderFirst bool, lateral, separation *big.Rat) {
	resolution, ok := exactBaseValue(report.Request.PointResolution)
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	var rims [2]proofarith.DyV3
	for i, center := range c.centers {
		rims[i] = proofarith.DvSub(center, dyScaleVec(plane.normal, c.radius))
	}
	if ordered := clearance.OrderedRulingEnds(rims); !sameDyV3(ordered[0], rims[0]) {
		rims = ordered
	}
	direction := plane.normal
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
	if plane.face.id < 0 || plane.face.id >= len(faces) {
		report.Reason = ContactAmbiguousFeature
		return
	}
	faceS := faces[plane.face.id]
	bound := proofbound.RatFloatUp(separation)
	points := make([]ContactPoint, 0, len(rims))
	for _, rim := range rims {
		foot := rim
		foot[plane.axis] = plane.offset
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
	value := ratFloatNearest(mid)
	held := proofarith.FloatRat(value)
	spread := survey2d.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(lo, held)), new(big.Rat).Abs(new(big.Rat).Sub(hi, held)))
	bound := proofbound.RatFloatUp(spread)
	if !finiteMeasurementValues(value, bound) {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: exactnessFromBound(bound)}, true
}
