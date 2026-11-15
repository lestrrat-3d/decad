package placed

import (
	"context"
	"math/big"

	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/placedruling"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/planarsweep"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PlacedCylinder is a full source cylinder staged through a query pose
// (agent-docs/contact-geometry-design.md §4.5, placed poses). Its identity record
// gives the body-frame axis, radius, axial length and end-disk centers; the
// pose maps each center exactly. The pose's float basis B is orthonormal only
// to rounding, so the occupied set is the identity cylinder's image under the
// exact float map, whose section is the disk's image under B. gram, the
// largest absolute row sum of BᵀB − I, bounds that departure: every unit m
// has |Bᵀm|² in [1 − gram, 1 + gram].
type PlacedCylinder struct {
	placedruling.Cylinder
	Source [2]proofarith.DyV3 // end-disk centers at the identity pose
	Wall   *topo.Face
}

// PlacedCylinderAt reads a full source cylinder at its identity pose and
// stages it through any pose with a positive determinant.
func PlacedCylinderAt(b *topo.Body, pose r3.Transform) (PlacedCylinder, bool) {
	source, ok := contactround.CylinderAtPose(b, r3.Identity())
	if !ok || !planarsnapshot.PositiveAffine(pose) {
		return PlacedCylinder{}, false
	}
	var wall *topo.Face
	for _, face := range topobuild.Faces(b) {
		if _, curved := face.Surface.(surfacegeom.Cylinder); curved {
			wall = face
		}
	}
	if wall == nil {
		return PlacedCylinder{}, false
	}
	ends := ends(source)
	staged := placedruling.Stage(source.Axis, source.Box.Lo, source.Box.Hi, ends, pose)
	c := PlacedCylinder{Cylinder: staged, Wall: wall, Source: ends}
	return c, true
}

// ClassifyPlacedRuling is agent-docs/contact-geometry-design.md §4.5 at placed
// query poses: a full source cylinder M, at any pose with a positive
// determinant, against an exact planar body S with zero held displacement,
// at its own pose. It reports false, leaving report untouched, when the pair
// is not admitted or no relation is proven.
//
// On a plane with all of S behind it, S lies in its vertices' hull behind the
// plane. M's least height above it is the lesser of its end disks', since
// height is affine along the axis, and each disk's is its center's height
// less r·|P·Bᵀn̂|: exactly the lesser H± when the pose is a signed-axis
// permutation, which proves a touch or a gap; otherwise within placedruling.Cylinder.SectionDrift
// of it. With δ = r·gram + r·α² + |α|·L, L the axial length, a least height
// above δ is a gap and one within δ of zero is a ContactBand of half-width
// c₀ = max|H±| + placedruling.Cylinder.SectionDrift. The lowest rim points lie within placedruling.Cylinder.RimDrift of
// c± − r·n̂, whose feet must stay inside S's face with that margin: the
// support plane then holds the whole contact ruling, and the gap is the least
// height.
//
// On a face-local plane (agent-docs/multibody-dynamics-design.md §10.6) S's
// material in front of the plane lies at least the lateral clearance m away
// from the cylinder, so a gap's lower end is the lesser of the least height's
// and m, its upper end unchanged; a touch or band publishes only when m
// exceeds its half-width, and otherwise the pair is Undecided.
func ClassifyPlacedRuling(ctx context.Context, mesher verifiedmesh.Mesher, report *contactReport) (bool, error) {
	cylinderFirst := true
	bodyM, bodyS, poseS := report.A, report.B, report.PoseB
	cylinder, ok := PlacedCylinderAt(bodyM, report.PoseA)
	if !ok {
		cylinderFirst, bodyM, bodyS, poseS = false, report.B, report.A, report.PoseA
		if cylinder, ok = PlacedCylinderAt(bodyM, report.PoseB); !ok {
			return false, nil
		}
	}
	if proofarith.DyCmp(cylinder.Gram, placedruling.GramLimit) > 0 {
		return false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	solid, delta, ok, err := planarpose.SolidAtPose(ctx, budget, mesher, bodyS, poseS, report.Request.HeldChord.Base())
	if err != nil || !ok || delta.Sign() != 0 {
		return false, err
	}
	plane, ok, err := placedruling.Support(cylinder.Cylinder, &solid, solid.Verts, budget.Step)
	if err != nil || !ok {
		return false, err
	}
	classification, ok := placedruling.Classify(cylinder.Cylinder, plane)
	if !ok {
		return false, nil
	}
	trial := contactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	switch classification.Relation {
	case placedruling.Separated:
		gap, ok := pointquery.RatIntervalMeasurement(classification.GapLo, classification.GapHi)
		if !ok {
			return false, nil
		}
		trial.Relation, trial.Gap = vocab.ContactSeparated, &gap
	case placedruling.Touching:
		trial.Relation = vocab.ContactTouching
		trial.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact}
		publishPlacedRulingManifold(&trial, &cylinder, &plane, cylinderFirst,
			classification.Lateral, new(big.Rat))
	case placedruling.Band:
		width := classification.BandWidth
		trial.Relation = vocab.ContactBand
		trial.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(width),
			Exactness: measurement.ExactnessOf(width)}
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
func publishPlacedRulingManifold(report *contactReport, c *PlacedCylinder, plane *placedruling.Plane,
	cylinderFirst bool, lateral, separation *big.Rat) {
	resolution, ok := sweeppath.ExactBaseValue(report.Request.PointResolution)
	if !ok {
		report.Reason = vocab.ContactPointTooCoarse
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
	normal, angle, ok := pairbox.PlanarNormalMeasurement(direction)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = vocab.ContactNoNormalProof
		return
	}
	faces := topobuild.Faces(report.A)
	if cylinderFirst {
		faces = topobuild.Faces(report.B)
	}
	if plane.Face.ID < 0 || plane.Face.ID >= len(faces) {
		report.Reason = vocab.ContactAmbiguousFeature
		return
	}
	faceS := faces[plane.Face.ID]
	bound := proofbound.RatFloatUp(separation)
	points := make([]contactPoint, 0, len(rims))
	for _, rim := range rims {
		foot := rim
		foot[plane.Axis] = plane.Offset
		onM, okM := planarsweep.PointReading([3]*big.Rat(planarsweep.RatOfDyV3(rim)), lateral, resolution)
		onS, okS := planarsweep.PointReading([3]*big.Rat(planarsweep.RatOfDyV3(foot)), lateral, resolution)
		if !okM || !okS {
			report.Reason = vocab.ContactPointTooCoarse
			return
		}
		point := contactPoint{Normal: normal, NormalAngle: angle,
			Separation: measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(bound),
				Exactness: measurement.ExactnessOf(bound)}}
		if cylinderFirst {
			point.OnA, point.OnB = onM, onS
			point.FeatureA, point.FeatureB = contactFeature{Face: c.Wall}, contactFeature{Face: faceS}
		} else {
			point.OnA, point.OnB = onS, onM
			point.FeatureA, point.FeatureB = contactFeature{Face: faceS}, contactFeature{Face: c.Wall}
		}
		point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
		points = append(points, point)
	}
	report.Manifold = &contactManifold{Points: points}
	report.Reason = vocab.ContactNoReason
}

// ends are the centers of a source cylinder's two end disks.
func ends(c contactround.CylinderProof) [2]proofarith.DyV3 {
	half := proofarith.MustDyOf(.5)
	var center proofarith.DyV3
	for k := range 3 {
		center[k] = proofarith.DyMul(proofarith.DyAdd(c.Box.Lo[k], c.Box.Hi[k]), half)
	}
	pair := [2]proofarith.DyV3{center, center}
	pair[0][c.Axis], pair[1][c.Axis] = c.Box.Lo[c.Axis], c.Box.Hi[c.Axis]
	return pair
}
