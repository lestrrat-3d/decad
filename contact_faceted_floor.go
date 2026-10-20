package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func certifyFacetedUnionLowerSupport(ctx context.Context, result, a, b *Body) error {
	pp, ok := result.n.Payload.(facetedPayload)
	if !ok || pp.MeshBound <= 0 || len(pp.ExactSourceVerts) != 0 {
		return nil
	}
	for candidate, operands := range [][2]*Body{{a, b}, {b, a}} {
		base, upper := operands[0], operands[1]
		box, exact := sourceBoxAtPose(base, r3.Identity())
		if !exact {
			continue
		}
		bounds, err := upper.Bounds()
		if err != nil || bounds.Bound.Kind() != units.Length ||
			!measurement.FiniteValues(bounds.Min.Z, bounds.Bound.Base()) || bounds.Bound.Base() < 0 ||
			proofarith.DyCmp(proofarith.DySubScalar(proofarith.MustDyOf(bounds.Min.Z),
				proofarith.MustDyOf(bounds.Bound.Base())), box.lo[2]) <= 0 {
			continue
		}
		faces := base.Faces()
		for i, face := range faces {
			if face != box.faces[2][0] {
				continue
			}
			sourceGroup := i + candidate*len(a.Faces())
			pp.LowerSupport = &facetproof.LowerSupport{Plane: box.lo[2], SourceGroup: sourceGroup,
				FootLo: [2]proofarith.Dyadic{box.lo[0], box.lo[1]},
				FootHi: [2]proofarith.Dyadic{box.hi[0], box.hi[1]}}
			result.n.Payload = pp
			_, proven, err := sourceFacetedAxisSupport(ctx, result, r3.Identity())
			if err != nil {
				return err
			}
			if proven {
				return nil
			}
			pp.LowerSupport = nil
			result.n.Payload = pp
			break
		}
	}
	return nil
}

// boundedFacetedExtent encloses the true boundary by the held vertex extrema
// and the payload's two-sided boundary displacement. It does not identify a
// support face, so callers may use it only for strict separation.
type boundedFacetedExtent struct {
	box   sourceBoxContactProof
	bound proofarith.Dyadic
}

func sourceBoundedFacetedExtent(ctx context.Context, b *Body,
	pose r3.Transform) (boundedFacetedExtent, bool, error) {
	if b == nil || !b.n.Solid || b.n.Kind != topo.BodySolid || !pairbox.SignedAxisTransform(pose) {
		return boundedFacetedExtent{}, false, nil
	}
	pp, ok := b.n.Payload.(facetedPayload)
	if !ok || len(pp.Verts) == 0 || !measurement.FiniteValues(pp.MeshBound, pp.VolSymDiff) ||
		pp.MeshBound < 0 || pp.VolSymDiff < 0 {
		return boundedFacetedExtent{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	var proof boundedFacetedExtent
	proof.bound = proofarith.MustDyOf(pp.MeshBound)
	for i, v := range pp.Verts {
		if err := budget.Step(); err != nil {
			return boundedFacetedExtent{}, false, err
		}
		if !proofbound.FiniteVec(v) {
			return boundedFacetedExtent{}, false, nil
		}
		placed := proofarith.DvTransform(pose, proofarith.DyVec(v))
		for axis := range 3 {
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.lo[axis]) < 0 {
				proof.box.lo[axis] = placed[axis]
			}
			if i == 0 || proofarith.DyCmp(placed[axis], proof.box.hi[axis]) > 0 {
				proof.box.hi[axis] = placed[axis]
			}
		}
	}
	return proof, true, budget.Err()
}

func boundedFacetedInsideFloor(extent boundedFacetedExtent, floor sourceBoxContactProof) bool {
	for axis := range 2 {
		if proofarith.DyCmp(proofarith.DySubScalar(extent.box.lo[axis], extent.bound), floor.lo[axis]) <= 0 ||
			proofarith.DyCmp(proofarith.DyAdd(extent.box.hi[axis], extent.bound), floor.hi[axis]) >= 0 {
			return false
		}
	}
	return true
}

func boundedFacetedFloorGap(extent boundedFacetedExtent,
	floor sourceBoxContactProof) (measurement.Measurement, bool) {
	held := proofarith.DySubScalar(extent.box.lo[2], floor.hi[2])
	if proofarith.DyCmp(held, extent.bound) <= 0 {
		return measurement.Measurement{}, false
	}
	reading, ok := pairbox.SignedMeasurement(held)
	if !ok {
		return measurement.Measurement{}, false
	}
	boundaryBound, _ := extent.bound.Float64()
	bound := proofbound.AbsSumUpper(reading.Bound.Base(), boundaryBound)
	if !measurement.FiniteValues(bound) || reading.Value.Base()-bound <= 0 {
		return measurement.Measurement{}, false
	}
	reading.Bound = units.Millimeters(bound)
	reading.Exactness = measurement.ExactnessOf(bound)
	return reading, true
}

// facetedAxisSupport is one complete rectangular extremal face of a faceted
// solid whose true support geometry is exact. The coordinates and footprint
// are exact dyadics; face belongs to the caller's body.
type facetedAxisSupport struct {
	face             *Face
	axis, side       int
	plane            proofarith.Dyadic
	footLo, footHi   [2]proofarith.Dyadic
	outerLo, outerHi [3]proofarith.Dyadic
	corners          [4]proofarith.DyV3
	normal           r3.Vec
}

// sourceFacetedAxisSupport proves that every point of b lies on the material
// side of one axis plane and that its complete contact set on that plane is
// one rectangular, outward-facing source Face. A zero-bound Boolean is read
// directly. A translation-only placement may instead read its saved exact
// source mesh, after checking that the rebuilt mesh stays within its bound.
// A positive-bound Union can also use its exact source-box lower face when
// its other operand is certified strictly above that face.
func sourceFacetedAxisSupport(ctx context.Context, b *Body, pose r3.Transform) (facetedAxisSupport, bool, error) {
	const (
		axis = 2
		side = 0
	)
	if b == nil ||
		!b.n.Solid || b.n.Kind != topo.BodySolid || !pairbox.SignedAxisTransform(pose) {
		return facetedAxisSupport{}, false, nil
	}
	pp, ok := b.n.Payload.(facetedPayload)
	if !ok {
		return facetedAxisSupport{}, false, nil
	}
	faces := b.Faces()
	valid := make([]bool, len(faces))
	for i, face := range faces {
		valid[i] = face.n.HeldPlanar && face.n.Surface.Kind() == surfacegeom.KindFaceted
	}
	result, proven, err := facetproof.ProveAxisSupport(ctx, facetproof.AxisSupportInput{
		Verts: pp.Verts, ExactSourceVerts: pp.ExactSourceVerts,
		Tris: pp.Tris, ExactSourceTris: pp.ExactSourceTris,
		FaceOf: pp.FaceOf, SourceGroup: pp.Src, FaceValid: valid,
		GroupCount: len(pp.Groups), MeshBound: pp.MeshBound,
		VolSymDiff: pp.VolSymDiff, Xform: pp.Xform, Pose: pose,
		Lower: pp.LowerSupport, Axis: axis, Side: side,
	})
	if err != nil || !proven {
		return facetedAxisSupport{}, false, err
	}
	return facetedAxisSupport{
		face: faces[result.FaceIndex], axis: result.Axis, side: result.Side,
		plane: result.Plane, footLo: result.FootLo, footHi: result.FootHi,
		outerLo: result.OuterLo, outerHi: result.OuterHi,
		corners: result.Corners, normal: result.Normal,
	}, true, nil
}

// classifyFacetedFloorBox admits a faceted body only when its complete lower
// support patch lies strictly inside a source-box floor's upper face. The
// certified support plane separates the whole occupied sets at and above it.
func classifyFacetedFloorBox(ctx context.Context, report *reportvocab.ContactReport, faceted *Body,
	pose r3.Transform, floor sourceBoxContactProof, facetedIsA bool) error {
	report.Reason = reportvocab.ContactPayloadUnsupported
	support, ok, err := sourceFacetedAxisSupport(ctx, faceted, pose)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if !ok {
		extent, bounded, err := sourceBoundedFacetedExtent(ctx, faceted, pose)
		if err != nil {
			return err
		}
		if bounded && extent.bound.Sign() > 0 && boundedFacetedInsideFloor(extent, floor) {
			if gap, measured := boundedFacetedFloorGap(extent, floor); measured {
				report.Relation, report.Gap, report.Reason = reportvocab.ContactSeparated, &gap, reportvocab.ContactNoReason
			}
		}
		return nil
	}
	for j := range 2 {
		if proofarith.DyCmp(floor.lo[j], support.footLo[j]) >= 0 ||
			proofarith.DyCmp(support.footHi[j], floor.hi[j]) >= 0 {
			return nil
		}
	}
	gap := proofarith.DySubScalar(support.plane, floor.hi[2])
	if gap.Sign() < 0 {
		return nil
	}
	if gap.Sign() > 0 {
		var gaps [3]proofarith.Dyadic
		gaps[2] = gap
		m, measured := pairbox.GapMeasurement(gaps)
		if !measured {
			report.Reason = reportvocab.ContactNoGapProof
			return nil
		}
		report.Relation, report.Gap, report.Reason = reportvocab.ContactSeparated, &m, reportvocab.ContactNoReason
		return nil
	}
	report.Relation = reportvocab.ContactTouching
	exactZero := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
	report.Gap = &exactZero
	patch := sourceBoxContactProof{}
	patch.lo[2], patch.hi[2] = support.plane, support.outerHi[2]
	patch.faces[2][0] = support.face
	for j := range 2 {
		patch.lo[j], patch.hi[j] = support.footLo[j], support.footHi[j]
	}
	if facetedIsA {
		publishSourceBoxPatch(report, patch, floor, 2, -1, proofarith.DyZero())
	} else {
		publishSourceBoxPatch(report, floor, patch, 2, 1, proofarith.DyZero())
	}
	if report.Manifold != nil {
		report.Reason = reportvocab.ContactNoReason
	}
	return nil
}

// sweepFacetedFloor uses the complete lower support face of one verified
// faceted solid. Equal transverse translations keep that face strictly inside
// the source-box floor; only the affine vertical support gap can then change
// the pair relation. A sampled negative gap is left undecided because
// ContactPair does not yet certify overlap for this payload.
func (d *Document) sweepFacetedFloor(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req reportvocab.SweepRequest, report *SweepReport,
	floor sourceBoxContactProof, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.From, pb.Delta, pa.Delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.From, pa.Delta, pb.Delta
	}
	support, ok, err := sourceFacetedAxisSupport(ctx, faceted, pose)
	if err != nil {
		return nil, err
	}
	if !ok {
		if pp, facetedPayloadOK := faceted.n.Payload.(facetedPayload); facetedPayloadOK && pp.MeshBound > 0 {
			return d.sweepBoundedFacetedFloorClear(ctx, a, b, pa, pb, req, report, floor, facetedFirst)
		}
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	if proofarith.DyCmp(support.outerHi[2], support.plane) <= 0 ||
		proofarith.DyCmp(facetedDelta[0], floorDelta[0]) != 0 ||
		proofarith.DyCmp(facetedDelta[1], floorDelta[1]) != 0 {
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	for i := range 2 {
		if proofarith.DyCmp(floor.lo[i], support.footLo[i]) >= 0 ||
			proofarith.DyCmp(support.footHi[i], floor.hi[i]) >= 0 {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
	}
	patch := sourceBoxContactProof{}
	for i := range 2 {
		patch.lo[i], patch.hi[i] = support.footLo[i], support.footHi[i]
	}
	patch.lo[2], patch.hi[2] = support.plane, support.outerHi[2]
	patch.faces[2][0] = support.face
	boxA, boxB := floor, patch
	if facetedFirst {
		boxA, boxB = patch, floor
	}
	report.replay = &sweepReplayProof{pa: pa, pb: pb, boxA: boxA, boxB: boxB,
		request: req.ContactRequest}
	run := pairSweepRun{doc: d, a: a, b: b, pa: pa, pb: pb, req: req,
		report: report, boxA: boxA, boxB: boxB, facetedFloor: true}
	resolution, _ := sweeppath.ExactBaseValue(req.TimeResolution)
	result, err := run.execute(ctx, resolution)
	if err != nil || result == nil {
		return result, err
	}
	if result.Outcome == SweepUndecided || result.Outcome == SweepInitiallyOverlapping ||
		result.Outcome == SweepImpactBracket &&
			(result.Event == nil || result.Event.Relation != ContactTouching) {
		result.replay = nil
		return result, nil
	}
	// An impact bracket may end only on exact touch. Generic source-box
	// overlap is not a faceted-body overlap proof.
	if result.Outcome == SweepImpactBracket {
		floorDelta, facetedDelta := pa.Delta, pb.Delta
		if facetedFirst {
			floorDelta, facetedDelta = pb.Delta, pa.Delta
		}
		startGap := proofarith.DySubScalar(patch.lo[2], floor.hi[2])
		slope := proofarith.DySubScalar(facetedDelta[2], floorDelta[2])
		endGap := new(big.Rat).Add(startGap.Rat(),
			new(big.Rat).Mul(slope.Rat(), result.bracketRight))
		if endGap.Sign() != 0 {
			result.Outcome, result.Cause = SweepUndecided, SweepContactUnsupported
			result.Unresolved = result.Bracket
			result.Bracket, result.Event, result.bracketRight = nil, nil, nil
			result.replay = nil
			return result, nil
		}
	}
	result.replay.snapshot(result)
	return result, nil
}

// sweepBoundedFacetedFloorClear uses only the Boolean boundary displacement
// when no exact source support-face proof survives placement.
func (d *Document) sweepBoundedFacetedFloorClear(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req reportvocab.SweepRequest, report *SweepReport,
	floor sourceBoxContactProof, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.From, pb.Delta, pa.Delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.From, pa.Delta, pb.Delta
	}
	for axis := range 2 {
		if !facetedDelta[axis].IsZero() || !floorDelta[axis].IsZero() {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
	}
	extent, ok, err := sourceBoundedFacetedExtent(ctx, faceted, pose)
	if err != nil {
		return nil, err
	}
	if !ok || !boundedFacetedInsideFloor(extent, floor) {
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	// The swept boxes enclose both bodies over the whole path, so their
	// strict separation is the clear certificate; the endpoint gaps below are
	// the published readings.
	sweptA, okA := sweptBoxOf(a, pa)
	sweptB, okB := sweptBoxOf(b, pb)
	if !okA || !okB || !sweptA.StrictlyDisjoint(sweptB) {
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	endExtent := extent
	endExtent.box = translatedAffineBox(extent.box, facetedDelta)
	endFloor := translatedAffineBox(floor, floorDelta)
	for index, fraction := range []*big.Rat{new(big.Rat), big.NewRat(1, 1)} {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		poseA, err := pa.PoseAt(fraction)
		if err != nil {
			return nil, err
		}
		poseB, err := pb.PoseAt(fraction)
		if err != nil {
			return nil, err
		}
		contact, err := d.ContactPair(ctx, a, b, poseA, poseB, contactRequestFromInternal(req.ContactRequest))
		if err != nil {
			return nil, err
		}
		if contact.Relation != ContactSeparated {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		observedFloor, observedExtent, deviation, ok := boundedFacetedReplayBoxes(
			floor, extent, pa, pb, poseA, poseB, fraction, facetedFirst)
		resolution, _ := sweeppath.ExactBaseValue(req.PointResolution)
		if !ok || deviation.Cmp(resolution) > 0 ||
			!boundedFacetedInsideFloor(observedExtent, observedFloor) {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		if _, ok := boundedFacetedFloorGap(observedExtent, observedFloor); !ok {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		idealExtent, idealFloor := extent, floor
		if index == 1 {
			idealExtent, idealFloor = endExtent, endFloor
		}
		held, ok := boundedFacetedFloorGap(idealExtent, idealFloor)
		if !ok {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		gap := measurementFromInternal(held)
		at := sweepInstant(fraction, pa.Duration)
		report.Samples = append(report.Samples, SweepSample{At: at, PoseA: poseA, PoseB: poseB,
			FloatContact: contact, Ideal: SweepEvent{At: at, Relation: ContactSeparated, Gap: &gap},
			exactFraction: fraction})
		report.PoseEvaluations++
	}
	report.Outcome, report.BoxExcluded = SweepClear, true
	report.replay = &sweepReplayProof{pa: pa, pb: pb, request: req.ContactRequest,
		boxA: floor, boxB: extent.box, facetedClear: &extent, facetedFirst: facetedFirst}
	if facetedFirst {
		report.replay.boxA, report.replay.boxB = extent.box, floor
	}
	report.replay.snapshot(report)
	return report, nil
}

func boundedFacetedReplayBoxes(floor sourceBoxContactProof, extent boundedFacetedExtent,
	pa, pb affinePairPath, poseA, poseB r3.Transform, f *big.Rat,
	facetedFirst bool) (sourceBoxContactProof, boundedFacetedExtent, *big.Rat, bool) {
	facetedPath, floorPath := pb, pa
	facetedPose, floorPose := poseB, poseA
	if facetedFirst {
		facetedPath, floorPath, facetedPose, floorPose = pa, pb, poseA, poseB
	}
	observedFloor, okFloor := translatedReplayBox(floor, floorPath.From, floorPose)
	observedFacet, okFacet := translatedReplayBox(extent.box, facetedPath.From, facetedPose)
	if !okFloor || !okFacet {
		return sourceBoxContactProof{}, boundedFacetedExtent{}, nil, false
	}
	deviation := boxPoseDeviation(floor, observedFloor, floorPath.Delta, f)
	deviation.Add(deviation, boxPoseDeviation(extent.box, observedFacet, facetedPath.Delta, f))
	extent.box = observedFacet
	return observedFloor, extent, deviation, true
}

func facetedSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return report
}
