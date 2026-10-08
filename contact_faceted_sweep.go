package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/sweeppath"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// sweepFacetedFloor uses the complete lower support face of one verified
// faceted solid. Equal transverse translations keep that face strictly inside
// the source-box floor; only the affine vertical support gap can then change
// the pair relation. A sampled negative gap is left undecided because
// ContactPair does not yet certify overlap for this payload.
func (d *Document) sweepFacetedFloor(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	floor sourceBoxContactProof, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.From, pb.Delta, pa.Delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.From, pa.Delta, pb.Delta
	}
	support, ok, err := sourceFacetedAxisSupport(ctx, faceted, pose, 2, 0)
	if err != nil {
		return nil, err
	}
	if !ok {
		if pp, facetedPayloadOK := faceted.payload.(facetedPayload); facetedPayloadOK && pp.meshBound > 0 {
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
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
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
		contact, err := d.ContactPair(ctx, a, b, poseA, poseB, req.ContactRequest)
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
		gap, ok := boundedFacetedFloorGap(idealExtent, idealFloor)
		if !ok {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
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
