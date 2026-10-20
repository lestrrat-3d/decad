package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/sweepproof"

	"github.com/lestrrat-3d/decad/internal/seal"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
)

// sweepFacetedFloor uses the complete lower support face of one verified
// faceted solid. Equal transverse translations keep that face strictly inside
// the source-box floor; only the affine vertical support gap can then change
// the pair relation. A sampled negative gap is left undecided because
// ContactPair does not yet certify overlap for this payload.
func (d *Document) sweepFacetedFloor(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	floor contactbox.SourceBox, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.From, pb.Delta, pa.Delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.From, pa.Delta, pb.Delta
	}
	support, ok, err := contactfaceted.AxisSupportAtPose(ctx, bodyNode(faceted), pose)
	if err != nil {
		return nil, err
	}
	if !ok {
		if pp, facetedPayloadOK := faceted.n.Payload.(facetedPayload); facetedPayloadOK && pp.MeshBound > 0 {
			return d.sweepBoundedFacetedFloorClear(ctx, a, b, pa, pb, req, report, floor, facetedFirst)
		}
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	if proofarith.DyCmp(support.OuterHi[2], support.Plane) <= 0 ||
		proofarith.DyCmp(facetedDelta[0], floorDelta[0]) != 0 ||
		proofarith.DyCmp(facetedDelta[1], floorDelta[1]) != 0 {
		return facetedSweepUndecided(report, pa.Duration), nil
	}
	for i := range 2 {
		if proofarith.DyCmp(floor.Lo[i], support.FootLo[i]) >= 0 ||
			proofarith.DyCmp(support.FootHi[i], floor.Hi[i]) >= 0 {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
	}
	patch := contactbox.SourceBox{}
	for i := range 2 {
		patch.Lo[i], patch.Hi[i] = support.FootLo[i], support.FootHi[i]
	}
	patch.Lo[2], patch.Hi[2] = support.Plane, support.OuterHi[2]
	patch.Faces[2][0] = support.Face
	boxA, boxB := floor, patch
	if facetedFirst {
		boxA, boxB = patch, floor
	}
	report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, BoxA: boxA, BoxB: boxB,
		Request: req.ContactRequest})
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
		result.SetReplay(seal.Key{}, nil)
		return result, nil
	}
	// An impact bracket may end only on exact touch. Generic source-box
	// overlap is not a faceted-body overlap proof.
	if result.Outcome == SweepImpactBracket {
		floorDelta, facetedDelta := pa.Delta, pb.Delta
		if facetedFirst {
			floorDelta, facetedDelta = pb.Delta, pa.Delta
		}
		startGap := proofarith.DySubScalar(patch.Lo[2], floor.Hi[2])
		slope := proofarith.DySubScalar(facetedDelta[2], floorDelta[2])
		endGap := new(big.Rat).Add(startGap.Rat(),
			new(big.Rat).Mul(slope.Rat(), result.BracketRight(seal.Key{})))
		if endGap.Sign() != 0 {
			result.Outcome, result.Cause = SweepUndecided, SweepContactUnsupported
			result.Unresolved = result.Bracket
			result.Bracket, result.Event = nil, nil
			result.SetBracketRight(seal.Key{}, nil)
			result.SetReplay(seal.Key{}, nil)
			return result, nil
		}
	}
	result.Replay(seal.Key{}).Snapshot(result.Outcome)
	return result, nil
}

// sweepBoundedFacetedFloorClear uses only the Boolean boundary displacement
// when no exact source support-face proof survives placement.
func (d *Document) sweepBoundedFacetedFloorClear(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	floor contactbox.SourceBox, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.From, pb.Delta, pa.Delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.From, pa.Delta, pb.Delta
	}
	for axis := range 2 {
		if !facetedDelta[axis].IsZero() || !floorDelta[axis].IsZero() {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
	}
	extent, ok, err := contactfaceted.BoundedExtentAtPose(ctx, bodyNode(faceted), pose)
	if err != nil {
		return nil, err
	}
	if !ok || !contactfaceted.InsideFloor(extent, floor) {
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
	endExtent.Box = translatedAffineBox(extent.Box, facetedDelta)
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
		observedFloor, observedExtent, deviation, ok := sweepproof.BoundedFacetedReplayBoxes(
			floor, extent, pa, pb, poseA, poseB, fraction, facetedFirst)
		resolution, _ := sweeppath.ExactBaseValue(req.PointResolution)
		if !ok || deviation.Cmp(resolution) > 0 ||
			!contactfaceted.InsideFloor(observedExtent, observedFloor) {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		if _, ok := contactfaceted.FloorGap(observedExtent, observedFloor); !ok {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		idealExtent, idealFloor := extent, floor
		if index == 1 {
			idealExtent, idealFloor = endExtent, endFloor
		}
		held, ok := contactfaceted.FloorGap(idealExtent, idealFloor)
		if !ok {
			return facetedSweepUndecided(report, pa.Duration), nil
		}
		gap := held
		at := sweeppath.Instant(fraction, pa.Duration)
		report.Samples = append(report.Samples, sampleWith(SweepSample{At: at, PoseA: poseA, PoseB: poseB,
			FloatContact: contact, Ideal: SweepEvent{At: at, Relation: ContactSeparated, Gap: &gap}}, fraction))
		report.PoseEvaluations++
	}
	report.Outcome, report.BoxExcluded = SweepClear, true
	report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, Request: req.ContactRequest,
		BoxA: floor, BoxB: extent.Box, FacetedClear: &extent, FacetedFirst: facetedFirst})
	if facetedFirst {
		report.Replay(seal.Key{}).BoxA, report.Replay(seal.Key{}).BoxB = extent.Box, floor
	}
	report.Replay(seal.Key{}).Snapshot(report.Outcome)
	return report, nil
}

func facetedSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweeppath.Instant(new(big.Rat), duration),
		To: sweeppath.Instant(big.NewRat(1, 1), duration)}
	return report
}
