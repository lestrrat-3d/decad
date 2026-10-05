package decad

import (
	"context"
	"math/big"
)

// sweepFacetedFloor uses the complete lower support face of one verified
// faceted solid. Equal transverse translations keep that face strictly inside
// the source-box floor; only the affine vertical support gap can then change
// the pair relation. A sampled negative gap is left undecided because
// ContactPair does not yet certify overlap for this payload.
func (d *Document) sweepFacetedFloor(ctx context.Context, a, b *Body,
	pa, pb affinePairPath, req SweepRequest, report *SweepReport,
	floor sourceBoxContactProof, facetedFirst bool) (*SweepReport, error) {
	faceted, pose, facetedDelta, floorDelta := b, pb.from, pb.delta, pa.delta
	if facetedFirst {
		faceted, pose, facetedDelta, floorDelta = a, pa.from, pa.delta, pb.delta
	}
	support, ok, err := sourceFacetedAxisSupport(ctx, faceted, pose, 2, 0)
	if err != nil {
		return nil, err
	}
	if !ok || dyCmp(support.outerHi[2], support.plane) <= 0 ||
		dyCmp(facetedDelta[0], floorDelta[0]) != 0 ||
		dyCmp(facetedDelta[1], floorDelta[1]) != 0 {
		return facetedSweepUndecided(report, pa.duration), nil
	}
	for i := range 2 {
		if dyCmp(floor.lo[i], support.footLo[i]) >= 0 ||
			dyCmp(support.footHi[i], floor.hi[i]) >= 0 {
			return facetedSweepUndecided(report, pa.duration), nil
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
	resolution, _ := exactBaseValue(req.TimeResolution)
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
		floorDelta, facetedDelta := pa.delta, pb.delta
		if facetedFirst {
			floorDelta, facetedDelta = pb.delta, pa.delta
		}
		startGap := dySubScalar(patch.lo[2], floor.hi[2])
		slope := dySubScalar(facetedDelta[2], floorDelta[2])
		endGap := new(big.Rat).Add(startGap.rat(),
			new(big.Rat).Mul(slope.rat(), result.bracketRight))
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

func facetedSweepUndecided(report *SweepReport, duration *big.Rat) *SweepReport {
	report.Outcome, report.Cause = SweepUndecided, SweepContactUnsupported
	report.Unresolved = &SweepInterval{From: sweepInstant(new(big.Rat), duration),
		To: sweepInstant(big.NewRat(1, 1), duration)}
	return report
}
