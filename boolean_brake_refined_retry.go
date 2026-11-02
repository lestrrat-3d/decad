package decad

import (
	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// retryBrakeRefinedUnion carries the authenticated slot and pin through one
// private rebuild before replaying the original Union and Cut.
func retryBrakeRefinedUnion(d *Document, in *brakeReplayInputs, hooks booleandispatch.Hooks,
	budget *proofbound.ReplayBudget) (*Body, producerID, bool, bool, error) {
	if d == nil || in == nil || budget == nil || in.c == nil || in.c.Payload == nil {
		return nil, 0, false, false, errBrakeRefinedReplayInputsMissing
	}
	fail := func(stage string, err error) (*Body, producerID, bool, bool, error) {
		traceBrakeReplay(in.ctx, stage, err)
		return brakeRetryFailure(in.ctx, err)
	}
	ref := d.nextProducerID()
	if ref < 0 || ref > producerID(int(^uint(0)>>1)-4) {
		return fail("refined producer identity", errBrakeRefinedReplayProducerOverflow)
	}
	refined, err := buildBrakeRefinedTool(in.ctx, d, in, ref, budget)
	if err != nil {
		return fail("refined tool", err)
	}
	cut, err := refined.cut(in.ctx, d, ref+1, hooks, budget)
	if err != nil {
		return fail("refined clevis Cut", err)
	}
	payload, err := brakeRefinedPlacementPayload(in.ctx, refined, cut, budget)
	if err != nil {
		return fail("refined clevis ancestry", err)
	}
	if !refined.growthVerified {
		return fail("refined growth", errBrakeRefinedGrowthProofMissing)
	}
	placed, err := payload.PlacedReplay(in.ctx, envOf(documentNode(d), ref+2), in.c.Payload.Transform(), budget)
	if err != nil {
		return fail("refined clevis placement", err)
	}
	if len(placed.OperationWarnings) != 0 {
		return fail("refined clevis placement", errBrakeRefinedPlacementWarned)
	}
	joined, movement, shiftWarnings, err := brakeReplayUnion(d, in, placed, refined.verifiedGrowth, ref+3, hooks, budget)
	if err != nil {
		return fail("refined Union", err)
	}
	final, err := brakeReplayFinal(d, in, joined, movement, cut.OperationWarnings, shiftWarnings, ref+4, hooks, budget)
	if err != nil {
		return fail("refined final Cut", err)
	}
	for _, source := range []*topo.Body{in.a, in.b, in.blank, in.u, in.c0, in.c} {
		if source == nil || source.Origin.Producer >= ref && source.Origin.Producer <= ref+4 {
			return fail("refined producer identity", errBrakePrivateProducerCollidesWithAPublicSource)
		}
	}
	if err := stripBrakePrivateSources(in.ctx, final, placed, joined, ref+2, budget); err != nil {
		return fail("refined output ancestry", err)
	}
	return bodyHandle(final), 5, true, true, nil
}
