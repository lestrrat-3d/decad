package decad

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type brakeRefinedTool struct {
	blank, original, slot, pin, body   *topo.Body
	blankID, originalID, slotID, pinID producerID
	cert                               *booleaneval.BrakeCandidateCertificate
	growth                             *booleaneval.AuthenticatedGrowth
	budget                             *proofbound.ReplayBudget
	cutBody                            *topo.Body
	cutSnapshot                        *booleaneval.BrakePlacementSnapshot
	verifiedGrowth                     float64
	growthVerified                     bool
}

func buildBrakeRefinedTool(ctx context.Context, d *Document, in *brakeReplayInputs, ref producerID,
	b *proofbound.ReplayBudget) (*brakeRefinedTool, error) {
	if d == nil || in == nil || in.blank == nil || in.u == nil || b == nil || !b.IsBrakeReplay() ||
		in.blank.Doc != documentNode(d) || in.u.Doc != documentNode(d) || len(in.u.OperationWarnings) != 0 {
		return nil, errBrakeRefinedClevisSourceUnavailable
	}
	if err := b.SpendWork(ctx, 512); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 32); err != nil {
		return nil, err
	}
	if err := b.ReserveBytes(ctx, 8192); err != nil {
		return nil, err
	}
	kind, sources, ok, err := brakeExpandable(ctx, d, in.u, b)
	if err != nil {
		return nil, err
	}
	if !ok || kind != meshbool.OpUnion || len(sources) != 2 || sources[0] == nil || sources[1] == nil ||
		sources[0] == sources[1] || sources[0].Doc != documentNode(d) || sources[1].Doc != documentNode(d) {
		return nil, errBrakeRefinedClevisUnionHistoryChanged
	}
	if in.c0 == nil || in.c0Growth == nil {
		return nil, errBrakeOriginalClevisGrowthIsMissing
	}
	growth, err := booleaneval.AuthenticateBrakeGrowth(ctx, in.c0Snapshot, in.c0, in.blank, in.u, sources[0], sources[1], in.c0Warning, b)
	if err != nil {
		return nil, err
	}
	body, cert, err := booleaneval.BuildBrakeCircleCandidate(ctx, envOf(documentNode(d), ref), in.blank, sources[0], sources[1], b)
	if err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(brakeRefinedTool{}))); err != nil {
		return nil, err
	}
	return &brakeRefinedTool{blank: in.blank, original: in.u, slot: sources[0], pin: sources[1], body: body,
		blankID: in.blank.Origin.Producer, originalID: in.u.Origin.Producer, slotID: sources[0].Origin.Producer,
		pinID: sources[1].Origin.Producer, cert: cert, growth: growth, budget: b}, nil
}

func (t *brakeRefinedTool) cut(ctx context.Context, d *Document, ref producerID,
	hooks booleandispatch.Hooks, b *proofbound.ReplayBudget) (*topo.Body, error) {
	if t == nil || d == nil || b == nil || t.budget != b || t.blank == nil || t.original == nil || t.slot == nil || t.pin == nil ||
		t.body == nil || t.blank.Origin.Producer != t.blankID || t.original.Origin.Producer != t.originalID ||
		t.slot.Origin.Producer != t.slotID || t.pin.Origin.Producer != t.pinID || len(t.original.OperationWarnings) != 0 {
		return nil, errBrakeRefinedClevisCapabilityChanged
	}
	if err := b.SpendWork(ctx, 512); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 32); err != nil {
		return nil, err
	}
	if err := b.ReserveBytes(ctx, 8192); err != nil {
		return nil, err
	}
	kind, sources, ok, err := brakeExpandable(ctx, d, t.original, b)
	if err != nil {
		return nil, err
	}
	if !ok || kind != meshbool.OpUnion || len(sources) != 2 || sources[0] != t.slot || sources[1] != t.pin {
		return nil, errBrakeRefinedClevisSourceHistoryChanged
	}
	result, err := booleandispatch.BodyBrakeRefinedClevisCut(booleaneval.WithBrakeAuthenticatedGrowth(ctx, t.growth), envOf(documentNode(d), ref), t.blank, t.body, t.cert, hooks, b)
	if err != nil {
		return nil, err
	}
	payload, ok := result.Payload.(facetedrecord.Payload)
	if !ok {
		return nil, errBrakeRefinedClevisPayloadMissing
	}
	snapshot, err := booleaneval.SnapshotBrakePlacement(ctx, payload, b)
	if err != nil {
		return nil, err
	}
	t.cutBody, t.cutSnapshot = result, snapshot
	return result, nil
}
