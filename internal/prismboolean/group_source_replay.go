package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// GroupUnionSourceReplay contains geometry proof data for an authenticated,
// ordered source list. Root authenticates the public operation history and
// supplies its topology nodes before calling ReplayGroupUnionSource.
type GroupUnionSourceReplay struct {
	Owners    []int
	GroupCaps []ReplayGroupCapFaces
	LeafCaps  []ReplayGroupCapFaces
}

// ReplayGroupUnionSource replays the whole-circle Union of authenticated
// source nodes, binds each output region to one source, and verifies the group
// and leaf cap structures. It does not read or authenticate document history.
// group, budget and every input MUST be non-nil; inputs MUST contain at least
// one node, be in operation order, and be authenticated by the caller.
func ReplayGroupUnionSource(ctx context.Context, group *topo.Body, inputs []*topo.Body,
	budget *proofbound.ReplayBudget) (*GroupUnionSourceReplay, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	groupPayload, ok := group.Payload.(stackedrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, 1, 8); err != nil {
		return nil, false, err
	}
	lineage := []int{0}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	payload := inputs[0].Payload
	for i := 1; i < len(inputs); i++ {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		next, rows, replayed, err := ReplayWholeCircleGroupUnion(ctx, budget, payload, inputs[i].Payload)
		if err != nil || !replayed {
			return nil, false, err
		}
		lineage, replayed, err = ReplayAdvanceGroupOrigins(ctx, lineage, rows, i, budget)
		if err != nil || !replayed {
			return nil, false, err
		}
		payload = next
	}
	replayed, ok := payload.(stackedrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	owners, ok, err := ReplayBindGroupOrigins(ctx, groupPayload, replayed, lineage, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	groupCaps, ok, err := ReplayGroupCapStructure(ctx, group, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(inputs))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(inputs)), 24); err != nil {
		return nil, false, err
	}
	leafCaps := make([]ReplayGroupCapFaces, len(inputs))
	for i, leaf := range inputs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if _, ok := leaf.Payload.(prismrecord.Payload); !ok {
			return nil, false, nil
		}
		faces, ok, err := ReplayPrismCapStructure(ctx, leaf, budget)
		if err != nil || !ok || len(faces) != 1 {
			return nil, false, err
		}
		leafCaps[i] = faces[0]
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	slab := groupPayload.Slabs[0]
	groupPlacement := prismplacement.Operand{Frame: groupPayload.Frame, Xform: groupPayload.Xform,
		Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta,
		Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	for region, owner := range owners {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if owner < 0 || owner >= len(inputs) {
			return nil, false, nil
		}
		for previous := range region {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if owners[previous] == owner {
				return nil, false, nil
			}
		}
		source, ok := inputs[owner].Payload.(prismrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		match, err := ReplayWholeCircleMatch(ctx, slab.Regions[region], source.Profile,
			groupPlacement, source.PlacementOperand(), budget)
		if err != nil || !match {
			return nil, false, err
		}
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytes(ctx, 256); err != nil {
		return nil, false, err
	}
	return &GroupUnionSourceReplay{Owners: owners, GroupCaps: groupCaps, LeafCaps: leafCaps}, true, nil
}
