package decad

import (
	"context"
	"sync/atomic"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

// groupUnionInputs snapshots the public Union's ordered source prisms. It stays
// in the root package because only the document owns public operation history.
// A successful snapshot does not certify the regions or admit a Boolean route.
// The caller owns the returned slice and must use the same budget for the
// remaining source and output proofs. budget and group must not be nil.
func groupUnionInputs(budget *proofbound.WorkBudget, group *Body) ([]*Body, bool, error) {
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if err := budget.Step(); err != nil {
		return nil, false, err
	}
	if group.n.Doc == nil || group.n.Owner != group || group.n.Origin.Role != provenance.RoleBody || len(group.n.OperationWarnings) != 0 {
		return nil, false, nil
	}
	payload, ok := group.n.Payload.(stackedrecord.Payload)
	if !ok || !payload.IsGroup() || len(payload.Upstream) != 0 {
		return nil, false, nil
	}
	doc := documentHandle(group.n.Doc)
	doc.n.LiveMu.RLock()
	defer doc.n.LiveMu.RUnlock()
	op := doc.producers[group]
	if op == nil || op.kind != OperationUnion || op.doc != group.n.Doc || len(op.outputs) != 1 || op.outputs[0] != group ||
		len(op.inputs) != len(payload.Slabs[0].Regions) {
		return nil, false, nil
	}
	count := len(op.inputs)
	if err := budget.Reserve(uint64(count)); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytesMul(uint64(count), 8); err != nil {
		return nil, false, err
	}
	inputs := make([]*Body, 0, count)
	for i := range op.inputs {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		input := op.inputs[i]
		leaf := input.Body
		role := "operand"
		if count == 2 {
			role = "first"
			if i == 1 {
				role = "others"
			}
		}
		if leaf == nil || input.Use != InputConsumed || input.Role != role || leaf.n.Doc != group.n.Doc || leaf.n.Owner != leaf ||
			leaf.n.Origin.Role != provenance.RoleBody || len(leaf.n.OperationWarnings) != 0 {
			return nil, false, nil
		}
		if _, ok := leaf.n.Payload.(prismrecord.Payload); !ok {
			return nil, false, nil
		}
		producer := doc.producers[leaf]
		if producer == nil || producer.doc != group.n.Doc || len(producer.outputs) != 1 || producer.outputs[0] != leaf {
			return nil, false, nil
		}
		for j := range inputs {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			previous := inputs[j]
			if previous == leaf || previous.n.Origin.Producer == leaf.n.Origin.Producer {
				return nil, false, nil
			}
		}
		inputs = append(inputs, leaf)
	}
	return inputs, true, nil
}

// groupDirectPrismSource accepts only a public Extrude or a finite chain of
// public Transforms from one Extrude. This scoped lineage carries no prior
// Boolean roles, which the blind face publisher cannot yet authenticate.
func groupDirectPrismSource(ctx context.Context, body *Body,
	budget *proofbound.ReplayBudget) (bool, error) {
	if body == nil || budget == nil || body.n.Doc == nil {
		return false, nil
	}
	doc := documentHandle(body.n.Doc)
	doc.n.LiveMu.RLock()
	defer doc.n.LiveMu.RUnlock()
	current := body
	for range 64 {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return false, err
		}
		if current == nil || current.n.Owner != current || current.n.Doc != body.n.Doc ||
			current.n.Origin.Role != provenance.RoleBody || len(current.n.OperationWarnings) != 0 {
			return false, nil
		}
		pp, ok := current.n.Payload.(prismrecord.Payload)
		if !ok || len(pp.Upstream) != 0 || !current.n.HasStrictPrismBuildSeal() {
			return false, nil
		}
		op := doc.producers[current]
		if op == nil || op.doc != body.n.Doc || len(op.outputs) != 1 || op.outputs[0] != current {
			return false, nil
		}
		switch op.kind {
		case OperationExtrude:
			return len(op.inputs) == 0, nil
		case OperationTransform:
			if len(op.inputs) != 1 || op.inputs[0].Role != operationRoleSource ||
				op.inputs[0].Use != InputConsumed || op.inputs[0].Body == current {
				return false, nil
			}
			current = op.inputs[0].Body
		default:
			return false, nil
		}
	}
	return false, nil
}

// groupUnionSourceSnapshot replays the ordered public Union and binds its
// segment origins and original Build faces. It is not yet a Cut admission:
// the caller must also prove placement, clearance, cap planes and output.
func groupUnionSourceSnapshot(ctx context.Context, group *Body, budget *proofbound.ReplayBudget) (*groupUnionSource, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	inputs, ok, err := groupUnionInputs(budget.WorkBudget(ctx), group)
	if err != nil || !ok {
		return nil, false, err
	}
	groupPayload, ok := group.n.Payload.(stackedrecord.Payload)
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
	payload := inputs[0].n.Payload
	for i := 1; i < len(inputs); i++ {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		next, rows, replayed, err := prismboolean.ReplayWholeCircleGroupUnion(ctx, budget, payload, inputs[i].n.Payload)
		if err != nil || !replayed {
			return nil, false, err
		}
		lineage, replayed, err = prismboolean.ReplayAdvanceGroupOrigins(ctx, lineage, rows, i, budget)
		if err != nil || !replayed {
			return nil, false, err
		}
		payload = next
	}
	replayed, ok := payload.(stackedrecord.Payload)
	if !ok {
		return nil, false, nil
	}
	owners, ok, err := prismboolean.ReplayBindGroupOrigins(ctx, groupPayload, replayed, lineage, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	groupCaps, ok, err := prismboolean.ReplayGroupCapStructure(ctx, &group.n, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(inputs))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(inputs)), 24); err != nil {
		return nil, false, err
	}
	leafCaps := make([]prismboolean.ReplayGroupCapFaces, len(inputs))
	for i := range inputs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		leaf := inputs[i]
		_, ok := leaf.n.Payload.(prismrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		faces, ok, err := prismboolean.ReplayPrismCapStructure(ctx, &leaf.n, budget)
		if err != nil || !ok || len(faces) != 1 {
			return nil, false, err
		}
		leafCaps[i] = faces[0]
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	slab := groupPayload.Slabs[0]
	groupPlacement := prismplacement.Operand{Frame: groupPayload.Frame, Xform: groupPayload.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	for region := range owners {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		owner := owners[region]
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
		source, ok := inputs[owner].n.Payload.(prismrecord.Payload)
		if !ok {
			return nil, false, nil
		}
		match, err := prismboolean.ReplayWholeCircleMatch(ctx, slab.Regions[region], source.Profile, groupPlacement, source.PlacementOperand(), budget)
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
	return &groupUnionSource{Inputs: inputs, Owners: owners, GroupCaps: groupCaps, LeafCaps: leafCaps}, true, nil
}

type groupUnionSource struct {
	Inputs    []*Body
	Owners    []int
	GroupCaps []prismboolean.ReplayGroupCapFaces
	LeafCaps  []prismboolean.ReplayGroupCapFaces
}

type blindHistoryEntry struct {
	body, source *Body
	operation    *Operation
	kind         OperationKind
	origin       provenance.Ref
}

// blindGroupProof stays in the root because document operation identities
// are root-owned. It is completed before the Cut result becomes visible.
type blindGroupProof struct {
	output, target, group *Body
	origin                provenance.Ref
	groupOrigin           provenance.Ref
	union, cut            *Operation
	history               []blindHistoryEntry
	faces                 *prismboolean.BlindPublishedWitness
	cache                 atomic.Pointer[blindTessBinding]
}

func captureBlindGroupProof(ctx context.Context, output, target, group *Body, leaves []*Body,
	faces *prismboolean.BlindPublishedWitness, budget *proofbound.ReplayBudget) (*blindGroupProof, bool, error) {
	if output == nil || target == nil || group == nil || faces == nil || budget == nil ||
		len(leaves) == 0 || len(leaves) > 4096 || output.n.Doc == nil {
		return nil, false, nil
	}
	capacity := 64 * (len(leaves) + 1)
	if err := budget.ReserveRecords(ctx, uint64(capacity+1)); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(blindHistoryEntry{}))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(blindGroupProof{}))); err != nil {
		return nil, false, err
	}
	// The fixed charge also covers the non-allocating operation binding at commit.
	if err := budget.SpendWork(ctx, 128); err != nil {
		return nil, false, err
	}
	proof := &blindGroupProof{output: output, target: target, group: group, origin: output.n.Origin, groupOrigin: group.n.Origin,
		history: make([]blindHistoryEntry, 0, capacity), faces: faces}
	doc := documentHandle(output.n.Doc)
	doc.n.LiveMu.RLock()
	defer doc.n.LiveMu.RUnlock()
	proof.union = doc.producers[group]
	if proof.union == nil || proof.union.kind != OperationUnion {
		return nil, false, nil
	}
	for i := -1; i < len(leaves); i++ {
		current := target
		if i >= 0 {
			current = leaves[i]
		}
		finished := false
		for range 64 {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return nil, false, err
			}
			if current == nil || current.n.Doc != output.n.Doc {
				return nil, false, nil
			}
			op := doc.producers[current]
			if op == nil || op.doc != output.n.Doc || len(op.outputs) != 1 || op.outputs[0] != current {
				return nil, false, nil
			}
			entry := blindHistoryEntry{body: current, operation: op, kind: op.kind, origin: current.n.Origin}
			switch op.kind {
			case OperationExtrude:
				if len(op.inputs) != 0 {
					return nil, false, nil
				}
				finished = true
			case OperationTransform:
				if len(op.inputs) != 1 || op.inputs[0].Role != operationRoleSource || op.inputs[0].Use != InputConsumed || op.inputs[0].Body == current {
					return nil, false, nil
				}
				entry.source = op.inputs[0].Body
			default:
				return nil, false, nil
			}
			proof.history = append(proof.history, entry)
			if finished {
				break
			}
			current = entry.source
		}
		if !finished {
			return nil, false, nil
		}
	}
	return proof, true, nil
}

func (p *blindGroupProof) acceptsOperation(op *Operation, output *Body) bool {
	return p != nil && p.cut == nil && p.output == output && output.n.Origin == p.origin && op != nil &&
		op.kind == OperationCut && op.doc == output.n.Doc && len(op.outputs) == 1 && op.outputs[0] == output &&
		len(op.inputs) == 2 && op.inputs[0].Body == p.target && op.inputs[0].Role == operationRoleTarget &&
		op.inputs[0].Use == InputConsumed && op.inputs[1].Body == p.group && op.inputs[1].Role == operationRoleTool &&
		op.inputs[1].Use == InputConsumed
}

func (p *blindGroupProof) recheckHistory(ctx context.Context, output, target, group *Body,
	op *Operation, budget *proofbound.ReplayBudget) (bool, error) {
	if p == nil || budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return false, err
	}
	if p.output != output || p.target != target || p.group != group || p.cut == nil || p.cut != op ||
		output.n.Origin != p.origin || group.n.Origin != p.groupOrigin || output.n.Doc == nil {
		return false, nil
	}
	doc := documentHandle(output.n.Doc)
	doc.n.LiveMu.RLock()
	defer doc.n.LiveMu.RUnlock()
	if doc.producers[output] != p.cut || doc.producers[group] != p.union {
		return false, nil
	}
	for _, entry := range p.history {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return false, err
		}
		if entry.body == nil || entry.body.n.Origin != entry.origin || doc.producers[entry.body] != entry.operation || entry.operation == nil ||
			entry.operation.doc != output.n.Doc || entry.operation.kind != entry.kind || len(entry.operation.outputs) != 1 ||
			entry.operation.outputs[0] != entry.body {
			return false, nil
		}
		if entry.kind == OperationExtrude {
			if len(entry.operation.inputs) != 0 {
				return false, nil
			}
		} else if entry.kind != OperationTransform || len(entry.operation.inputs) != 1 ||
			entry.operation.inputs[0].Body != entry.source || entry.operation.inputs[0].Role != operationRoleSource ||
			entry.operation.inputs[0].Use != InputConsumed {
			return false, nil
		}
	}
	return true, ctx.Err()
}
