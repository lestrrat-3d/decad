package decad

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// groupBlindVerifyAllWithBudget replays the source-bound mesh only for a public Cut
// output with its exact ordered target/tool history. Each Tessellate call owns
// a new finite meter; it does not share the preceding public Cut's budget.
func groupBlindVerifyAllWithBudget(ctx context.Context, body *Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, bool, error) {
	if budget == nil || body == nil || body.n.Doc == nil || body.n.Owner != body ||
		body.n.Origin.Role != provenance.RoleBody || len(body.n.OperationWarnings) != 0 ||
		!body.n.HasStrictPrismBuildSeal() {
		return nil, false, nil
	}
	sp, ok := body.n.Payload.(stackedrecord.Payload)
	if !ok || len(sp.Slabs) != 2 || len(sp.Interfaces) != 1 || sp.IsGroup() ||
		len(sp.Upstream) == 0 {
		return nil, false, nil
	}
	doc := documentHandle(body.n.Doc)
	doc.n.LiveMu.RLock()
	op := doc.producers[body]
	var host, tool *Body
	if op != nil && op.kind == OperationCut && op.doc == body.n.Doc &&
		len(op.outputs) == 1 && op.outputs[0] == body && len(op.inputs) == 2 &&
		op.inputs[0].Role == "target" && op.inputs[0].Use == InputConsumed &&
		op.inputs[1].Role == "tool" && op.inputs[1].Use == InputConsumed {
		host, tool = op.inputs[0].Body, op.inputs[1].Body
	}
	doc.n.LiveMu.RUnlock()
	if host == nil || tool == nil || host == tool || host.n.Doc != body.n.Doc ||
		tool.n.Doc != body.n.Doc || host.n.Owner != host || tool.n.Owner != tool ||
		len(host.n.OperationWarnings) != 0 || len(tool.n.OperationWarnings) != 0 {
		return nil, true, decaderr.ErrUnsupported
	}
	group, ok := tool.n.Payload.(stackedrecord.Payload)
	if !ok || !group.IsGroup() {
		return nil, true, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, true, err
	}
	direct, err := groupDirectPrismSource(ctx, host, budget)
	if err != nil {
		return nil, true, err
	}
	if !direct {
		return nil, true, decaderr.ErrUnsupported
	}
	hostPayload, ok := host.n.Payload.(prismrecord.Payload)
	if !ok {
		return nil, true, decaderr.ErrUnsupported
	}
	isometric, err := prismplacement.ReplayWorldIsometry(ctx, hostPayload.PlacementOperand(), budget)
	if err != nil {
		return nil, true, err
	}
	if !isometric {
		return nil, true, decaderr.ErrUnsupported
	}
	source, ok, err := groupUnionSourceSnapshot(ctx, tool, budget)
	if err != nil {
		return nil, true, err
	}
	if !ok {
		return nil, true, decaderr.ErrUnsupported
	}
	validHistory, err := body.blindProof.recheckHistory(ctx, body, host, tool, op, budget)
	if err != nil {
		return nil, true, err
	}
	if !validHistory {
		return nil, true, decaderr.ErrUnsupported
	}
	for _, leaf := range source.Inputs {
		direct, err = groupDirectPrismSource(ctx, leaf, budget)
		if err != nil {
			return nil, true, err
		}
		if !direct {
			return nil, true, decaderr.ErrUnsupported
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(source.Inputs))); err != nil {
		return nil, true, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(source.Inputs)), uint64(unsafe.Sizeof((*topo.Body)(nil)))); err != nil {
		return nil, true, err
	}
	leaves := make([]*topo.Body, len(source.Inputs))
	for i, leaf := range source.Inputs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, true, err
		}
		leaves[i] = &leaf.n
	}
	validFaces, err := body.blindProof.faces.Recheck(ctx, &body.n, &host.n, &tool.n, leaves, budget)
	if err != nil {
		return nil, true, err
	}
	if !validFaces {
		return nil, true, decaderr.ErrUnsupported
	}
	validOwners, err := recheckBlindFaceOwners(ctx, &body.n, budget)
	if err != nil {
		return nil, true, err
	}
	if !validOwners {
		return nil, true, decaderr.ErrUnsupported
	}
	mesh, proven, err := stackedrecord.ReplayBlindMeshVolumeWithSource(ctx,
		&body.n, &host.n, chord, budget)
	if err != nil {
		return nil, true, err
	}
	if !proven || mesh.Mesh == nil {
		return nil, true, decaderr.ErrUnsupported
	}
	return mesh.Mesh, true, nil
}

// recheckBlindFaceOwners stays in the root because only it owns Face handles.
func recheckBlindFaceOwners(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if lump == nil {
			return false, nil
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			if shell == nil {
				return false, nil
			}
			for _, face := range shell.Faces {
				if err := budget.SpendWork(ctx, 8); err != nil {
					return false, err
				}
				if face == nil {
					return false, nil
				}
				handle, ok := face.Owner.(*Face)
				if !ok || handle == nil || &handle.n != face {
					return false, nil
				}
			}
		}
	}
	return true, ctx.Err()
}
