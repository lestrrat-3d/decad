package decad

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// brakeBooleanProducer reads only a binary committed producer with bounded work.
func brakeBooleanProducer(ctx context.Context, d *Document, n *topo.Body, budget *proofbound.ReplayBudget) (meshbool.OperationKind, []*topo.Body, bool, error) {
	if d == nil || n == nil || budget == nil {
		return 0, nil, false, errBrakeMissingReplayHistory
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, nil, false, err
	}
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	op := d.producers[bodyHandle(n)]
	if op == nil {
		return 0, nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, nil, false, err
	}
	if op.doc != &d.n || len(op.inputs) != 2 || len(op.outputs) != 1 || op.outputs[0] != bodyHandle(n) {
		return 0, nil, false, nil
	}
	var kind meshbool.OperationKind
	switch op.kind {
	case OperationCut:
		kind = meshbool.OpCut
	case OperationUnion:
		kind = meshbool.OpUnion
	case OperationIntersect:
		kind = meshbool.OpIntersect
	default:
		return 0, nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, 2); err != nil {
		return 0, nil, false, err
	}
	if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof((*topo.Body)(nil)))); err != nil {
		return 0, nil, false, err
	}
	in := make([]*topo.Body, 2)
	for i := range 2 {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, nil, false, err
		}
		input := op.inputs[i]
		if input.Use != InputConsumed || input.Body == nil {
			return 0, nil, false, nil
		}
		if kind == meshbool.OpCut && ((i == 0 && input.Role != "target") || (i == 1 && input.Role != "tool")) {
			return 0, nil, false, nil
		}
		in[i] = &input.Body.n
	}
	return kind, in, true, nil
}

// brakeExpandable applies the ordinary warning rule with bounded traversal.
func brakeExpandable(ctx context.Context, d *Document, n *topo.Body, budget *proofbound.ReplayBudget) (meshbool.OperationKind, []*topo.Body, bool, error) {
	if n == nil || budget == nil {
		return 0, nil, false, errBrakeMissingReplaySource
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, nil, false, err
	}
	p, ok := n.Payload.(facetedrecord.Payload)
	if !ok || p.PointCone != nil || len(n.OperationWarnings) > 8 {
		return 0, nil, false, nil
	}
	for _, warning := range n.OperationWarnings {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, nil, false, err
		}
		if warning.Code != opdiagnostic.CodeChainAllowanceApplied || warning.Change != opdiagnostic.ChangeUnknown {
			return 0, nil, false, nil
		}
	}
	return brakeBooleanProducer(ctx, d, n, budget)
}
