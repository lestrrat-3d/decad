package decad

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// regroupBoolean tries the chain regroup for a refused two-body Boolean
// (agent-docs/boolean-chain-regroup-design.md). ok=false means the caller returns cause
// unchanged. A non-nil error is cancellation or a surfaced invariant break. It stays
// in root because it reads Document.producers and maps BooleanError codes, both
// root-only.
func regroupBoolean(ctx context.Context, d *Document, op meshbool.OperationKind, a, b *Body,
	cfg booleanConfig, cause error) (*Body, producerID, bool, error) {
	if !errors.Is(cause, ErrUnsupported) || errors.Is(cause, context.Canceled) ||
		errors.Is(cause, context.DeadlineExceeded) || cfg.matchingFaces {
		return nil, 0, false, nil
	}
	chain, ok := boolchain.Flatten(op, &a.n, &b.n, historyProducers{d})
	if !ok {
		return nil, 0, false, nil
	}
	ectx, err := d.evalContext(ctx)
	if err != nil {
		return nil, 0, false, err
	}
	n, span, ok, err := booleandispatch.Regroup(ectx, chain, envOf(documentNode(d), d.nextProducerID()),
		booleanHooks(cfg), regroupSurfaces)
	if err != nil || !ok {
		return nil, 0, false, err
	}
	return bodyHandle(n), span, true, nil
}

// regroupSurfaces reports whether a private step's error is an invariant break that
// must surface instead of falling back to the caller's refusal.
func regroupSurfaces(err error) bool {
	if be, ok := errors.AsType[*BooleanError](err); ok {
		return be.Code == BooleanEvaluatorFailure
	}
	return errors.Is(err, ErrBooleanFailed)
}

// historyProducers implements boolchain.Source over d's operation records.
type historyProducers struct{ d *Document }

// BooleanProducer reports the Cut, Union or Intersect record that produced n and its
// consumed inputs: a Cut's target then its tool, otherwise in recorded order.
func (h historyProducers) BooleanProducer(n *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	h.d.n.LiveMu.RLock()
	op := h.d.producers[bodyHandle(n)]
	h.d.n.LiveMu.RUnlock()
	if op == nil {
		return 0, nil, false
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
		return 0, nil, false
	}
	var in []*topo.Body
	for _, input := range op.inputs {
		if input.Use == InputConsumed {
			in = append(in, &input.Body.n)
		}
	}
	if kind == meshbool.OpCut {
		if len(in) != 2 || op.inputs[0].Role != "target" || op.inputs[1].Role != "tool" {
			return 0, nil, false
		}
	}
	return kind, in, true
}
