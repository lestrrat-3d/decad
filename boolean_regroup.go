package decad

import (
	"context"
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type pairedCutAncestry struct {
	sources      [2]*Body
	firstTool    *Body
	union, prior *Operation
}

// pairedCutSources authenticates the exact public operations whose set identity
// permits the private plan. The snapshot owns the producer map and operations.
func pairedCutSources(ctx context.Context, d *Document, target, tool *Body) (pairedCutAncestry, bool) {
	if _, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); suppressed {
		return pairedCutAncestry{}, false
	}
	if d.requireLive(target) != nil || d.requireLive(tool) != nil || target == tool {
		return pairedCutAncestry{}, false
	}
	return pairedCutSnapshot(d.OperationHistory(), target, tool)
}

func pairedCutSnapshot(h *OperationHistory, target, tool *Body) (pairedCutAncestry, bool) {
	var result pairedCutAncestry
	src := pairedHistorySource{h: h}
	if !src.authenticated(target) || !src.authenticated(tool) {
		return result, false
	}
	kind, inputs, ok := boolchain.Expandable(src, &target.n)
	if !ok {
		return result, false
	}
	unionBody := target
	if kind == meshbool.OpCut {
		if len(inputs) != 2 {
			return result, false
		}
		unionBody, result.firstTool = bodyHandle(inputs[0]), bodyHandle(inputs[1])
		result.prior = h.producers[target]
		if !src.authenticated(result.firstTool) {
			return pairedCutAncestry{}, false
		}
		kind, inputs, ok = boolchain.Expandable(src, &unionBody.n)
	}
	if !ok || kind != meshbool.OpUnion || len(inputs) != 2 {
		return pairedCutAncestry{}, false
	}
	result.union = h.producers[unionBody]
	result.sources = [2]*Body{bodyHandle(inputs[0]), bodyHandle(inputs[1])}
	if result.sources[0] == result.sources[1] || !src.authenticated(result.sources[0]) ||
		!src.authenticated(result.sources[1]) {
		return pairedCutAncestry{}, false
	}
	return result, true
}

// commitPairedCut rechecks the public sources while holding the same lock as
// Remove, then publishes the operation and retires its original operands.
// Like other document mutation, concurrent modeling calls are not supported.
func commitPairedCut(ctx context.Context, d *Document, target, tool, body *Body, span, base producerID,
	ancestry pairedCutAncestry, original error) error {
	inputs := operationInputsForBoolean(meshbool.OpCut, target, tool)
	operation := d.operation(OperationCut, inputs, body)
	if err := d.validateOperation(operation, []*Body{target, tool}); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	d.n.LiveMu.Lock()
	defer d.n.LiveMu.Unlock()
	if err := ctx.Err(); err != nil {
		return err
	}
	if !slices.Contains(d.n.LiveSet, &target.n) || !slices.Contains(d.n.LiveSet, &tool.n) ||
		d.n.NextProducer != base {
		return original
	}
	h := &OperationHistory{doc: &d.n, operations: d.operations, producers: d.producers, consumers: d.consumers}
	current, ok := pairedCutSnapshot(h, target, tool)
	if !ok || current != ancestry {
		return original
	}
	d.n.NextProducer += span
	dest := d.destinationLocked(nil, []*Body{target, tool})
	d.retire(target)
	d.retire(tool)
	d.n.LiveSet = append(d.n.LiveSet, &body.n)
	d.placeLocked(&body.n, dest)
	stampOperationWarningKind(body, OperationCut)
	d.appendOperationLocked(operation)
	return nil
}

type pairedHistorySource struct{ h *OperationHistory }

func (s pairedHistorySource) authenticated(body *Body) bool {
	if body == nil || body.n.Doc != s.h.doc {
		return false
	}
	op := s.h.producers[body]
	return op != nil && op.doc == s.h.doc && op.matchesIdentitySeal() && slices.Contains(s.h.operations, op) &&
		len(op.outputs) == 1 && op.outputs[0] == body
}

func (s pairedHistorySource) BooleanProducer(body *topo.Body) (meshbool.OperationKind, []*topo.Body, bool) {
	handle := bodyHandle(body)
	if !s.authenticated(handle) {
		return 0, nil, false
	}
	op := s.h.producers[handle]
	var kind meshbool.OperationKind
	var roles [2]string
	switch op.kind {
	case OperationUnion:
		kind, roles = meshbool.OpUnion, [2]string{"first", "others"}
	case OperationCut:
		kind, roles = meshbool.OpCut, [2]string{operationRoleTarget, operationRoleTool}
	default:
		return 0, nil, false
	}
	if len(op.inputs) != 2 {
		return 0, nil, false
	}
	var inputs []*topo.Body
	for i, input := range op.inputs {
		if input.Body == nil || input.Body.n.Doc != s.h.doc || input.Use != InputConsumed || input.Role != roles[i] ||
			!s.authenticated(input.Body) || !slices.Contains(s.h.consumers[input.Body], op) {
			return 0, nil, false
		}
		inputs = append(inputs, &input.Body.n)
	}
	return kind, inputs, true
}

// pairedCutContinuation uses distributivity of regularized difference over
// union: (L union U) minus H equals (L minus H) union (U minus H). Applying
// the same identity a second time gives the five-step plan. Public history
// authenticates every operand; no geometric sample establishes this identity.
// After the original refusal, the plan makes three or five ordinary calls.
// Those calls share no replay meter and expose no work, record or byte counts.
// This adapter stays in root to bind Document.producers and Body.n handles;
// booleandispatch.RunPairedCutPlan owns the node-only private call sequence.
func pairedCutContinuation(ctx context.Context, d *Document, target, tool *Body, cfg booleanConfig,
	cause error) (*Body, producerID, pairedCutAncestry, bool, error) {
	var empty pairedCutAncestry
	if err := ctx.Err(); err != nil {
		return nil, 0, empty, false, err
	}
	if !coarseTargetCut(meshbool.OpCut, cfg, cause) {
		return nil, 0, empty, false, nil
	}
	ancestry, ok := pairedCutSources(ctx, d, target, tool)
	if !ok {
		return nil, 0, empty, false, nil
	}
	span := producerID(3)
	firstTool := tool
	if ancestry.firstTool != nil {
		span, firstTool = 5, ancestry.firstTool
	}
	base := d.nextProducerID()
	if base < 0 || base > producerID(math.MaxInt)-span {
		return nil, 0, empty, false, nil
	}
	ectx, err := d.evalContext(ctx)
	if err != nil {
		return nil, 0, empty, false, err
	}
	hooks := booleanHooks(cfg)
	step := producerID(0)
	run := func(op meshbool.OperationKind, a, b *topo.Body, scale float64) (*topo.Body, error) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		n, err := booleandispatch.BodyScaled(ectx, op, envOf(documentNode(d), base+step), a, b, hooks, scale)
		if err != nil {
			return nil, err
		}
		step++
		if len(n.OperationWarnings) != 0 {
			return nil, ErrUnsupported
		}
		return n, ctx.Err()
	}
	sources := [2]*topo.Body{&ancestry.sources[0].n, &ancestry.sources[1].n}
	var secondTool *topo.Body
	if ancestry.firstTool != nil {
		secondTool = &tool.n
	}
	n, err := booleandispatch.RunPairedCutPlan(ctx, sources, &firstTool.n, secondTool, run)
	if err != nil {
		return pairedCutFailure(ctx, err)
	}
	if step != span {
		return nil, 0, empty, false, ErrBooleanFailed
	}
	body := bodyHandle(n)
	chord, _, err := booloperand.PairChord(ctx, &target.n, &tool.n)
	if err != nil {
		return pairedCutFailure(ctx, err)
	}
	mesh, _, err := tessellateBooleanOperand(ctx, body, chord)
	if err != nil {
		return pairedCutFailure(ctx, err)
	}
	volume, bound := body.n.Volume.Value.Base(), body.n.Volume.Bound.Base()
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() || bound < 0 || !(volume > bound) ||
		math.IsInf(volume, 0) || math.IsInf(bound, 0) {
		return nil, 0, empty, false, nil
	}
	return body, span, ancestry, true, ctx.Err()
}

func pairedCutFailure(ctx context.Context, err error) (*Body, producerID, pairedCutAncestry, bool, error) {
	if ctx.Err() != nil {
		return nil, 0, pairedCutAncestry{}, false, ctx.Err()
	}
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) || regroupSurfaces(err) {
		return nil, 0, pairedCutAncestry{}, false, err
	}
	return nil, 0, pairedCutAncestry{}, false, nil
}

// coarseTargetCut admits only the target-side held-contact staging refusal.
// Every nested tag must agree; a joined or wrapped conflicting refusal cannot
// authorize a rewrite of the caller's operation.
func coarseTargetCut(op meshbool.OperationKind, cfg booleanConfig, cause error) bool {
	if op != meshbool.OpCut || cfg.matchingFaces || cause == nil ||
		errors.Is(cause, context.Canceled) || errors.Is(cause, context.DeadlineExceeded) {
		return false
	}
	expected, ok := asExpectedBoolean(cause)
	if !ok || expected.Kind != meshbool.BooleanExpectedStaging || expected.Operand != 0 {
		return false
	}
	var held *meshbool.CoarseHeldContactError
	if !errors.As(expected.Err, &held) || held.Operand != 0 {
		return false
	}
	return consistentCoarseTags(cause, 0)
}

func consistentCoarseTags(err error, depth int) bool {
	if err == nil {
		return true
	}
	if depth >= 128 {
		return false
	}
	switch e := err.(type) {
	case *meshbool.BooleanExpectedError:
		if e.Kind != meshbool.BooleanExpectedStaging || e.Operand != 0 {
			return false
		}
	case *meshbool.CoarseHeldContactError:
		if e.Operand != 0 {
			return false
		}
	}
	if _, ok := err.(interface{ Unwrap() []error }); ok {
		return false
	}
	if e, ok := err.(interface{ Unwrap() error }); ok {
		return consistentCoarseTags(e.Unwrap(), depth+1)
	}
	return true
}

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
		if len(in) != 2 || op.inputs[0].Role != operationRoleTarget || op.inputs[1].Role != operationRoleTool {
			return 0, nil, false
		}
	}
	return kind, in, true
}
