package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
)

// combineTools applies op between b and each tool in order and commits the
// final body in place of b. When merge is set the tools are first united with
// one another in order, and b meets the single merged tool: tools that share
// planes meet each other on the prism path before they meet b. Nothing is registered until every step has
// succeeded and accept, when not nil, has approved the result, so a failure
// leaves b live and the document unchanged. The tools remain private to this call.
func combineTools(ctx context.Context, d *Document, kind OperationKind, b *Body, op meshbool.OperationKind,
	tools []*Body, merge bool,
	accept func(result *Body) error, capture *operationHistoryCapture,
	additionalInputs ...OperationInput) (*Body, error) {
	if err := d.requireSameContainer([]*Body{b}); err != nil {
		return nil, err
	}
	privateSpan := capture.producerSpan(d)
	start := d.nextProducerID() + privateSpan
	ref := start
	var warnings opdiagnostic.CompositeSummaries
	if merge && len(tools) > 1 {
		merged := tools[0]
		for _, tool := range tools[1:] {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			next, err := booleanBody(ctx, meshbool.OpUnion, merged, tool, ref)
			if err != nil {
				return nil, err
			}
			warnings.Add(next.n.OperationWarnings, int(kind))
			merged = next
			ref++
		}
		tools = []*Body{merged}
	}
	acc := b
	for _, tool := range tools {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		next, err := booleanBody(ctx, op, acc, tool, ref)
		if err != nil {
			return nil, err
		}
		warnings.Add(next.n.OperationWarnings, int(kind))
		acc = next
		ref++
	}
	acc.n.OperationWarnings = warnings.Records()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if accept != nil {
		if err := accept(acc); err != nil {
			return nil, err
		}
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	inputs := append([]OperationInput{{Body: b, Role: operationRoleTarget, Use: InputConsumed}}, additionalInputs...)
	if err := d.commitOperationSpanPrefixIn(ctx, nil, kind, inputs, acc, privateSpan, ref-start, b); err != nil {
		return nil, err
	}
	return acc, nil
}

// removeTools retires the tool bodies a feature registered. A tool is
// live until the feature returns, so a removal cannot fail except on a body
// already gone.
func removeTools(tools []*Body) {
	for _, t := range tools {
		if t == nil || t.n.Doc == nil {
			continue
		}
		d := documentHandle(t.n.Doc)
		d.n.LiveMu.Lock()
		if slices.Contains(d.n.LiveSet, bodyNode(t)) {
			d.retire(t)
		}
		d.n.LiveMu.Unlock()
	}
}
