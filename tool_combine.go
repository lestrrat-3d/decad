package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// combineTools applies op between b and each tool in order and commits the
// final body in place of b. When merge is set the tools are first united with
// one another in order, and b meets the single merged tool: tools that share
// planes meet each other on the prism path before they meet b. Nothing is registered until every step has
// succeeded and accept, when not nil, has approved the result, so a failure
// leaves b live and the document unchanged. The caller removes the tools, which
// stay live until it does.
func combineTools(ctx context.Context, d *Document, b *Body, op meshbool.OperationKind, tools []*Body, merge bool,
	accept func(result *Body) error) (*Body, error) {
	start := d.nextProducerID()
	ref := start
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
		acc = next
		ref++
	}
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
	d.commitSpan(acc, ref-start, b)
	return acc, nil
}

// removeTools retires the tool bodies a feature registered. A tool is
// live until the feature returns, so a removal cannot fail except on a body
// already gone.
func removeTools(tools []*Body) {
	for _, t := range tools {
		_ = Remove(t)
	}
}
