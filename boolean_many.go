package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// performUnionMany validates every input before evaluation and commits the
// single result only after its complete proof and cancellation check. The
// certified pattern evaluators construct all tools in one operation.
func performUnionMany(ctx context.Context, operands []*Body) (*Body, error) {
	a := operands[0]
	if a == nil || a.n.Doc == nil {
		return nil, fmt.Errorf("%w: the first operand belongs to no document", ErrDegenerate)
	}
	d := documentHandle(a.n.Doc)
	seen := make(map[*Body]struct{}, len(operands))
	for _, operand := range operands {
		if err := d.requireLive(operand); err != nil {
			return nil, err
		}
		if _, duplicate := seen[operand]; duplicate {
			return nil, fmt.Errorf("%w: Union needs distinct bodies", ErrDegenerate)
		}
		seen[operand] = struct{}{}
	}
	for _, operand := range operands {
		if err := refuseSheetOperand(operand, "Union"); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	if body, matched, err := tryPrismUnionMany(ctx, d, ref, operands); matched || err != nil {
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		d.commit(body, operands...)
		return body, nil
	}
	tools := operands[1:]
	body, matched, err := tryLoftMirrorCylinderPairs(ctx, d, ref, a, tools, nil)
	if !matched && err == nil {
		body, matched, err = tryRevolveLoftSectorUnionMany(ctx, d, ref, operands)
	}
	if !matched && err == nil {
		body, matched, err = tryPrismPatternUnionMany(ctx, d, ref, operands)
	}
	span := producerID(1)
	if !matched && err == nil {
		body, span, matched, err = tryPrismUnionChain(ctx, ref, operands)
	}
	if !matched && err == nil {
		body, err = spiralCombineBody(ctx, d, ref, a, tools)
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commitSpan(body, span, operands...)
	return body, nil
}

// tryPrismUnionChain gives prism inputs that miss the specialized many-body
// paths the same pair evaluator as two-body Union. Every intermediate body is
// transient. The caller commits only the final certified result and reserves
// every producer identity used by its intermediate features.
func tryPrismUnionChain(ctx context.Context, ref producerID, operands []*Body) (*Body, producerID, bool, error) {
	for _, operand := range operands {
		if _, ok := operand.payload.(prismPayload); !ok {
			return nil, 0, false, nil
		}
	}
	result := operands[0]
	for i, operand := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, 0, true, err
		}
		var err error
		result, err = booleanBody(ctx, meshbool.OpUnion, result, operand, ref+producerID(i))
		if err != nil {
			return nil, 0, true, err
		}
	}
	return result, producerID(len(operands) - 1), true, nil
}
