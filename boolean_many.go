package decad

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/unionmany"
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
	ctx, err := d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
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
	if err := d.requireSameContainer(operands); err != nil {
		return nil, err
	}
	for _, operand := range operands {
		if err := refuseSheetOperand(operand, "Union"); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body, span, err := unionManyBodies(ctx, d, d.nextProducerID(), operands)
	if err != nil {
		return nil, err
	}
	inputs := make([]OperationInput, len(operands))
	for i, operand := range operands {
		inputs[i] = OperationInput{Body: operand, Role: operationRoleOperand, Use: InputConsumed}
	}
	if err := d.commitOperationSpanIn(ctx, nil, OperationUnion, inputs, body, span, operands...); err != nil {
		return nil, err
	}
	return body, nil
}

// unionManyBodies is the many-operand Union's route chain. It evaluates under
// producer identity ref and returns the result beside the span of producer
// identities it used. It commits nothing and checks no cancellation after the
// last route, so the caller does.
func unionManyBodies(ctx context.Context, d *Document, ref producerID, operands []*Body) (*Body, producerID, error) {
	body, span, err := unionManyRoutes(ctx, d, ref, operands)
	if err == nil || !errors.Is(err, ErrUnsupported) {
		return body, span, err
	}
	// Axis-square faceted solids join exactly on one grid (agent-docs/orthogonal-design.md).
	if joined, ok := orthogonalUnionMany(ctx, d, ref, operands); ok {
		return joined, 1, ctx.Err()
	}
	// Every certified set route refused. A refusal from the union of the prism
	// sections stays the answer. Otherwise try the generic pair chain and keep
	// the first refusal if it fails.
	var sectionRefusal *unionmany.SectionUnionError
	if errors.As(err, &sectionRefusal) || !isManyUnionRefusal(err) {
		return nil, 0, err
	}
	body, span, matched, chainErr := tryPrismUnionChain(ctx, ref, operands)
	if chainErr != nil {
		return nil, 0, chainErr
	}
	if matched {
		return body, span, ctx.Err()
	}
	return nil, 0, err
}

// unionManyRoutes unwraps private Document.n and Body.n fields, supplies the
// handle allocator through envOf, and preserves the caller's producer identity
// for unionmany.Routes. The returned node becomes a root handle for its caller.
func unionManyRoutes(ctx context.Context, d *Document, ref producerID, operands []*Body) (*Body, producerID, error) {
	node, span, err := unionmany.Routes(ctx, envOf(documentNode(d), ref), ref, bodyNodes(operands), verifyAllMesher{})
	return bodyHandle(node), span, err
}

// tryPrismUnionChain unwraps private Body.n fields and supplies the handle
// allocator, typed-error hooks and root invariant predicate to unionmany.
// Its caller has already resolved the document's evaluator tolerances.
func tryPrismUnionChain(ctx context.Context, ref producerID, operands []*Body) (*Body, producerID, bool, error) {
	node, span, matched, err := unionmany.PrismChain(ctx, envOf(operands[0].n.Doc, ref), bodyNodes(operands),
		true, booleanHooks(booleanConfig{}), regroupSurfaces)
	return bodyHandle(node), span, matched, err
}

// isManyUnionRefusal reports whether err is the refusal of a Union of three or
// more bodies that no certified route takes as a set.
func isManyUnionRefusal(err error) bool {
	return err != nil && strings.Contains(err.Error(), "Union of three or more bodies needs a certified")
}

// unionFold joins bodies two at a time, the first with the second, that result
// with the third, and so on, each step a [Union] of two bodies with that Union's
// routes and refusals. The whole chain is run on copies first, so a step that
// refuses leaves every body live and the document as it was; only a chain that
// completes on the copies is run on the bodies themselves.
func unionFold(ctx context.Context, bodies []*Body) (*Body, error) {
	copies := make([]*Body, 0, len(bodies))
	defer func() { removeTools(copies) }()
	for _, b := range bodies {
		c, err := Copy(ctx, b)
		if err != nil {
			return nil, err
		}
		copies = append(copies, c)
	}
	trial, err := foldPairs(ctx, copies)
	removeTools([]*Body{trial})
	if err != nil {
		return nil, err
	}
	return foldPairs(ctx, bodies)
}

// foldPairs unions bodies left to right, each pair through performBoolean. On a
// refusal it returns the body the chain had reached, beside the error.
func foldPairs(ctx context.Context, bodies []*Body) (*Body, error) {
	acc := bodies[0]
	for _, next := range bodies[1:] {
		joined, err := performBoolean(ctx, meshbool.OpUnion, acc, next)
		if err != nil {
			return acc, operationError(OperationUnion, err)
		}
		acc = joined
	}
	return acc, nil
}
