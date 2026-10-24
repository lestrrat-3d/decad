package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	d.commitSpan(body, span, operands...)
	return body, nil
}

// unionManyBodies is the many-operand Union's route chain over operands that
// need not be live: Patterned's and PatternJoin's instances are built and never
// registered. It evaluates under producer identity ref and returns the result
// beside the span of producer identities it used. It commits nothing and
// checks no cancellation after the last route, so the caller does.
func unionManyBodies(ctx context.Context, d *Document, ref producerID, operands []*Body) (*Body, producerID, error) {
	body, span, err := unionManyRoutes(ctx, d, ref, operands)
	if err == nil || !errors.Is(err, ErrUnsupported) {
		return body, span, err
	}
	// Every route above refused. A set of prisms with a free-form section can
	// still join through the union of their sections, which reads a root arc
	// that lies on the blank's rim and any number of copies. A miss or a refusal
	// there leaves the first refusal as the answer.
	node, matched, aerr := axialsection.UnionManyPrisms(ctx, envOf(documentNode(d), ref), bodyNodes(operands))
	if aerr != nil || !matched {
		return nil, 0, err
	}
	return bodyHandle(node), 1, ctx.Err()
}

// unionManyRoutes is unionManyBodies' route chain.
func unionManyRoutes(ctx context.Context, d *Document, ref producerID, operands []*Body) (*Body, producerID, error) {
	a := operands[0]
	if node, matched, err := prismboolean.UnionMany(ctx, envOf(documentNode(d), ref), bodyNodes(operands)); matched || err != nil {
		if err != nil {
			return nil, 0, err
		}
		return bodyHandle(node), 1, ctx.Err()
	}
	if node, matched, err := axialsection.UnionMany(ctx, envOf(documentNode(d), ref), bodyNodes(operands)); matched || err != nil {
		if err != nil {
			return nil, 0, err
		}
		return bodyHandle(node), 1, ctx.Err()
	}
	tools := operands[1:]
	node, matched, err := loftcylinder.MirrorPairs(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
		bodyNode(a), bodyNodes(tools), nil)
	body := bodyHandle(node)
	if !matched && err == nil {
		var node *topo.Body
		node, matched, err = loftcylinder.UnionTeeth(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
			bodyNode(a), bodyNodes(tools))
		body = bodyHandle(node)
	}
	if !matched && err == nil {
		var node *topo.Body
		node, matched, err = revolveshell.SectorUnionMany(ctx, envOf(documentNode(d), ref), bodyNodes(operands))
		body = bodyHandle(node)
	}
	if !matched && err == nil {
		var node *topo.Body
		node, matched, err = prismpattern.UnionMany(ctx, envOf(documentNode(d), ref), verifyAllMesher{},
			bodyNodes(operands))
		body = bodyHandle(node)
	}
	if !matched && err == nil {
		var node *topo.Body
		node, matched, err = prismboolean.StraightLoftUnionMany(ctx, envOf(documentNode(d), ref), bodyNodes(operands))
		body = bodyHandle(node)
	}
	span := producerID(1)
	if !matched && err == nil {
		body, span, matched, err = tryPrismUnionChain(ctx, ref, operands)
	}
	if err != nil {
		return nil, 0, err
	}
	if !matched {
		return nil, 0, fmt.Errorf("%w: Union of three or more bodies needs a certified prism, "+
			"axial or copied-tooth set; join them two at a time", ErrUnsupported)
	}
	return body, span, ctx.Err()
}

// tryPrismUnionChain gives prism inputs and exact straight-source Loft inputs
// the same pair evaluator as two-body Union. Every intermediate body is
// transient. The caller commits only the final certified result and reserves
// every producer identity used by its intermediate features.
func tryPrismUnionChain(ctx context.Context, ref producerID, operands []*Body) (*Body, producerID, bool, error) {
	firstPrism := false
	for i, operand := range operands {
		if _, ok := operand.n.Payload.(prismPayload); ok {
			if i < 2 {
				firstPrism = true
			}
			continue
		}
		loft, ok := operand.n.Payload.(loftPayload)
		if !ok {
			return nil, 0, false, nil
		}
		if _, ok, err := loftrecord.ExactAxialPrism(ctx, loft); err != nil {
			return nil, 0, true, err
		} else if !ok {
			return nil, 0, false, nil
		}
	}
	if !firstPrism {
		return nil, 0, false, nil
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
