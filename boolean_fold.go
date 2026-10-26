package decad

import (
	"context"
	"strings"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

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
