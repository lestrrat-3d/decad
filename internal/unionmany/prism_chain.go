package unionmany

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// PrismChain joins prism and exact straight-source Loft inputs through the
// ordinary two-body Union dispatcher, left to right. At least one of the first
// two inputs must be a prism. Every intermediate is transient; the caller
// commits the final result and reserves the returned producer span.
// regroupThree permits the three-source retry in prism-boolean design §4.7.
// A miss returns matched=false; a pair refusal or cancellation returns an error.
// ctx must carry the document's evaluator tolerances. env.Doc, env.Alloc,
// surface and every operand must not be nil; operands must contain at least
// three bodies belonging to env.Doc, and hooks must supply ordinary dispatch.
func PrismChain(ctx context.Context, env topo.Env, operands []*topo.Body, regroupThree bool,
	hooks booleandispatch.Hooks, surface func(error) bool) (*topo.Body, provenance.Producer, bool, error) {
	firstPrism := false
	for i, operand := range operands {
		if _, ok := operand.Payload.(prismrecord.Payload); ok {
			if i < 2 {
				firstPrism = true
			}
			continue
		}
		loft, ok := operand.Payload.(loftrecord.Payload)
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
	var warnings opdiagnostic.UnionSummaries
	for i, operand := range operands[1:] {
		if err := ctx.Err(); err != nil {
			return nil, 0, true, err
		}
		pairEnv := env
		pairEnv.Ref += provenance.Producer(i)
		var err error
		result, err = booleandispatch.Body(ctx, meshbool.OpUnion, pairEnv, result, operand, hooks)
		if err != nil {
			var held *meshbool.CoarseHeldContactError
			if regroupThree && len(operands) == 3 && i == 1 && errors.As(err, &held) && held.Operand == 0 {
				// The original source sets associate without reusing the coarse rim.
				// Both alternate pairs retain every ordinary proof and refusal gate.
				nodes := [3]*topo.Body{operands[0], operands[1], operands[2]}
				node, span, ok, retryErr := booleandispatch.RegroupUnionThree(ctx, nodes, env, hooks, surface)
				if retryErr != nil {
					return nil, 0, true, retryErr
				}
				if ok {
					return node, span, true, nil
				}
			}
			return nil, 0, true, err
		}
		warnings.AddPair(result.OperationWarnings, i+1)
	}
	result.OperationWarnings = warnings.Records()
	return result, provenance.Producer(len(operands) - 1), true, nil
}
