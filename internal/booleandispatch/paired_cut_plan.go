package booleandispatch

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RunPairedCutPlan distributes one or two Cuts across two Union source nodes,
// then joins the resulting branches. It checks cancellation at every call.
// The caller authenticates the sources and supplies ordinary Boolean dispatch
// through run, retaining its proof and warning gates. It commits no operation.
// ctx, both sources, firstTool and run must not be nil; secondTool is optional.
func RunPairedCutPlan(ctx context.Context, sources [2]*topo.Body, firstTool, secondTool *topo.Body,
	run func(meshbool.OperationKind, *topo.Body, *topo.Body, float64) (*topo.Body, error)) (*topo.Body, error) {
	var branches [2]*topo.Body
	for i, source := range sources {
		scale := 0.25
		if secondTool != nil && i == 1 {
			scale = 0.0625
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var err error
		branches[i], err = run(meshbool.OpCut, source, firstTool, scale)
		if err != nil {
			return nil, err
		}
		if secondTool != nil {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			branches[i], err = run(meshbool.OpCut, branches[i], secondTool, 0.125)
			if err != nil {
				return nil, err
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	result, err := run(meshbool.OpUnion, branches[0], branches[1], 1)
	if err != nil {
		return nil, err
	}
	return result, ctx.Err()
}
