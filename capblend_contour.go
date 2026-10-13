package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file adapts a built cap band's corner and edge records to the neutral
// contour and closure proofs in internal/capband.

// loopContourDelta is the package function of the same name read under
// cbp's corner rule for loop li: a draft view's contour is the sharp offset
// (cbp.offsetJoins), each walk at its own amount (walkAmounts), so its
// displacement is enclosed over the sharp joins, never over the reflex-corner
// arcs a chamfer's contour holds.
func (cbp capBlendPayload) loopContourDelta(ctx context.Context, li int, loop loopRecord, d, dDelta float64) (float64, error) {
	if cbp.fillet {
		return filletband.LoopContourDelta(ctx, loop, d, dDelta, shellTol)
	}
	if !cbp.draft {
		return capband.LoopContourDelta(ctx, loop, d, dDelta, shellTol)
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	amounts := cbp.walkAmounts(li, cl.walks, d)
	if len(cl.walks) == 1 && cl.walks[0].Closed {
		a, aDelta := amounts[0], dDelta
		if a == 0 {
			aDelta = 0
		}
		return capband.WallCircleDisplacement(cl.walks[0], a, aDelta, shellTol)
	}
	joins, err := cbp.offsetJoins(budget, li, cl, d)
	if err != nil {
		return 0, err
	}
	return capband.AmountsContourDisplacement(cl.walks, capband.ContourJoins(joins), amounts, dDelta, shellTol)
}

// capBandClosure is the neutral proof of the slivers between integrated
// patches and the closed surface they meet.
type capBandClosure = capband.Closure

// capBandClosureOf reads the built band's edge lengths for that proof.
func capBandClosureOf(walks []survey2d.SideWalk, joins []cornerJoin, slantIn, slantOut []*Edge, slantInHeld, slantOutHeld []float64) (capBandClosure, error) {
	in := make([]float64, len(slantIn))
	out := make([]float64, len(slantOut))
	for i, edge := range slantIn {
		in[i] = edge.length
	}
	for i, edge := range slantOut {
		out[i] = edge.length
	}
	closure, ok := capband.ClosureOf(walks, capband.ContourJoins(joins), in, out, slantInHeld, slantOutHeld)
	if !ok {
		return capBandClosure{}, capband.ErrHeldUnbounded
	}
	return closure, nil
}
