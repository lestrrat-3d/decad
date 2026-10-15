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
func (cbp capBlendPayload) LoopContourDelta(ctx context.Context, li int, loop loopRecord, d, dDelta float64) (float64, error) {
	if cbp.Fillet {
		return filletband.LoopContourDelta(ctx, loop, d, dDelta, shellTol)
	}
	if !cbp.Draft {
		return capband.LoopContourDelta(ctx, loop, d, dDelta, shellTol)
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	amounts := cbp.WalkAmounts(li, cl.Walks, d)
	if len(cl.Walks) == 1 && cl.Walks[0].Closed {
		a, aDelta := amounts[0], dDelta
		if a == 0 {
			aDelta = 0
		}
		return capband.WallCircleDisplacement(cl.Walks[0], a, aDelta, shellTol)
	}
	joins, err := cbp.OffsetJoins(budget, li, cl, d)
	if err != nil {
		return 0, err
	}
	return capband.AmountsContourDisplacement(cl.Walks, capband.ContourJoins(joins), amounts, dDelta, shellTol)
}

// capBandClosure is the neutral proof of the slivers between integrated
// patches and the closed surface they meet.
type capBandClosure = capband.Closure

// capBandClosureOf reads the built band's edge lengths for that proof.
func capBandClosureOf(walks []survey2d.SideWalk, joins []cornerJoin, slantIn, slantOut []*Edge, slantInHeld, slantOutHeld []float64) (capBandClosure, error) {
	in := make([]float64, len(slantIn))
	out := make([]float64, len(slantOut))
	for i, edge := range slantIn {
		in[i] = edge.n.Length
	}
	for i, edge := range slantOut {
		out[i] = edge.n.Length
	}
	closure, ok := capband.ClosureOf(walks, capband.ContourJoins(joins), in, out, slantInHeld, slantOutHeld)
	if !ok {
		return capBandClosure{}, capband.ErrHeldUnbounded
	}
	return closure, nil
}
