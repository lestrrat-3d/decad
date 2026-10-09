package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file adapts a built cap band's corner and edge records to the neutral
// contour and closure proofs in internal/capband.

func capContourJoins(joins []cornerJoin) []capcontour.Join {
	readings := make([]capcontour.Join, len(joins))
	for i, j := range joins {
		readings[i] = capcontour.Join{
			Arc: j.arc, G1: j.g1, VU: j.vU, VV: j.vV,
			M: j.m, PA: j.pA, PB: j.pB,
		}
	}
	return readings
}

// loopContourDelta re-derives one loop's contour displacement from its record
// for readings that hold no built band. It uses the same walks and joins as
// buildCapBand.
func loopContourDelta(ctx context.Context, loop loopRecord, d, dDelta float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return 0, err
	}
	if len(cl.walks) == 1 && cl.walks[0].Closed {
		return capband.WholeCircleDisplacement(cl.walks[0], d, dDelta, shellTol)
	}
	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return 0, err
	}
	return capband.ContourDisplacement(cl.walks, capContourJoins(joins), d, dDelta, shellTol)
}

// loopContourDelta is the package function of the same name read under
// cbp's corner rule for loop li: a draft view's contour is the sharp offset
// (cbp.offsetJoins), each walk at its own amount (walkAmounts), so its
// displacement is enclosed over the sharp joins, never over the reflex-corner
// arcs a chamfer's contour holds.
func (cbp capBlendPayload) loopContourDelta(ctx context.Context, li int, loop loopRecord, d, dDelta float64) (float64, error) {
	if cbp.fillet {
		return filletContourDelta(ctx, loop, d, dDelta)
	}
	if !cbp.draft {
		return loopContourDelta(ctx, loop, d, dDelta)
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
	return capband.AmountsContourDisplacement(cl.walks, capContourJoins(joins), amounts, dDelta, shellTol)
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
	closure, ok := capband.ClosureOf(walks, capContourJoins(joins), in, out, slantInHeld, slantOutHeld)
	if !ok {
		return capBandClosure{}, capband.ErrHeldUnbounded
	}
	return closure, nil
}
