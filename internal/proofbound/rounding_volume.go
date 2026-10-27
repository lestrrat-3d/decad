package proofbound

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/r3"
)

// PerturbedAreaAndRoundingVolumeContext returns PerturbedAreaUpperContext's
// global area bound and a swept-volume bound using each vertex's own rounding
// radius. ctx MUST NOT be nil; rounding must have one entry per vertex.
// tris must include every pre-round facet, including facets collapsed by the
// final weld. Vertices are their final held positions, and each rounding[i]
// must bound that vertex's distance from every exact corner it represents.
// global bounds every such distance and supplies the global area bound.
// Invalid scalar claims or non-finite referenced vertices return +Inf.
//
// On triangle t, put d_t = max(rounding at its three corners). Every point
// moves at speed at most d_t along the straight homotopy, by convexity.
// PerturbedTriangleAreaUpper with d_t bounds its area throughout that motion.
// Its swept volume is therefore at most d_t times that area bound. Summing
// these nonnegative terms covers every point whose winding changes, even if
// an intermediate surface folds or a final facet collapses. This bounds the
// occupied symmetric difference, without subtracting signed volumes.
func PerturbedAreaAndRoundingVolumeContext(ctx context.Context, verts []r3.Vec,
	tris [][3]int, global float64, rounding []float64,
) (float64, float64, error) {
	if err := ctx.Err(); err != nil {
		return 0, 0, err
	}
	if len(rounding) != len(verts) || IsNonFinite(global) || global < 0 {
		return math.Inf(1), math.Inf(1), nil
	}
	budget := NewWorkBudget(ctx)
	terms, err := perturbedTriangleTerms(
		budget, verts, tris, orderedwork.EachWorkers(len(tris), perturbedParallelMin))
	if err != nil {
		return 0, 0, err
	}
	area, volume, volumeTerms := 0.0, 0.0, 0
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return 0, 0, err
		}
		term := terms[i]
		if !term.ok {
			return math.Inf(1), math.Inf(1), nil
		}
		delta := 0.0
		for _, v := range tri {
			d := rounding[v]
			if IsNonFinite(d) || d < 0 {
				return math.Inf(1), math.Inf(1), nil
			}
			delta = max(delta, d)
		}
		area += AbsSumUpper(term.area, perturbedAllow(global, term.lu, term.lv))
		if delta == 0 {
			continue
		}
		localArea := AbsSumUpper(term.area, perturbedAllow(delta, term.lu, term.lv))
		volume += SweptVolumeAllow(delta, localArea)
		volumeTerms++
	}
	if err := budget.Err(); err != nil {
		return 0, 0, err
	}
	area = UpRound(area + SumSlop(len(tris), area))
	if volume > 0 {
		volume = UpRound(volume + SumSlop(volumeTerms, volume))
	}
	return area, volume, nil
}
