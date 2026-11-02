package prismrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CircleCurveBoundSignedReplay evaluates the ordinary circular curve bound
// where frame and placement are exact signed coordinate bases. This proves
// zero map stretch and zero axis tilt without exact matrix construction.
func (pp Payload) CircleCurveBoundSignedReplay(ctx context.Context, cu, cv, z, zDelta, centerDelta,
	radius, radiusBound float64, center, axis r3.Vec, budget *proofbound.ReplayBudget,
) (float64, bool, error) {
	if budget == nil {
		return 0, false, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return 0, false, err
	}
	basis := pp.Xform.Basis()
	normal := pp.Dir(0, 0, 1)
	if !replaySignedBasis([3]r3.Vec{pp.Frame.U(), pp.Frame.V(), pp.Frame.N()}) ||
		!replaySignedBasis([3]r3.Vec{basis.EX, basis.EY, basis.EZ}) ||
		!(replaySameVec(axis, normal) || replaySameVec(axis, normal.Scale(-1))) {
		return 0, false, decaderr.ErrUnsupported
	}
	held, lift, err := pp.LiftedVertexReplay(ctx, cu, cv, z, budget)
	if err != nil {
		return 0, false, err
	}
	if !replaySameVec(held, center) {
		return 0, false, decaderr.ErrUnsupported
	}
	reach := proofbound.AbsSumUpper(radius, radiusBound)
	gap := proofbound.AbsSumUpper(
		proofbound.ProductUpper(2, proofbound.ProductUpper(reach, 0)),
		radiusBound,
		proofbound.ProductUpper(0, reach),
	)
	displaced := proofbound.AbsSumUpper(zDelta, proofbound.ProductUpper(2, pp.SectionDelta), centerDelta)
	bound := proofbound.AbsSumUpper(lift, gap,
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, 0), displaced))
	if proofbound.IsNonFinite(bound) || !(proofbound.ProductUpper(2, bound) < radius) {
		return math.Inf(1), false, nil
	}
	return bound, true, nil
}

func replaySignedBasis(axes [3]r3.Vec) bool {
	seen := [3]bool{}
	for _, v := range axes {
		axis := -1
		switch v {
		case r3.Vec{X: 1}, r3.Vec{X: -1}:
			axis = 0
		case r3.Vec{Y: 1}, r3.Vec{Y: -1}:
			axis = 1
		case r3.Vec{Z: 1}, r3.Vec{Z: -1}:
			axis = 2
		}
		if axis < 0 || seen[axis] {
			return false
		}
		seen[axis] = true
	}
	return true
}

func replaySameVec(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) &&
		math.Float64bits(a.Y) == math.Float64bits(b.Y) &&
		math.Float64bits(a.Z) == math.Float64bits(b.Z)
}
