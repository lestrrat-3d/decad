package facetedrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replaySourceDiameterUpper(ctx context.Context, verts []r3.Vec, departure float64, b *proofbound.ReplayBudget) (float64, error) {
	if b == nil || len(verts) == 0 || departure < 0 || proofbound.IsNonFinite(departure) {
		return 0, errBrakeReplayInvalidBoundedSourceDiameter
	}
	lo, hi := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}, [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for _, v := range verts {
		if err := b.SpendWork(ctx, 3); err != nil {
			return 0, err
		}
		for i, x := range [3]float64{v.X, v.Y, v.Z} {
			if proofbound.IsNonFinite(x) {
				return 0, errBrakeReplayInvalidSourceVertex
			}
			lo[i] = min(lo[i], x)
			hi[i] = max(hi[i], x)
		}
	}
	sum, err := proof.ReplayFloatRat(ctx, 0, b)
	if err != nil {
		return 0, err
	}
	for i := range 3 {
		a, err := proof.ReplayFloatRat(ctx, hi[i], b)
		if err != nil {
			return 0, err
		}
		c, err := proof.ReplayFloatRat(ctx, lo[i], b)
		if err != nil {
			return 0, err
		}
		d, err := proof.ReplayRatArithmetic(ctx, a, c, '-', b)
		if err != nil {
			return 0, err
		}
		square, err := proof.ReplayRatArithmetic(ctx, d, d, '*', b)
		if err != nil {
			return 0, err
		}
		sum, err = proof.ReplayRatArithmetic(ctx, sum, square, '+', b)
		if err != nil {
			return 0, err
		}
	}
	length, err := proof.ReplaySqrtDyadicRat(ctx, sum, b)
	if err != nil {
		return 0, err
	}
	return proofbound.AbsSumUpper(length, proofbound.ProductUpper(2, departure)), nil
}
