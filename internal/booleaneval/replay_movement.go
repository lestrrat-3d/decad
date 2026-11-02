package booleaneval

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replayMoveRounding(ctx context.Context, b *proofbound.ReplayBudget, v, w, move, centre r3.Vec, k float64, grown bool) (float64, error) {
	if err := b.ReserveBytes(ctx, 256); err != nil {
		return 0, err
	}
	total := new(big.Rat)
	var kr *big.Rat
	var err error
	if grown {
		kr, err = proof.ReplayFloatRat(ctx, k, b)
		if err != nil {
			return 0, err
		}
	}
	for i := range 3 {
		x, y, s, c := [3]float64{v.X, v.Y, v.Z}[i], [3]float64{w.X, w.Y, w.Z}[i], [3]float64{move.X, move.Y, move.Z}[i], [3]float64{centre.X, centre.Y, centre.Z}[i]
		xr, err := proof.ReplayFloatRat(ctx, x, b)
		if err != nil {
			return 0, err
		}
		yr, err := proof.ReplayFloatRat(ctx, y, b)
		if err != nil {
			return 0, err
		}
		sr, err := proof.ReplayFloatRat(ctx, s, b)
		if err != nil {
			return 0, err
		}
		delta, err := proof.ReplayRatArithmetic(ctx, yr, xr, '-', b)
		if err != nil {
			return 0, err
		}
		delta, err = proof.ReplayRatArithmetic(ctx, delta, sr, '-', b)
		if err != nil {
			return 0, err
		}
		if grown {
			cr, err := proof.ReplayFloatRat(ctx, c, b)
			if err != nil {
				return 0, err
			}
			term, err := proof.ReplayRatArithmetic(ctx, xr, cr, '-', b)
			if err != nil {
				return 0, err
			}
			term, err = proof.ReplayRatArithmetic(ctx, term, kr, '*', b)
			if err != nil {
				return 0, err
			}
			delta, err = proof.ReplayRatArithmetic(ctx, delta, term, '-', b)
			if err != nil {
				return 0, err
			}
		}
		square, err := proof.ReplayRatArithmetic(ctx, delta, delta, '*', b)
		if err != nil {
			return 0, err
		}
		total, err = proof.ReplayRatArithmetic(ctx, total, square, '+', b)
		if err != nil {
			return 0, err
		}
	}
	return proof.ReplaySqrtDyadicRat(ctx, total, b)
}
