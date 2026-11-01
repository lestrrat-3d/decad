package facetproof

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func FacetedExtremeErrorReplay(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, beta, B []float64, lo, hi r3.Vec) (float64, error) {
	m := replayExactMeter{budget}
	if err := budget.ReserveBytesMul(1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
		return 0, err
	}
	worst := new(big.Rat)
	coordinate := func(v r3.Vec, axis int) float64 {
		switch axis {
		case 0:
			return v.X
		case 1:
			return v.Y
		default:
			return v.Z
		}
	}
	for axis := range 3 {
		m0, err := proof.ReplayFloatRat(ctx, coordinate(lo, axis), m)
		if err != nil {
			return 0, err
		}
		m1, err := proof.ReplayFloatRat(ctx, coordinate(hi, axis), m)
		if err != nil {
			return 0, err
		}
		var reach, held [2]*big.Rat
		for i, v := range verts {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			c, err := proof.ReplayFloatRat(ctx, coordinate(v, axis), m)
			if err != nil {
				return 0, err
			}
			bound, err := proof.ReplayFloatRat(ctx, B[i], m)
			if err != nil {
				return 0, err
			}
			own, err := proof.ReplayFloatRat(ctx, beta[i], m)
			if err != nil {
				return 0, err
			}
			var values [4]*big.Rat
			values[0], err = proof.ReplayRatArithmetic(ctx, c, bound, '-', m)
			if err != nil {
				return 0, err
			}
			values[1], err = proof.ReplayRatArithmetic(ctx, c, bound, '+', m)
			if err != nil {
				return 0, err
			}
			values[2], err = proof.ReplayRatArithmetic(ctx, c, own, '+', m)
			if err != nil {
				return 0, err
			}
			values[3], err = proof.ReplayRatArithmetic(ctx, c, own, '-', m)
			if err != nil {
				return 0, err
			}
			dest := [4]**big.Rat{&reach[0], &reach[1], &held[0], &held[1]}
			for j, value := range values {
				if *dest[j] == nil {
					*dest[j] = value
					continue
				}
				cmp, err := proof.ReplayRatCompare(ctx, value, *dest[j], m)
				if err != nil {
					return 0, err
				}
				if (j == 0 || j == 2) && cmp < 0 || (j == 1 || j == 3) && cmp > 0 {
					*dest[j] = value
				}
			}
		}
		pairs := [4][2]*big.Rat{{m0, reach[0]}, {held[0], m0}, {reach[1], m1}, {m1, held[1]}}
		for _, p := range pairs {
			gap, err := proof.ReplayRatArithmetic(ctx, p[0], p[1], '-', m)
			if err != nil {
				return 0, err
			}
			cmp, err := proof.ReplayRatCompare(ctx, gap, worst, m)
			if err != nil {
				return 0, err
			}
			if cmp > 0 {
				worst = gap
			}
		}
	}
	if worst.Sign() == 0 {
		return 0, nil
	}
	f, err := proof.ReplayFractionFloat64(ctx, worst.Num(), worst.Denom(), m)
	if err != nil {
		return 0, err
	}
	return proofbound.ProvenUpRound(f), nil
}
