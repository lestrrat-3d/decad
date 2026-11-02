package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func replayCrossFromPoints(ctx context.Context, budget *proofbound.WorkBudget, a, b, c proof.Xpt) ([3]*big.Int, error) {
	meter := replayExactWork{budget}
	var u, v, out [3]*big.Int
	for i := range 3 {
		var err error
		u[i], err = proof.ReplayProductDifference(ctx, XIntCoordOf(b, i), a.W, XIntCoordOf(a, i), b.W, meter)
		if err != nil {
			return out, err
		}
		v[i], err = proof.ReplayProductDifference(ctx, XIntCoordOf(c, i), a.W, XIntCoordOf(a, i), c.W, meter)
		if err != nil {
			return out, err
		}
	}
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		var err error
		out[i], err = proof.ReplayProductDifference(ctx, u[j], v[k], u[k], v[j], meter)
		if err != nil {
			return out, err
		}
	}
	return out, nil
}
func replaySegmentInterior3(ctx context.Context, budget *proofbound.WorkBudget, a, b, p proof.Xpt) (bool, error) {
	meter := replayExactWork{budget}
	cross, err := replayCrossFromPoints(ctx, budget, a, b, p)
	if err != nil {
		return false, err
	}
	for _, v := range cross {
		if v.Sign() != 0 {
			return false, nil
		}
	}
	var d, ap [3]*big.Int
	axis := 0
	for i := range 3 {
		d[i], err = proof.ReplayProductDifference(ctx, XIntCoordOf(b, i), a.W, XIntCoordOf(a, i), b.W, meter)
		if err != nil {
			return false, err
		}
		ap[i], err = proof.ReplayProductDifference(ctx, XIntCoordOf(p, i), a.W, XIntCoordOf(a, i), p.W, meter)
		if err != nil {
			return false, err
		}
		if i > 0 {
			if _, err := budget.StepN(uint64(max(len(d[i].Bits()), len(d[axis].Bits()))) + 1); err != nil {
				return false, err
			}
			if d[i].CmpAbs(d[axis]) > 0 {
				axis = i
			}
		}
	}
	if d[axis].Sign() == 0 {
		return false, nil
	}
	cmp, err := proof.ReplayProductDifference(ctx, ap[axis], b.W, d[axis], p.W, meter)
	if err != nil {
		return false, err
	}
	if d[axis].Sign() > 0 {
		return ap[axis].Sign() > 0 && cmp.Sign() < 0, nil
	}
	return ap[axis].Sign() < 0 && cmp.Sign() > 0, nil
}
