package facetedrecord

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replayPlaneFloatBound(ctx context.Context, q *big.Rat, up bool, b *proofbound.ReplayBudget) (float64, error) {
	f, err := proof.ReplayFractionFloat64(ctx, q.Num(), q.Denom(), b)
	if err != nil || proofbound.IsNonFinite(f) {
		return f, err
	}
	held, err := proof.ReplayFloatRat(ctx, f, b)
	if err != nil {
		return 0, err
	}
	cmp, err := proof.ReplayRatCompare(ctx, held, q, b)
	if err != nil {
		return 0, err
	}
	if up && cmp < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	if !up && cmp > 0 {
		f = math.Nextafter(f, math.Inf(-1))
	}
	return f, nil
}
func replayPlaneVec(ctx context.Context, v r3.Vec, b *proofbound.ReplayBudget) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	for i, x := range [3]float64{v.X, v.Y, v.Z} {
		q, err := proof.ReplayFloatRat(ctx, x, b)
		if err != nil {
			return out, err
		}
		out[i] = q
	}
	return out, nil
}
func replayPlaneDot(ctx context.Context, a, c [3]*big.Rat, b *proofbound.ReplayBudget) (*big.Rat, error) {
	out, err := proof.ReplayFloatRat(ctx, 0, b)
	if err != nil {
		return nil, err
	}
	for i := range 3 {
		p, err := proof.ReplayRatArithmetic(ctx, a[i], c[i], '*', b)
		if err != nil {
			return nil, err
		}
		out, err = proof.ReplayRatArithmetic(ctx, out, p, '+', b)
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}
func replayPlaneNormalGap(ctx context.Context, ideal [3]*big.Rat, moved, u, v r3.Vec, b *proofbound.ReplayBudget) (float64, error) {
	uu, err := replayPlaneVec(ctx, u, b)
	if err != nil {
		return 0, err
	}
	vv, err := replayPlaneVec(ctx, v, b)
	if err != nil {
		return 0, err
	}
	mm, err := replayPlaneVec(ctx, moved, b)
	if err != nil {
		return 0, err
	}
	var normal, gap [3]*big.Rat
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		x, err := proof.ReplayRatArithmetic(ctx, uu[j], vv[k], '*', b)
		if err != nil {
			return 0, err
		}
		y, err := proof.ReplayRatArithmetic(ctx, uu[k], vv[j], '*', b)
		if err != nil {
			return 0, err
		}
		normal[i], err = proof.ReplayRatArithmetic(ctx, x, y, '-', b)
		if err != nil {
			return 0, err
		}
		gap[i], err = proof.ReplayRatArithmetic(ctx, ideal[i], mm[i], '-', b)
		if err != nil {
			return 0, err
		}
	}
	length2, err := replayPlaneDot(ctx, normal, normal, b)
	if err != nil {
		return 0, err
	}
	down, err := replayPlaneFloatBound(ctx, length2, false, b)
	if err != nil {
		return 0, err
	}
	lengthLow := math.Nextafter(math.Sqrt(down), 0)
	if !(lengthLow > 0) {
		return math.Inf(1), nil
	}
	d, err := replayPlaneDot(ctx, gap, normal, b)
	if err != nil {
		return 0, err
	}
	if d.Sign() == 0 {
		return 0, nil
	}
	d.Abs(d)
	num, err := replayPlaneFloatBound(ctx, d, true, b)
	if err != nil {
		return 0, err
	}
	return proofbound.DivUpper(num, lengthLow), nil
}
