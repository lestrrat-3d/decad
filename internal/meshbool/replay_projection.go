package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func replayProjection(ctx context.Context, p proof.Xpt, u, v int, budget proof.ReplayKeyBudget) (Xp2, error) {
	hu, hv := XIntCoordOf(p, u), XIntCoordOf(p, v)
	ur, err := proof.ReplayReducedFraction(ctx, hu, p.W, budget)
	if err != nil {
		return Xp2{}, err
	}
	vr, err := proof.ReplayReducedFraction(ctx, hv, p.W, budget)
	if err != nil {
		return Xp2{}, err
	}
	fu, err := proof.ReplayFractionFloat64(ctx, ur.Num(), ur.Denom(), budget)
	if err != nil {
		return Xp2{}, err
	}
	fv, err := proof.ReplayFractionFloat64(ctx, vr.Num(), vr.Denom(), budget)
	if err != nil {
		return Xp2{}, err
	}
	return Xp2{U: ur, V: vr, Fu: fu, Fv: fv, floatFromExact: true, Hu: hu, Hv: hv, Hw: p.W, FloatFinite: xp2FloatSafe(ur, fu) && xp2FloatSafe(vr, fv)}, nil
}

func replayXP2FromRats(ctx context.Context, u, v *big.Rat, budget proof.ReplayKeyBudget) (Xp2, error) {
	fu, err := proof.ReplayFractionFloat64(ctx, u.Num(), u.Denom(), budget)
	if err != nil {
		return Xp2{}, err
	}
	fv, err := proof.ReplayFractionFloat64(ctx, v.Num(), v.Denom(), budget)
	if err != nil {
		return Xp2{}, err
	}
	return Xp2{U: u, V: v, Fu: fu, Fv: fv, floatFromExact: true, FloatFinite: xp2FloatSafe(u, fu) && xp2FloatSafe(v, fv)}, nil
}
