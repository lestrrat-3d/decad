package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// replayCoordinate reads the already materialized replay projection.
func replayCoordinate(p Xp2, axis int) *big.Rat {
	if axis == 0 {
		return p.U
	}
	return p.V
}

func replayCompare2(ctx context.Context, b *proofbound.WorkBudget, a, c Xp2, axis int) (int, error) {
	af, cf := a.Fu, c.Fu
	if axis == 1 {
		af, cf = a.Fv, c.Fv
	}
	if a.floatFromExact && c.floatFromExact {
		if err := b.Step(); err != nil {
			return 0, err
		}
		if af < cf {
			return -1, nil
		}
		if af > cf {
			return 1, nil
		}
	}
	return proof.ReplayRatCompare(ctx, replayCoordinate(a, axis), replayCoordinate(c, axis), replayExactWork{b})
}

func replayCross2(ctx context.Context, b *proofbound.WorkBudget, a, c, p Xp2) (int, error) {
	meter := replayExactWork{b}
	if err := b.Step(); err != nil {
		return 0, err
	}
	if a.FloatFinite && c.FloatFinite && p.FloatFinite {
		det := (c.Fu-a.Fu)*(p.Fv-a.Fv) - (c.Fv-a.Fv)*(p.Fu-a.Fu)
		scale := (math.Abs(c.Fu)+math.Abs(a.Fu))*(math.Abs(p.Fv)+math.Abs(a.Fv)) + (math.Abs(c.Fv)+math.Abs(a.Fv))*(math.Abs(p.Fu)+math.Abs(a.Fu))
		e := 1e-12 * scale
		if !math.IsNaN(det) && !math.IsInf(det, 0) && !math.IsInf(e, 0) && e > 0 {
			if det > e {
				return 1, nil
			}
			if det < -e {
				return -1, nil
			}
		}
	}
	if a.Hu != nil && c.Hu != nil && p.Hu != nil {
		bx, err := proof.ReplayProductDifference(ctx, c.Hu, a.Hw, a.Hu, c.Hw, meter)
		if err != nil {
			return 0, err
		}
		by, err := proof.ReplayProductDifference(ctx, c.Hv, a.Hw, a.Hv, c.Hw, meter)
		if err != nil {
			return 0, err
		}
		px, err := proof.ReplayProductDifference(ctx, p.Hu, a.Hw, a.Hu, p.Hw, meter)
		if err != nil {
			return 0, err
		}
		py, err := proof.ReplayProductDifference(ctx, p.Hv, a.Hw, a.Hv, p.Hw, meter)
		if err != nil {
			return 0, err
		}
		det, err := proof.ReplayProductDifference(ctx, bx, py, by, px, meter)
		if err != nil {
			return 0, err
		}
		return det.Sign(), nil
	}
	sub := func(x, y *big.Rat) (*big.Rat, error) { return proof.ReplayRatArithmetic(ctx, x, y, '-', meter) }
	x, err := sub(c.U, a.U)
	if err != nil {
		return 0, err
	}
	y, err := sub(c.V, a.V)
	if err != nil {
		return 0, err
	}
	z, err := sub(p.U, a.U)
	if err != nil {
		return 0, err
	}
	w, err := sub(p.V, a.V)
	if err != nil {
		return 0, err
	}
	left, err := proof.ReplayRatArithmetic(ctx, x, w, '*', meter)
	if err != nil {
		return 0, err
	}
	right, err := proof.ReplayRatArithmetic(ctx, y, z, '*', meter)
	if err != nil {
		return 0, err
	}
	return proof.ReplayRatCompare(ctx, left, right, meter)
}

func replayOnSegment2(ctx context.Context, b *proofbound.WorkBudget, a, c, p Xp2) (bool, error) {
	side, err := replayCross2(ctx, b, a, c, p)
	if err != nil || side != 0 {
		return false, err
	}
	axis := 0
	order, err := replayCompare2(ctx, b, a, c, axis)
	if err != nil {
		return false, err
	}
	if order == 0 {
		axis = 1
		order, err = replayCompare2(ctx, b, a, c, axis)
		if err != nil {
			return false, err
		}
		if order == 0 {
			x, e := replayCompare2(ctx, b, p, a, 0)
			if e != nil || x != 0 {
				return false, e
			}
			y, e := replayCompare2(ctx, b, p, a, 1)
			return y == 0, e
		}
	}
	if order > 0 {
		a, c = c, a
	}
	lo, err := replayCompare2(ctx, b, p, a, axis)
	if err != nil {
		return false, err
	}
	hi, err := replayCompare2(ctx, b, p, c, axis)
	return lo >= 0 && hi <= 0, err
}

func polygonSide(ctx context.Context, b *proofbound.WorkBudget, a, c, p Xp2) (int, error) {
	if b.Replay {
		return replayCross2(ctx, b, a, c, p)
	}
	return Cross2xSign(a, c, p), nil
}
