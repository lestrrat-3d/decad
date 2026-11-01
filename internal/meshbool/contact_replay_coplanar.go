package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

var replayExactOne = big.NewInt(1)

func replayProjectionAxes(ctx context.Context, n proof.Xpt, b proof.ReplayKeyBudget) (int, int, error) {
	if err := b.SpendWork(ctx, uint64(len(n.X.Bits())+len(n.Y.Bits())+len(n.Z.Bits()))+3); err != nil {
		return 0, 0, err
	}
	if n.Z.CmpAbs(n.X) >= 0 && n.Z.CmpAbs(n.Y) >= 0 {
		return 0, 1, nil
	}
	if n.Y.CmpAbs(n.X) >= 0 {
		return 2, 0, nil
	}
	return 1, 2, nil
}

// replayCross2Numerator is the homogeneous 2D cross-product numerator.
// Its positive denominator is a.W²*b.W*p.W.
func replayCross2Numerator(ctx context.Context, a, b, p proof.Xpt, u, v int, meter proof.ReplayKeyBudget) (*big.Int, error) {
	bx, err := proof.ReplayProductDifference(ctx, Coord(b, u), a.W, Coord(a, u), b.W, meter)
	if err != nil {
		return nil, err
	}
	by, err := proof.ReplayProductDifference(ctx, Coord(b, v), a.W, Coord(a, v), b.W, meter)
	if err != nil {
		return nil, err
	}
	px, err := proof.ReplayProductDifference(ctx, Coord(p, u), a.W, Coord(a, u), p.W, meter)
	if err != nil {
		return nil, err
	}
	py, err := proof.ReplayProductDifference(ctx, Coord(p, v), a.W, Coord(a, v), p.W, meter)
	if err != nil {
		return nil, err
	}
	return proof.ReplayProductDifference(ctx, bx, py, by, px, meter)
}

type replayClipFrac struct{ num, den *big.Int }

func replayClipCompare(ctx context.Context, a, b replayClipFrac, meter proof.ReplayKeyBudget) (int, error) {
	d, err := proof.ReplayProductDifference(ctx, a.num, b.den, b.num, a.den, meter)
	if err != nil {
		return 0, err
	}
	return d.Sign(), nil
}

func replaySegTriOverlap(ctx context.Context, a, b proof.Xpt, tri [3]proof.Xpt, u, v int, meter proof.ReplayKeyBudget) (bool, error) {
	return replaySegTriOverlapHeld(ctx, a, b, tri, u, v, meter, false)
}

func replaySegTriOverlapHeld(ctx context.Context, a, b proof.Xpt, tri [3]proof.Xpt, u, v int, meter proof.ReplayKeyBudget, chargeNeg bool) (bool, error) {
	orientation, err := replayCross2Numerator(ctx, tri[0], tri[1], tri[2], u, v, meter)
	if err != nil {
		return false, err
	}
	ccw := orientation.Sign()
	if ccw == 0 {
		return false, nil
	}
	lo := replayClipFrac{replayExactZero, replayExactOne}
	hi := replayClipFrac{replayExactOne, replayExactOne}
	for i := range 3 {
		e0, e1 := tri[i], tri[(i+1)%3]
		fa, err := replayCross2Numerator(ctx, e0, e1, a, u, v, meter)
		if err != nil {
			return false, err
		}
		fb, err := replayCross2Numerator(ctx, e0, e1, b, u, v, meter)
		if err != nil {
			return false, err
		}
		sa, sb := fa.Sign()*ccw, fb.Sign()*ccw
		if sa >= 0 && sb >= 0 {
			continue
		}
		if sa < 0 && sb < 0 {
			return false, nil
		}
		x, err := proof.ReplayProductDifference(ctx, fa, b.W, replayExactZero, replayExactZero, meter)
		if err != nil {
			return false, err
		}
		den, err := proof.ReplayProductDifference(ctx, fb, a.W, replayExactOne, x, meter)
		if err != nil {
			return false, err
		}
		if chargeNeg {
			if err := meter.SpendWork(ctx, uint64(len(x.Bits()))+1); err != nil {
				return false, err
			}
		}
		x.Neg(x)
		if den.Sign() < 0 {
			if chargeNeg {
				if err := meter.SpendWork(ctx, uint64(len(x.Bits()))+1); err != nil {
					return false, err
				}
			}
			x.Neg(x)
			if chargeNeg {
				if err := meter.SpendWork(ctx, uint64(len(den.Bits()))+1); err != nil {
					return false, err
				}
			}
			den.Neg(den)
		}
		t := replayClipFrac{x, den}
		if sa < 0 {
			cmp, err := replayClipCompare(ctx, t, lo, meter)
			if err != nil {
				return false, err
			}
			if cmp > 0 {
				lo = t
			}
		} else {
			cmp, err := replayClipCompare(ctx, t, hi, meter)
			if err != nil {
				return false, err
			}
			if cmp < 0 {
				hi = t
			}
		}
	}
	cmp, err := replayClipCompare(ctx, lo, hi, meter)
	return cmp < 0, err
}

func replayCoplanarOverlap(ctx context.Context, a, b [3]proof.Xpt, normal proof.Xpt, meter proof.ReplayKeyBudget) (bool, error) {
	u, v, err := replayProjectionAxes(ctx, normal, meter)
	if err != nil {
		return false, err
	}
	for i := range 3 {
		if err := meter.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		found, err := replaySegTriOverlap(ctx, a[i], a[(i+1)%3], b, u, v, meter)
		if err != nil || found {
			return found, err
		}
		found, err = replaySegTriOverlap(ctx, b[i], b[(i+1)%3], a, u, v, meter)
		if err != nil || found {
			return found, err
		}
	}
	return false, nil
}

func replayPointOnTri(ctx context.Context, p proof.Xpt, tri [3]proof.Xpt, u, v int, meter proof.ReplayKeyBudget) (bool, error) {
	var signs [3]int
	for i := range 3 {
		cross, err := replayCross2Numerator(ctx, tri[i], tri[(i+1)%3], p, u, v, meter)
		if err != nil {
			return false, err
		}
		signs[i] = cross.Sign()
	}
	return (signs[0] >= 0 && signs[1] >= 0 && signs[2] >= 0) || (signs[0] <= 0 && signs[1] <= 0 && signs[2] <= 0), nil
}

func replayCoplanarContactHeld(ctx context.Context, ta, tb [3]r3.Vec, xa, xb [3]proof.Xpt, na, nb proof.Xpt, meter proof.ReplayKeyBudget, heldSharedOne bool) (TriContact, error) {
	out := TriContact{EdgeA: -1, EdgeB: -1}
	u, v, err := replayProjectionAxes(ctx, na, meter)
	if err != nil {
		return out, err
	}
	charge := uint64(1024)
	if heldSharedOne {
		charge = 512
	}
	if err := meter.SpendWork(ctx, charge); err != nil {
		return out, err
	}
	if !heldSharedOne {
		var a, b [3][2]float64
		for i := range 3 {
			a[i] = [2]float64{AxisCoord(ta[i], u), AxisCoord(ta[i], v)}
			b[i] = [2]float64{AxisCoord(tb[i], u), AxisCoord(tb[i], v)}
		}
		for i := range 3 {
			if FloatEdgeSeparates(a[i], a[(i+1)%3], a[(i+2)%3], b) || FloatEdgeSeparates(b[i], b[(i+1)%3], b[(i+2)%3], a) {
				return out, nil
			}
		}
	}
	for i := range 3 {
		ok, err := replaySegTriOverlapHeld(ctx, xa[i], xa[(i+1)%3], xb, u, v, meter, heldSharedOne)
		if err != nil {
			return out, err
		}
		if ok {
			out.Kind = ContactRegion
			return out, nil
		}
		ok, err = replaySegTriOverlapHeld(ctx, xb[i], xb[(i+1)%3], xa, u, v, meter, heldSharedOne)
		if err != nil {
			return out, err
		}
		if ok {
			out.Kind = ContactRegion
			return out, nil
		}
	}
	if nb.X.Sign() == 0 && nb.Y.Sign() == 0 && nb.Z.Sign() == 0 {
		out.Kind = ContactPoint
		out.P0, out.P1 = xa[0], xa[0]
		return out, nil
	}
	bu, bv, err := replayProjectionAxes(ctx, nb, meter)
	if err != nil {
		return out, err
	}
	for _, p := range xa {
		ok, err := replayPointOnTri(ctx, p, xb, bu, bv, meter)
		if err != nil {
			return out, err
		}
		if ok {
			out.Kind = ContactPoint
			out.P0, out.P1 = p, p
			return out, nil
		}
	}
	if na.X.Sign() == 0 && na.Y.Sign() == 0 && na.Z.Sign() == 0 {
		out.Kind = ContactPoint
		out.P0, out.P1 = xb[0], xb[0]
		return out, nil
	}
	for _, p := range xb {
		ok, err := replayPointOnTri(ctx, p, xa, u, v, meter)
		if err != nil {
			return out, err
		}
		if ok {
			out.Kind = ContactPoint
			out.P0, out.P1 = p, p
			return out, nil
		}
	}
	return out, nil
}
