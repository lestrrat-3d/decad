package proof

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

func replayScalePow2(ctx context.Context, v *big.Int, shift uint, m ReplayKeyBudget) (*big.Int, error) {
	words := uint64(len(v.Bits())) + uint64(shift)/uint64(bits.UintSize) + 1
	if words > 65536 {
		return nil, errReplayReplayAffineWidth
	}
	if err := m.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := m.ReserveBytes(ctx, 2*words*uint64(bits.UintSize/8)+128); err != nil {
		return nil, err
	}
	if err := m.SpendWork(ctx, words); err != nil {
		return nil, err
	}
	storage := make([]big.Word, int(words))
	copy(storage, v.Bits())
	if err := keyShiftLeft(ctx, storage, shift, m); err != nil {
		return nil, err
	}
	out := new(big.Int).SetBits(storage)
	if v.Sign() < 0 {
		out.Neg(out)
	}
	return out, nil
}

// ReplayExactAffineImage matches the ordinary dyadic affine image with
// fixed-limb rational arithmetic and checked shifts.
func ReplayExactAffineImage(ctx context.Context, ex, ey, ez, t, p r3.Vec, m ReplayKeyBudget) (Xpt, error) {
	basis := [3]r3.Vec{ex, ey, ez}
	pv := [3]float64{p.X, p.Y, p.Z}
	tv := [3]float64{t.X, t.Y, t.Z}
	var coord [3]*big.Rat
	for axis := range 3 {
		var sum *big.Rat
		for j := range 3 {
			coef := [3]float64{basis[j].X, basis[j].Y, basis[j].Z}[axis]
			a, err := ReplayFloatRat(ctx, coef, m)
			if err != nil {
				return Xpt{}, err
			}
			b, err := ReplayFloatRat(ctx, pv[j], m)
			if err != nil {
				return Xpt{}, err
			}
			term, err := ReplayRatArithmetic(ctx, a, b, '*', m)
			if err != nil {
				return Xpt{}, err
			}
			if sum == nil {
				sum = term
			} else {
				sum, err = ReplayRatArithmetic(ctx, sum, term, '+', m)
				if err != nil {
					return Xpt{}, err
				}
			}
		}
		tr, err := ReplayFloatRat(ctx, tv[axis], m)
		if err != nil {
			return Xpt{}, err
		}
		coord[axis], err = ReplayRatArithmetic(ctx, sum, tr, '+', m)
		if err != nil {
			return Xpt{}, err
		}
	}
	maxExp := 0
	var exp [3]int
	for i, q := range coord {
		v, ok := powerOfTwoExp(q.Denom())
		if !ok {
			return Xpt{}, errReplayReplayAffineDenominator
		}
		exp[i] = v
		maxExp = max(maxExp, v)
	}
	var vals [3]*big.Int
	for i, q := range coord {
		v, err := replayScalePow2(ctx, q.Num(), uint(maxExp-exp[i]), m)
		if err != nil {
			return Xpt{}, err
		}
		vals[i] = v
	}
	one, err := replayScalePow2(ctx, replayOne, uint(maxExp), m)
	if err != nil {
		return Xpt{}, err
	}
	out := Xpt{X: vals[0], Y: vals[1], Z: vals[2], W: one}
	if err := replayStripTwos(ctx, out, m); err != nil {
		return Xpt{}, err
	}
	return out, nil
}
func ReplayXptDenotes(ctx context.Context, p Xpt, v r3.Vec, m ReplayKeyBudget) (bool, error) {
	x, err := XptOfBounded(v, func(n uint64) error {
		if err := m.ReserveBytes(ctx, n*64); err != nil {
			return err
		}
		return m.ReserveRecords(ctx, n)
	})
	if err != nil {
		return false, err
	}
	for i := range 3 {
		d, err := ReplayProductDifference(ctx, XIntCoordReplay(p, i), x.W, XIntCoordReplay(x, i), p.W, m)
		if err != nil {
			return false, err
		}
		if d.Sign() != 0 {
			return false, nil
		}
	}
	return true, nil
}
func XIntCoordReplay(p Xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}
