package proofbound

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// TurnFractionSinCosReplay reproduces the ordinary whole-circle station
// enclosure for the interior fraction k/n using bounded integer arithmetic.
func TurnFractionSinCosReplay(ctx context.Context, k, n int, b *ReplayBudget) (RatInterval, RatInterval, error) {
	if n < 3 || n > 1<<14 || k <= 0 || k >= n || b == nil || !b.IsBrakeReplay() {
		return RatInterval{}, RatInterval{}, fmt.Errorf("%w: invalid replay circle fraction", decaderr.ErrUnsupported)
	}
	a, err := newReplayTrigArena(ctx, b)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	// The ordinary circle endpoint uses exact point intervals at quadrants.
	if 4*k%n == 0 {
		q := 4 * k / n
		if err := a.setWord(0, 0); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.setWord(1, 1); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		zero, err := a.dyadicRat(replayTrigSigned{slot: 0}, 0)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		one, err := a.dyadicRat(replayTrigSigned{slot: 1, negative: q >= 2}, 0)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		// PointInterval copies its rational twice.
		if err := b.ReserveArray(ctx, 8, 256); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := b.ReserveRecords(ctx, 8); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if q%2 == 0 {
			return PointInterval(zero), PointInterval(one), nil
		}
		return PointInterval(one), PointInterval(zero), nil
	}
	oct := 8 * k / n
	rem := 8*k - oct*n
	if oct%2 != 0 {
		rem = n - rem
	}
	// The ordinary first-octant endpoints are floor/ceil of
	// pi * 2 * rem/(8*n) * 2^TrigFixedBits.
	for i, p := range []*big.Rat{PiLower, PiUpper} {
		if err := a.loadRatPair(0, p); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.setWord(2, uint64(rem)); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.mul(3, 0, 2); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.left(3, TrigFixedBits+1); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.setWord(2, uint64(8*n)); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.mul(4, 1, 2); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err := a.div(5, 6, 3, 4); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if i == 1 && replayTrigBitLen(&a.slots[6]) != 0 {
			if err := a.setWord(2, 1); err != nil {
				return RatInterval{}, RatInterval{}, err
			}
			if err := a.add(5, 5, 2); err != nil {
				return RatInterval{}, RatInterval{}, err
			}
		}
		if err := a.copySlot(8+i, 5); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	if err := a.fixedSeries(8, 10, 12, 14); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err := a.fixedSeries(9, 11, 13, 15); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	margin := 14
	if replayTrigCmp(&a.slots[15], &a.slots[14]) > 0 {
		margin = 15
	}
	sl, err := a.signedAdd(10, replayTrigSigned{10, false}, replayTrigSigned{margin, true})
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sh, err := a.signedAdd(11, replayTrigSigned{11, false}, replayTrigSigned{margin, false})
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cl, err := a.signedAdd(13, replayTrigSigned{13, false}, replayTrigSigned{margin, true})
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	ch, err := a.signedAdd(12, replayTrigSigned{12, false}, replayTrigSigned{margin, false})
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sin, cos := replayTrigSpan{sl, sh}, replayTrigSpan{cl, ch}
	neg := func(s replayTrigSpan) replayTrigSpan {
		return replayTrigSpan{replayTrigSigned{s.hi.slot, !s.hi.negative}, replayTrigSigned{s.lo.slot, !s.lo.negative}}
	}
	switch oct {
	case 1:
		sin, cos = cos, sin
	case 2:
		sin, cos = cos, neg(sin)
	case 3:
		cos = neg(cos)
	case 4:
		sin, cos = neg(sin), neg(cos)
	case 5:
		sin, cos = neg(cos), neg(sin)
	case 6:
		sin, cos = neg(cos), sin
	case 7:
		sin = neg(sin)
	}
	values := [4]replayTrigSigned{sin.lo, sin.hi, cos.lo, cos.hi}
	var out [4]*big.Rat
	for i, v := range values {
		out[i], err = a.dyadicRat(v, TrigFixedBits)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	return RatInterval{Lo: out[0], Hi: out[1]}, RatInterval{Lo: out[2], Hi: out[3]}, nil
}
