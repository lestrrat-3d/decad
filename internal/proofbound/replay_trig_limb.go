package proofbound

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

func (a *replayTrigArena) loadRatPart(slot int, q *big.Rat, numerator bool) error {
	if slot < 0 || slot >= replayTrigSlots {
		return fmt.Errorf("%w: bounded trig slot", decaderr.ErrUnsupported)
	}
	return proof.ReplayRatPartToWords(a.ctx, q, numerator, a.slots[slot][:], a.budget)
}

func (a *replayTrigArena) loadRatPair(slot int, q *big.Rat) error {
	if err := a.loadRatPart(slot, q, true); err != nil {
		return err
	}
	return a.loadRatPart(slot+1, q, false)
}

// turnFloor reads x/(2*pi) downward on the 2^-96 grid. The signed result
// occupies slot 6. A negative input uses the lower pi bound before flooring.
// Slots 0 through 9 are scratch and must be free on entry.
func (a *replayTrigArena) turnFloor(x *big.Rat) (replayTrigSigned, error) {
	out := replayTrigSigned{slot: 6}
	if x == nil || x.Num().BitLen() > 4096 || x.Denom().BitLen() > 4096 {
		return out, fmt.Errorf("%w: bounded trig input width", decaderr.ErrUnsupported)
	}
	pi := PiUpper
	if x.Sign() < 0 {
		pi = PiLower
	}
	if err := a.loadRatPair(0, x); err != nil {
		return out, err
	}
	if err := a.loadRatPair(2, pi); err != nil {
		return out, err
	}
	if err := a.loadRatPair(8, PiUpper); err != nil {
		return out, err
	}
	// |x| <= 8*piUpper. Reject before constructing larger intermediates.
	if err := a.mul(4, 0, 9); err != nil {
		return out, err
	}
	if err := a.mul(5, 1, 8); err != nil {
		return out, err
	}
	if err := a.left(5, 3); err != nil {
		return out, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return out, err
	}
	if replayTrigCmp(&a.slots[4], &a.slots[5]) > 0 {
		return out, fmt.Errorf("%w: bounded trig angle range", decaderr.ErrUnsupported)
	}
	if err := a.mul(4, 0, 3); err != nil {
		return out, err
	}
	if err := a.left(4, TurnGridShift); err != nil {
		return out, err
	}
	if err := a.mul(5, 1, 2); err != nil {
		return out, err
	}
	if err := a.left(5, 1); err != nil {
		return out, err
	}
	if err := a.div(6, 7, 4, 5); err != nil {
		return out, err
	}
	if x.Sign() < 0 {
		if err := a.charge(replayTrigWords); err != nil {
			return out, err
		}
		if replayTrigBitLen(&a.slots[7]) != 0 {
			if err := a.setWord(8, 1); err != nil {
				return out, err
			}
			if err := a.add(6, 6, 8); err != nil {
				return out, err
			}
		}
		out.negative = replayTrigBitLen(&a.slots[6]) != 0
	}
	return out, nil
}

// gridSeries evaluates the signed 2^-96 turn in slot 6. The four returned
// endpoints denote integers on the 2^-200 grid.
func (a *replayTrigArena) gridSeries(grid replayTrigSigned) (replayTrigSpan, replayTrigSpan, error) {
	var empty replayTrigSpan
	if grid.slot != 6 {
		return empty, empty, fmt.Errorf("%w: bounded trig grid slot", decaderr.ErrUnsupported)
	}
	if err := a.copySlot(7, 6); err != nil {
		return empty, empty, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return empty, empty, err
	}
	a.slots[7][1] &= (uint64(1) << 32) - 1
	for i := 2; i < replayTrigWords; i++ {
		a.slots[7][i] = 0
	}
	if grid.negative && replayTrigBitLen(&a.slots[7]) != 0 {
		if err := a.setWord(0, 1); err != nil {
			return empty, empty, err
		}
		if err := a.left(0, 96); err != nil {
			return empty, empty, err
		}
		if err := a.sub(7, 0, 7); err != nil {
			return empty, empty, err
		}
	}
	oct := int(a.slots[7][1] >> 29)
	a.slots[7][1] &= (uint64(1) << 29) - 1
	if oct%2 == 1 {
		if err := a.setWord(0, 1); err != nil {
			return empty, empty, err
		}
		if err := a.left(0, 93); err != nil {
			return empty, empty, err
		}
		if err := a.sub(7, 0, 7); err != nil {
			return empty, empty, err
		}
	}
	// Outward floor and ceiling of pi*residual*2^105.
	for i, p := range []*big.Rat{PiLower, PiUpper} {
		if err := a.loadRatPair(0, p); err != nil {
			return empty, empty, err
		}
		if err := a.mul(2, 0, 7); err != nil {
			return empty, empty, err
		}
		if err := a.left(2, 105); err != nil {
			return empty, empty, err
		}
		if err := a.div(3, 4, 2, 1); err != nil {
			return empty, empty, err
		}
		if i == 1 && replayTrigBitLen(&a.slots[4]) != 0 {
			if err := a.setWord(5, 1); err != nil {
				return empty, empty, err
			}
			if err := a.add(3, 3, 5); err != nil {
				return empty, empty, err
			}
		}
		if err := a.copySlot(8+i, 3); err != nil {
			return empty, empty, err
		}
	}
	if err := a.fixedSeries(8, 10, 12, 14); err != nil {
		return empty, empty, err
	}
	if err := a.fixedSeries(9, 11, 13, 15); err != nil {
		return empty, empty, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return empty, empty, err
	}
	margin := 14
	if replayTrigCmp(&a.slots[15], &a.slots[14]) > 0 {
		margin = 15
	}
	sl, err := a.signedAdd(10, replayTrigSigned{10, false}, replayTrigSigned{margin, true})
	if err != nil {
		return empty, empty, err
	}
	su, err := a.signedAdd(11, replayTrigSigned{11, false}, replayTrigSigned{margin, false})
	if err != nil {
		return empty, empty, err
	}
	cl, err := a.signedAdd(13, replayTrigSigned{13, false}, replayTrigSigned{margin, true})
	if err != nil {
		return empty, empty, err
	}
	cu, err := a.signedAdd(12, replayTrigSigned{12, false}, replayTrigSigned{margin, false})
	if err != nil {
		return empty, empty, err
	}
	sin, cos := replayTrigSpan{sl, su}, replayTrigSpan{cl, cu}
	neg := func(s replayTrigSpan) replayTrigSpan {
		return replayTrigSpan{replayTrigSigned{s.hi.slot, !s.hi.negative}, replayTrigSigned{s.lo.slot, !s.lo.negative}}
	}
	switch oct {
	case 0:
		return sin, cos, nil
	case 1:
		return cos, sin, nil
	case 2:
		return cos, neg(sin), nil
	case 3:
		return sin, neg(cos), nil
	case 4:
		return neg(sin), neg(cos), nil
	case 5:
		return neg(cos), neg(sin), nil
	case 6:
		return neg(cos), sin, nil
	default:
		return neg(sin), cos, nil
	}
}

// widening rounds a nonnegative fraction upward to the same magnitude-scaled
// dyadic grid as trigWidening. Its mantissa is returned in slot 4.
func (a *replayTrigArena) widening(num, den int) (int, error) {
	if err := a.charge(2 * replayTrigWords); err != nil {
		return 0, err
	}
	nb, db := replayTrigBitLen(&a.slots[num]), replayTrigBitLen(&a.slots[den])
	if db == 0 {
		return 0, fmt.Errorf("%w: bounded trig widening denominator", decaderr.ErrUnsupported)
	}
	if nb == 0 {
		if err := a.setWord(4, 0); err != nil {
			return 0, err
		}
		return 0, nil
	}
	shift := TrigFixedBits - 1 + db - nb
	if err := a.copySlot(2, num); err != nil {
		return 0, err
	}
	if err := a.copySlot(3, den); err != nil {
		return 0, err
	}
	if shift >= 0 {
		if err := a.left(2, uint(shift)); err != nil {
			return 0, err
		}
	} else {
		if err := a.left(3, uint(-shift)); err != nil {
			return 0, err
		}
	}
	if err := a.div(4, 5, 2, 3); err != nil {
		return 0, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return 0, err
	}
	if replayTrigBitLen(&a.slots[5]) != 0 {
		if err := a.setWord(2, 1); err != nil {
			return 0, err
		}
		if err := a.add(4, 4, 2); err != nil {
			return 0, err
		}
	}
	// Unreduced numerator and denominator bit lengths can choose a grid one
	// bit finer than the reduced fraction. Two units cover that coarser grid.
	if err := a.setWord(2, 2); err != nil {
		return 0, err
	}
	if err := a.add(4, 4, 2); err != nil {
		return 0, err
	}
	return shift, nil
}

func (a *replayTrigArena) widenSpans(s, c replayTrigSpan, scale, shift int) (replayTrigSpan, replayTrigSpan, int, error) {
	if err := a.charge(replayTrigWords); err != nil {
		return s, c, scale, err
	}
	if replayTrigBitLen(&a.slots[4]) == 0 {
		return s, c, scale, nil
	}
	target := max(scale, shift, 0)
	if target > 4095 || target-shift > 4095 {
		return s, c, scale, fmt.Errorf("%w: bounded trig widening scale", decaderr.ErrUnsupported)
	}
	if err := a.copySlot(19, 4); err != nil {
		return s, c, scale, err
	}
	if err := a.left(19, uint(target-shift)); err != nil {
		return s, c, scale, err
	}
	for _, e := range []*replayTrigSigned{&s.lo, &s.hi, &c.lo, &c.hi} {
		if err := a.left(e.slot, uint(target-scale)); err != nil {
			return s, c, scale, err
		}
	}
	var err error
	s.lo, err = a.signedAdd(s.lo.slot, s.lo, replayTrigSigned{19, true})
	if err != nil {
		return s, c, scale, err
	}
	s.hi, err = a.signedAdd(s.hi.slot, s.hi, replayTrigSigned{19, false})
	if err != nil {
		return s, c, scale, err
	}
	c.lo, err = a.signedAdd(c.lo.slot, c.lo, replayTrigSigned{19, true})
	if err != nil {
		return s, c, scale, err
	}
	c.hi, err = a.signedAdd(c.hi.slot, c.hi, replayTrigSigned{19, false})
	return s, c, target, err
}

// widenRadianGap adds the uncertainty between the lower turn-grid point and
// the upper x/(2*pi) reading, scaled by the upper 2*pi bound.
func (a *replayTrigArena) widenRadianGap(x *big.Rat, grid replayTrigSigned, s, c replayTrigSpan, scale int) (replayTrigSpan, replayTrigSpan, int, error) {
	pi := PiLower
	if x.Sign() < 0 {
		pi = PiUpper
	}
	if err := a.loadRatPair(0, x); err != nil {
		return s, c, scale, err
	}
	if err := a.loadRatPair(2, pi); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(4, 0, 3); err != nil {
		return s, c, scale, err
	}
	if err := a.left(4, TurnGridShift); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(5, 1, 2); err != nil {
		return s, c, scale, err
	}
	if err := a.left(5, 1); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(7, 6, 5); err != nil {
		return s, c, scale, err
	}
	difference, err := a.signedAdd(8, replayTrigSigned{4, x.Sign() < 0}, replayTrigSigned{7, !grid.negative})
	if err != nil {
		return s, c, scale, err
	}
	if difference.negative {
		return s, c, scale, fmt.Errorf("%w: bounded trig negative turn gap", decaderr.ErrUnsupported)
	}
	if err := a.copySlot(9, 5); err != nil {
		return s, c, scale, err
	}
	if err := a.left(9, TurnGridShift); err != nil {
		return s, c, scale, err
	}
	if err := a.loadRatPair(2, PiUpper); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(0, 8, 2); err != nil {
		return s, c, scale, err
	}
	if err := a.left(0, 1); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(1, 9, 3); err != nil {
		return s, c, scale, err
	}
	shift, err := a.widening(0, 1)
	if err != nil {
		return s, c, scale, err
	}
	return a.widenSpans(s, c, scale, shift)
}

func (a *replayTrigArena) widenRadianSpan(lo, hi *big.Rat, s, c replayTrigSpan, scale int) (replayTrigSpan, replayTrigSpan, int, error) {
	if err := a.loadRatPair(0, lo); err != nil {
		return s, c, scale, err
	}
	if err := a.loadRatPair(2, hi); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(4, 2, 1); err != nil {
		return s, c, scale, err
	}
	if err := a.mul(5, 0, 3); err != nil {
		return s, c, scale, err
	}
	difference, err := a.signedAdd(8, replayTrigSigned{4, hi.Sign() < 0}, replayTrigSigned{5, lo.Sign() >= 0})
	if err != nil {
		return s, c, scale, err
	}
	if difference.negative {
		return s, c, scale, fmt.Errorf("%w: bounded trig interval is reversed", decaderr.ErrUnsupported)
	}
	if err := a.mul(9, 1, 3); err != nil {
		return s, c, scale, err
	}
	shift, err := a.widening(8, 9)
	if err != nil {
		return s, c, scale, err
	}
	return a.widenSpans(s, c, scale, shift)
}

// RadSinCosSpanReplay encloses a bounded radian interval without the global
// memo or variable-width exact arithmetic. It charges the shared replay meter.
func RadSinCosSpanReplay(ctx context.Context, span RatInterval, budget *ReplayBudget) (RatInterval, RatInterval, error) {
	return RadSinCosSpanReplayCached(ctx, span, budget, nil)
}

// RadSinCosSpanReplayCached shares only identical grid-series readings within one replay.
func RadSinCosSpanReplayCached(ctx context.Context, span RatInterval, budget *ReplayBudget, cache *ReplayTrigCache) (RatInterval, RatInterval, error) {
	if span.Lo == nil || span.Hi == nil {
		return RatInterval{}, RatInterval{}, fmt.Errorf("%w: missing bounded trig endpoint", decaderr.ErrUnsupported)
	}
	a, err := newReplayTrigArena(ctx, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if _, err = a.turnFloor(span.Hi); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	grid, err := a.turnFloor(span.Lo)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	var s, c replayTrigSpan
	scale := TrigFixedBits
	if span.Lo.Sign() == 0 {
		if err = a.setWord(10, 0); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err = a.setWord(11, 0); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err = a.setWord(12, 1); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err = a.left(12, TrigFixedBits); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err = a.copySlot(13, 12); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		s = replayTrigSpan{replayTrigSigned{10, false}, replayTrigSigned{11, false}}
		c = replayTrigSpan{replayTrigSigned{12, false}, replayTrigSigned{13, false}}
	} else {
		s, c, err = a.gridSeriesCached(grid, cache)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		s, c, scale, err = a.widenRadianGap(span.Lo, grid, s, c, scale)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	s, c, scale, err = a.widenRadianSpan(span.Lo, span.Hi, s, c, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sl, err := a.dyadicRat(s.lo, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sh, err := a.dyadicRat(s.hi, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cl, err := a.dyadicRat(c.lo, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	ch, err := a.dyadicRat(c.hi, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	return RatInterval{Lo: sl, Hi: sh}, RatInterval{Lo: cl, Hi: ch}, nil
}

// TurnSinCosReplay encloses a bounded rational turn on the same grid used by
// the radian path. Its interval includes the exact unrounded turn.
func TurnSinCosReplay(ctx context.Context, turn *big.Rat, budget *ReplayBudget) (RatInterval, RatInterval, error) {
	if turn == nil {
		return RatInterval{}, RatInterval{}, fmt.Errorf("%w: missing bounded turn", decaderr.ErrUnsupported)
	}
	a, err := newReplayTrigArena(ctx, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.loadRatPair(0, turn); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.left(0, TurnGridShift); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.div(6, 7, 0, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	grid := replayTrigSigned{6, turn.Sign() < 0}
	if grid.negative && replayTrigBitLen(&a.slots[7]) != 0 {
		if err = a.setWord(2, 1); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		if err = a.add(6, 6, 2); err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	s, c, err := a.gridSeries(grid)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.loadRatPair(0, turn); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.left(0, TurnGridShift); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.mul(7, 6, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	gap, err := a.signedAdd(8, replayTrigSigned{0, turn.Sign() < 0}, replayTrigSigned{7, !grid.negative})
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if gap.negative {
		return RatInterval{}, RatInterval{}, fmt.Errorf("%w: negative bounded turn gap", decaderr.ErrUnsupported)
	}
	if err = a.copySlot(9, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.left(9, TurnGridShift); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.loadRatPair(2, PiUpper); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.mul(0, 8, 2); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.left(0, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err = a.mul(1, 9, 3); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	shift, err := a.widening(0, 1)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	s, c, scale, err := a.widenSpans(s, c, TrigFixedBits, shift)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sl, err := a.dyadicRat(s.lo, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sh, err := a.dyadicRat(s.hi, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cl, err := a.dyadicRat(c.lo, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	ch, err := a.dyadicRat(c.hi, scale)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	return RatInterval{Lo: sl, Hi: sh}, RatInterval{Lo: cl, Hi: ch}, nil
}

// RoundIntervalGridReplay rounds an interval outward to the requested dyadic
// grid using fixed-capacity integer division.
func RoundIntervalGridReplay(ctx context.Context, iv RatInterval, shift int, b *ReplayBudget) (RatInterval, error) {
	if iv.Lo == nil || iv.Hi == nil || shift < 0 || shift > 1024 {
		return RatInterval{}, fmt.Errorf("%w: bounded grid interval", decaderr.ErrUnsupported)
	}
	a, err := newReplayTrigArena(ctx, b)
	if err != nil {
		return RatInterval{}, err
	}
	round := func(q *big.Rat, up bool) (*big.Rat, error) {
		if err := a.loadRatPair(0, q); err != nil {
			return nil, err
		}
		if err := a.left(0, uint(shift)); err != nil {
			return nil, err
		}
		if err := a.div(6, 7, 0, 1); err != nil {
			return nil, err
		}
		if err := a.charge(replayTrigWords); err != nil {
			return nil, err
		}
		if replayTrigBitLen(&a.slots[7]) != 0 && ((q.Sign() < 0 && !up) || (q.Sign() > 0 && up)) {
			if err := a.setWord(2, 1); err != nil {
				return nil, err
			}
			if err := a.add(6, 6, 2); err != nil {
				return nil, err
			}
		}
		return a.dyadicRat(replayTrigSigned{slot: 6, negative: q.Sign() < 0}, shift)
	}
	lo, err := round(iv.Lo, false)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := round(iv.Hi, true)
	if err != nil {
		return RatInterval{}, err
	}
	return RatInterval{Lo: lo, Hi: hi}, nil
}
