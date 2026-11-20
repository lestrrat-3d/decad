package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// A closed disk intersects any line segment in an interval. Require an
// exterior start and an interior endpoint; this rules out a second exposed
// strip after the first crossing, including near a nominal tangency.
func windowCrossing(ctx context.Context, n windowNative, start *big.Rat, meter *proofbound.ReplayBudget) (*big.Rat, *big.Rat, bool, error) {
	if meter == nil || start.Cmp(n.last) >= 0 {
		return nil, nil, false, nil
	}
	if e := meter.ReserveBundle(ctx, 2500, 0, 2<<20); e != nil {
		return nil, nil, false, e
	}
	r2 := times(n.r, n.r)
	rad := func(x *big.Rat) *big.Rat {
		xx, zz := minus(x, n.cx), minus(n.side(x), n.cz)
		return minus(plus(times(xx, xx), times(zz, zz)), r2)
	}
	if rad(start).Sign() <= 0 || rad(n.last).Sign() > 0 {
		return nil, nil, false, nil
	}
	bottom := new(big.Rat)
	for _, x := range []*big.Rat{start, n.last} {
		if signed(minus(n.side(x), n.cz), n.out).Sign() <= 0 {
			return nil, nil, false, nil
		}
		dx := minus(x, n.cx)
		v := minus(r2, times(dx, dx))
		if v.Sign() <= 0 {
			return nil, nil, false, nil
		}
		if bottom.Sign() == 0 || bottom.Cmp(v) > 0 {
			bottom.Set(v)
		}
	}
	minOrd := scalar(proofbound.RatSqrtDown(bottom))
	if minOrd == nil || minOrd.Sign() <= 0 {
		return nil, nil, false, nil
	}
	// Isolate the first crossing from outside. Its interval, together with
	// the native endpoint, bounds the terminal generator's X overrun.
	l, r := start, n.last
	for range 40 {
		mid := divide(plus(l, r), scalar(2))
		if rad(mid).Sign() > 0 {
			l = mid
		} else {
			r = mid
		}
	}
	return l, bottom, true, nil
}
