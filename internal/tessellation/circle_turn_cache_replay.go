package tessellation

import (
	"context"
	"unsafe"

	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type circleTurnCache struct {
	n, next  int
	meter    *proofbound.ReplayBudget
	zero     *big.Rat
	sin, cos []proofbound.RatInterval
}

func newCircleTurnCache(ctx context.Context, n int, b *proofbound.ReplayBudget) (*circleTurnCache, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || n < 3 || n > 1<<14 {
		return nil, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	half := n/2 + 1
	if err := b.ReserveRecords(ctx, uint64(2*half+1)); err != nil {
		return nil, err
	}
	for range 2 {
		if err := b.ReserveArray(ctx, uint64(half), uint64(unsafe.Sizeof(proofbound.RatInterval{}))); err != nil {
			return nil, err
		}
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(circleTurnCache{}))); err != nil {
		return nil, err
	}
	zero, err := proof.ReplayFloatRat(ctx, 0, b)
	if err != nil {
		return nil, err
	}
	return &circleTurnCache{n: n, next: 1, meter: b, zero: zero, sin: make([]proofbound.RatInterval, half), cos: make([]proofbound.RatInterval, half)}, nil
}

func (c *circleTurnCache) station(ctx context.Context, k int, b *proofbound.ReplayBudget) (proofbound.RatInterval, proofbound.RatInterval, error) {
	if err := ctx.Err(); err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	if c == nil || b == nil || c.meter != b || k != c.next || k <= 0 || k >= c.n {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	var s, co proofbound.RatInterval
	neg := func(x proofbound.RatInterval) (proofbound.RatInterval, error) {
		if x.Lo == nil || x.Hi == nil {
			return proofbound.RatInterval{}, decaderr.ErrUnsupported
		}
		lo, err := proof.ReplayRatArithmetic(ctx, c.zero, x.Hi, '-', b)
		if err != nil {
			return proofbound.RatInterval{}, err
		}
		hi, err := proof.ReplayRatArithmetic(ctx, c.zero, x.Lo, '-', b)
		if err != nil {
			return proofbound.RatInterval{}, err
		}
		return proofbound.RatInterval{Lo: lo, Hi: hi}, nil
	}
	n := c.n
	var err error
	switch {
	case 8*k%n == 0:
		s, co, err = proofbound.CircleTurnFractionReplay(ctx, k, n, b)
	case n%4 == 0 && k <= n/4 && k > n/8 && k < n/4:
		i := n/4 - k
		s, co = c.cos[i], c.sin[i]
	case n%2 == 0 && k <= n/2 && k > n/4 && k < n/2:
		i := n/2 - k
		s = c.sin[i]
		co, err = neg(c.cos[i])
	case k <= n/2:
		s, co, err = proofbound.CircleTurnFractionReplay(ctx, k, n, b)
	default:
		i := n - k
		s, err = neg(c.sin[i])
		co = c.cos[i]
	}
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	if s.Lo == nil || s.Hi == nil || co.Lo == nil || co.Hi == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, decaderr.ErrUnsupported
	}
	if k <= n/2 {
		c.sin[k], c.cos[k] = s, co
	}
	c.next++
	return s, co, nil
}
