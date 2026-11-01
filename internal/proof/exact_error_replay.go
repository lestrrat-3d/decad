package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/r3"
)

func replayFloatRat(ctx context.Context, v float64, m ReplayKeyBudget) (*big.Rat, error) {
	if m == nil {
		return nil, fmt.Errorf("%w: missing rational budget", decaderr.ErrUnsupported)
	}
	p, err := XptOfBounded(r3.Vec{X: v}, func(blocks uint64) error {
		if blocks > ^uint64(0)/64 {
			return fmt.Errorf("%w: rational lift overflow", decaderr.ErrUnsupported)
		}
		if err := m.SpendWork(ctx, blocks*64); err != nil {
			return err
		}
		if err := m.ReserveRecords(ctx, blocks); err != nil {
			return err
		}
		return m.ReserveBytes(ctx, blocks*64)
	})
	if err != nil {
		return nil, err
	}
	return ReplayReducedFraction(ctx, p.X, p.W, m)
}

// ReplayRationalFloatError computes |exact-held| rounded toward positive infinity.
func ReplayRationalFloatError(ctx context.Context, exact *big.Rat, held float64, m ReplayKeyBudget) (float64, error) {
	h, err := replayFloatRat(ctx, held, m)
	if err != nil {
		return 0, err
	}
	return replayRationalRatError(ctx, exact, h, m)
}

// ReplayRationalRatError bounds an exact rational difference without lifting
// the same held rational repeatedly. It does not mutate either input.
func ReplayRationalRatError(ctx context.Context, exact, held *big.Rat, m ReplayKeyBudget) (float64, error) {
	if exact == nil || held == nil || m == nil {
		return 0, fmt.Errorf("%w: missing rational error input", decaderr.ErrUnsupported)
	}
	if err := m.SpendWork(ctx, 4); err != nil {
		return 0, err
	}
	return replayRationalRatError(ctx, exact, held, m)
}

func replayRationalRatError(ctx context.Context, exact, held *big.Rat, m ReplayKeyBudget) (float64, error) {
	d, err := ReplayRatArithmetic(ctx, exact, held, '-', m)
	if err != nil {
		return 0, err
	}
	if d.Sign() < 0 {
		d.Neg(d)
	}
	if d.Sign() == 0 {
		return 0, nil
	}
	f, err := ReplayFractionFloat64(ctx, d.Num(), d.Denom(), m)
	if err != nil {
		return 0, err
	}
	if math.IsInf(f, 1) {
		return f, nil
	}
	r, err := replayFloatRat(ctx, f, m)
	if err != nil {
		return 0, err
	}
	cmp, err := ReplayRatCompare(ctx, r, d, m)
	if err != nil {
		return 0, err
	}
	if cmp != 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f, nil
}

// ReplayFloatRat lifts a finite float to its exact rational value.
func ReplayFloatRat(ctx context.Context, v float64, m ReplayKeyBudget) (*big.Rat, error) {
	return replayFloatRat(ctx, v, m)
}
