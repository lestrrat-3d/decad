package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errExactErrorReplay3 = fmt.Errorf("%w: missing rational error input", decaderr.ErrUnsupported)
)

// ReplayRationalRatError bounds an exact rational difference without lifting
// the same held rational repeatedly. It does not mutate either input.
func ReplayRationalRatError(ctx context.Context, exact, held *big.Rat, m ReplayKeyBudget) (float64, error) {
	if exact == nil || held == nil || m == nil {
		return 0, errExactErrorReplay3
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
	r, err := ReplayFloatRat(ctx, f, m)
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
