package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

var errReplayFloatRat = fmt.Errorf("%w: exact lift needs a finite value and budget", decaderr.ErrUnsupported)

var replayZero = big.NewInt(0)
var replayNegativeOne = big.NewInt(-1)

// ReplayFloatRat lifts a finite binary64 value to a metered exact rational.
func ReplayFloatRat(ctx context.Context, v float64, m ReplayKeyBudget) (*big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if m == nil || math.IsNaN(v) || math.IsInf(v, 0) {
		return nil, errReplayFloatRat
	}
	// FloatRat reads one binary64 word, then shifts an odd signed mantissa
	// into either numerator or denominator. The largest possible result has
	// 1,075 bits (17 machine words); three records, 2 KiB, and 128 work cover
	// its header, limb backing, and the one linear shift before construction.
	if err := m.SpendWork(ctx, 128); err != nil {
		return nil, err
	}
	if err := m.ReserveRecords(ctx, 3); err != nil {
		return nil, err
	}
	if err := m.ReserveBytes(ctx, 2048); err != nil {
		return nil, err
	}
	return FloatRat(v), nil
}
