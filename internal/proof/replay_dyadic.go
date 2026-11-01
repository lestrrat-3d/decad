package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayDyadicArithmetic applies exact dyadic arithmetic with precharged
// operand-width-dependent scratch and limb work.
func ReplayDyadicArithmetic(ctx context.Context, a, b Dyadic, op byte, m ReplayKeyBudget) (Dyadic, error) {
	if m == nil {
		return Dyadic{}, fmt.Errorf("%w: bounded dyadic meter missing", decaderr.ErrUnsupported)
	}
	reserve := func(n uint64) error {
		if n > ^uint64(0)/64 {
			return fmt.Errorf("%w: bounded dyadic storage overflow", decaderr.ErrUnsupported)
		}
		if err := m.ReserveBytes(ctx, n*64); err != nil {
			return err
		}
		return m.ReserveRecords(ctx, n)
	}
	spend := func(n uint64) error { return m.SpendWork(ctx, n) }
	switch op {
	case '+', '-':
		if err := normalChargeDifference(a, b, reserve, spend); err != nil {
			return Dyadic{}, err
		}
		if op == '+' {
			return DyAdd(a, b), nil
		}
		return DySubScalar(a, b), nil
	case '*':
		return normalMulReplay(a, b, reserve, spend)
	default:
		return Dyadic{}, fmt.Errorf("%w: unknown bounded dyadic operation", decaderr.ErrUnsupported)
	}
}

// ReplaySqrtDyadic matches DySqrtDown or DySqrtUp while charging every exact
// square and comparison. The fixed-size seed reads only the leading words;
// ambiguous seeds use a bounded exact search.
func ReplaySqrtDyadic(ctx context.Context, d Dyadic, down bool, m ReplayKeyBudget) (float64, error) {
	if m == nil {
		return 0, fmt.Errorf("%w: bounded dyadic meter missing", decaderr.ErrUnsupported)
	}
	if err := m.SpendWork(ctx, 16); err != nil {
		return 0, err
	}
	if d.Sign() <= 0 {
		return 0, nil
	}
	if d.exp < -16384 || d.exp > 16384 {
		return 0, fmt.Errorf("%w: bounded square-root exponent", decaderr.ErrUnsupported)
	}
	var f float64
	if d.big == nil {
		f = dySqrtSeedInline(d)
	} else {
		var ok bool
		f, ok = RatioSqrtSeed(d.big, replayOne, -d.exp)
		if !ok {
			return replaySqrtDyadicSearch(ctx, d, down, m)
		}
	}
	if math.IsInf(f, 0) || math.IsNaN(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		v, ok := DyOf(f)
		if !ok {
			return math.Inf(1), nil
		}
		sq, err := ReplayDyadicArithmetic(ctx, v, v, '*', m)
		if err != nil {
			return 0, err
		}
		diff, err := ReplayDyadicArithmetic(ctx, sq, d, '-', m)
		if err != nil {
			return 0, err
		}
		if down && diff.Sign() <= 0 || !down && diff.Sign() >= 0 {
			return f, nil
		}
		if down {
			f = math.Nextafter(f, 0)
		} else {
			f = math.Nextafter(f, math.Inf(1))
		}
	}
	if down {
		return 0, nil
	}
	return math.Inf(1), nil
}

// ReplayDyadicRat constructs the exact rational after charging its maximum
// shifted numerator and denominator storage.
func ReplayDyadicRat(ctx context.Context, d Dyadic, m ReplayKeyBudget) (*big.Rat, error) {
	if m == nil {
		return nil, fmt.Errorf("%w: bounded dyadic meter missing", decaderr.ErrUnsupported)
	}
	width := max(1, d.bitLen()+max(d.exp, 0), 1+max(-d.exp, 0))
	words := uint64((width + bits.UintSize - 1) / bits.UintSize)
	if words > (^uint64(0)-16)/8 {
		return nil, fmt.Errorf("%w: bounded dyadic width overflow", decaderr.ErrUnsupported)
	}
	if err := m.ReserveBytes(ctx, 8*(words+4)*uint64(bits.UintSize/8)+512); err != nil {
		return nil, err
	}
	if err := m.ReserveRecords(ctx, 4); err != nil {
		return nil, err
	}
	if err := m.SpendWork(ctx, 4*words+4); err != nil {
		return nil, err
	}
	return d.Rat(), nil
}

// ReplayDyadicFloatError computes the same upward-rounded absolute error as
// ReplayRationalFloatError without lifting the held value to a rational.
func ReplayDyadicFloatError(ctx context.Context, exact Dyadic, held float64, m ReplayKeyBudget) (float64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if m == nil || math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, decaderr.ErrUnsupported
	}
	if err := m.SpendWork(ctx, 64); err != nil {
		return 0, err
	}
	d, err := ReplayDyadicArithmetic(ctx, exact, DyOfFinite(held), '-', m)
	if err != nil {
		return 0, err
	}
	if d.Sign() < 0 {
		d, err = ReplayDyadicArithmetic(ctx, DyZero(), d, '-', m)
		if err != nil {
			return 0, err
		}
	}
	if d.Sign() == 0 {
		return 0, nil
	}
	r, err := ReplayDyadicRat(ctx, d, m)
	if err != nil {
		return 0, err
	}
	f, err := ReplayFractionFloat64(ctx, r.Num(), r.Denom(), m)
	if err != nil {
		return 0, err
	}
	if math.IsInf(f, 1) {
		return f, nil
	}
	cmp, err := ReplayDyadicArithmetic(ctx, DyOfFinite(f), d, '-', m)
	if err != nil {
		return 0, err
	}
	if cmp.Sign() != 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f, nil
}
