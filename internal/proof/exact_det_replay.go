package proof

import (
	"context"
	"math/big"
)

// ReplayDetDyadic constructs the same exact determinant as XdotDyadic(a,Xcross(b,c)).
// Every multiplication, addition and normalization uses the shared limb meter.
func ReplayDetDyadic(ctx context.Context, a, b, c Xpt, meter ReplayKeyBudget) (Dyadic, *big.Rat, error) {
	fail := func(err error) (Dyadic, *big.Rat, error) { return Dyadic{}, nil, err }
	for _, p := range [3]Xpt{a, b, c} {
		if p.W == nil || p.W.Sign() <= 0 {
			return fail(errReplayInvalidDeterminantDenominator)
		}
		if _, ok := powerOfTwoExp(p.W); !ok {
			return fail(errReplayNonDyadicDeterminant)
		}
		for _, v := range [4]*big.Int{p.X, p.Y, p.Z, p.W} {
			if v == nil || v.BitLen() > 1024 {
				return fail(errReplayReplayDeterminantInputWidth)
			}
		}
	}
	diff := func(x, y, z, w *big.Int) (*big.Int, error) { return ReplayProductDifference(ctx, x, y, z, w, meter) }
	x, err := diff(b.Y, c.Z, b.Z, c.Y)
	if err != nil {
		return fail(err)
	}
	y, err := diff(b.Z, c.X, b.X, c.Z)
	if err != nil {
		return fail(err)
	}
	z, err := diff(b.X, c.Y, b.Y, c.X)
	if err != nil {
		return fail(err)
	}
	sum, err := diff(a.X, x, replayNegativeOne, replayZero)
	if err != nil {
		return fail(err)
	}
	sum, err = diff(a.Y, y, replayNegativeOne, sum)
	if err != nil {
		return fail(err)
	}
	sum, err = diff(a.Z, z, replayNegativeOne, sum)
	if err != nil {
		return fail(err)
	}
	den, err := diff(a.W, b.W, replayZero, replayZero)
	if err != nil {
		return fail(err)
	}
	den, err = diff(den, c.W, replayZero, replayZero)
	if err != nil {
		return fail(err)
	}
	if err := meter.ReserveBytes(ctx, 256); err != nil {
		return fail(err)
	}
	var zeroY, zeroZ big.Int
	out := Xpt{X: sum, Y: &zeroY, Z: &zeroZ, W: den}
	if err = replayStripTwos(ctx, out, meter); err != nil {
		return fail(err)
	}
	rat, err := ReplayReducedFraction(ctx, out.X, out.W, meter)
	if err != nil {
		return fail(err)
	}
	exp, ok := powerOfTwoExp(out.W)
	if !ok {
		return fail(errReplayNonDyadicDeterminantResult)
	}
	// The output numerator is already odd, so fromBig retains it without allocation.
	return fromBig(out.X, -exp), rat, nil
}
