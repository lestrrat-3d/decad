package proof

import (
	"context"
	"math/big"
)

var replayCentroidThree = big.NewInt(3)

// ReplayCentroid constructs the same unreduced centroid spelling as XCentroid.
func ReplayCentroid(ctx context.Context, a, b, c Xpt, budget ReplayKeyBudget) (Xpt, error) {
	mul := func(x, y *big.Int) (*big.Int, error) {
		return ReplayProductDifference(ctx, x, y, replayZero, replayZero, budget)
	}
	add := func(x, y *big.Int) (*big.Int, error) {
		return ReplayProductDifference(ctx, x, replayOne, replayNegativeOne, y, budget)
	}
	bc, err := mul(b.W, c.W)
	if err != nil {
		return Xpt{}, err
	}
	ac, err := mul(a.W, c.W)
	if err != nil {
		return Xpt{}, err
	}
	ab, err := mul(a.W, b.W)
	if err != nil {
		return Xpt{}, err
	}
	axis := func(x, y, z *big.Int) (*big.Int, error) {
		p, e := mul(x, bc)
		if e != nil {
			return nil, e
		}
		q, e := mul(y, ac)
		if e != nil {
			return nil, e
		}
		p, e = add(p, q)
		if e != nil {
			return nil, e
		}
		q, e = mul(z, ab)
		if e != nil {
			return nil, e
		}
		return add(p, q)
	}
	x, err := axis(a.X, b.X, c.X)
	if err != nil {
		return Xpt{}, err
	}
	y, err := axis(a.Y, b.Y, c.Y)
	if err != nil {
		return Xpt{}, err
	}
	z, err := axis(a.Z, b.Z, c.Z)
	if err != nil {
		return Xpt{}, err
	}
	w, err := mul(ab, c.W)
	if err != nil {
		return Xpt{}, err
	}
	w, err = mul(w, replayCentroidThree)
	if err != nil {
		return Xpt{}, err
	}
	out := Xpt{X: x, Y: y, Z: z, W: w}
	if err = replayStripTwos(ctx, out, budget); err != nil {
		return Xpt{}, err
	}
	return out, nil
}
