package massmoment

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayPlacementCharge computes the exact placement defect under the shared replay budget.
func ReplayPlacementCharge(ctx context.Context, placement r3.Transform, b *proofbound.ReplayBudget) (MapCharge, error) {
	basis := placement.Basis()
	columns := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	var m [3][3]*big.Rat
	for j, c := range columns {
		for i, f := range [3]float64{c.X, c.Y, c.Z} {
			q, err := proofarith.ReplayFloatRat(ctx, f, b)
			if err != nil {
				return MapCharge{}, err
			}
			m[i][j] = q
		}
	}
	zero, err := proofarith.ReplayFloatRat(ctx, 0, b)
	if err != nil {
		return MapCharge{}, err
	}
	one, err := proofarith.ReplayFloatRat(ctx, 1, b)
	if err != nil {
		return MapCharge{}, err
	}
	half, err := proofarith.ReplayFloatRat(ctx, 0.5, b)
	if err != nil {
		return MapCharge{}, err
	}
	op := func(a, c *big.Rat, kind byte) (*big.Rat, error) {
		return proofarith.ReplayRatArithmetic(ctx, a, c, kind, b)
	}
	defect := zero
	for i := range 3 {
		for j := range 3 {
			dot := zero
			for k := range 3 {
				p, e := op(m[k][i], m[k][j], '*')
				if e != nil {
					return MapCharge{}, e
				}
				dot, e = op(dot, p, '+')
				if e != nil {
					return MapCharge{}, e
				}
			}
			if i == j {
				dot, err = op(dot, one, '-')
				if err != nil {
					return MapCharge{}, err
				}
			}
			if dot.Sign() < 0 {
				dot.Neg(dot)
			}
			defect, err = op(defect, dot, '+')
			if err != nil {
				return MapCharge{}, err
			}
		}
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, defect, half, b)
	if err != nil {
		return MapCharge{}, err
	}
	if cmp >= 0 {
		return MapCharge{}, fmt.Errorf("%w: replay placement frame defect", decaderr.ErrUnsupported)
	}
	minor := func(i, j, k, l int) (*big.Rat, error) {
		a, e := op(m[i][k], m[j][l], '*')
		if e != nil {
			return nil, e
		}
		c, e := op(m[i][l], m[j][k], '*')
		if e != nil {
			return nil, e
		}
		return op(a, c, '-')
	}
	a, err := minor(1, 2, 1, 2)
	if err != nil {
		return MapCharge{}, err
	}
	a, err = op(m[0][0], a, '*')
	if err != nil {
		return MapCharge{}, err
	}
	c, err := minor(1, 2, 0, 2)
	if err != nil {
		return MapCharge{}, err
	}
	c, err = op(m[0][1], c, '*')
	if err != nil {
		return MapCharge{}, err
	}
	a, err = op(a, c, '-')
	if err != nil {
		return MapCharge{}, err
	}
	c, err = minor(1, 2, 0, 1)
	if err != nil {
		return MapCharge{}, err
	}
	c, err = op(m[0][2], c, '*')
	if err != nil {
		return MapCharge{}, err
	}
	a, err = op(a, c, '+')
	if err != nil {
		return MapCharge{}, err
	}
	if a.Sign() < 0 {
		a.Neg(a)
	}
	a, err = op(a, one, '-')
	if err != nil {
		return MapCharge{}, err
	}
	if a.Sign() < 0 {
		a.Neg(a)
	}
	up := func(q *big.Rat) (float64, error) {
		f, e := proofarith.ReplayRatFloat64(ctx, q, b)
		if e != nil {
			return 0, e
		}
		if math.IsInf(f, 0) {
			return f, nil
		}
		r, e := proofarith.ReplayFloatRat(ctx, f, b)
		if e != nil {
			return 0, e
		}
		n, e := proofarith.ReplayRatCompare(ctx, r, q, b)
		if e != nil {
			return 0, e
		}
		if n < 0 {
			f = math.Nextafter(f, math.Inf(1))
		}
		return f, nil
	}
	volume, err := up(a)
	if err != nil {
		return MapCharge{}, err
	}
	stretch, err := up(defect)
	return MapCharge{Volume: volume, Stretch: stretch}, err
}
