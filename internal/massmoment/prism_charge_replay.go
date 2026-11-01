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

// These refusals are constructed once so rejected replay does not allocate.
var (
	errPrismChargeReplay1 = fmt.Errorf("%w: replay placement frame defect", decaderr.ErrUnsupported)
	errPrismChargeReplay2 = fmt.Errorf("%w: missing prism charge budget", decaderr.ErrUnsupported)
	errPrismChargeReplay3 = fmt.Errorf("%w: invalid prism chord", decaderr.ErrUnsupported)
	errPrismChargeReplay4 = fmt.Errorf("%w: invalid prism lift", decaderr.ErrUnsupported)
	errPrismChargeReplay5 = fmt.Errorf("%w: invalid local chord", decaderr.ErrUnsupported)
)

// ReplayPlacementCharge computes the exact placement defect under the shared replay budget.
func ReplayPlacementCharge(ctx context.Context, placement r3.Transform, b *proofbound.ReplayBudget) (MapCharge, error) {
	if b == nil {
		return MapCharge{}, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return MapCharge{}, err
	}
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
	return replayMatrixCharge(ctx, m, b)
}

func replayMatrixCharge(ctx context.Context, m [3][3]*big.Rat, b *proofbound.ReplayBudget) (MapCharge, error) {
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
		return MapCharge{}, errPrismChargeReplay1
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

// ReplayPrismCharge composes the exact held frame and placement before
// deriving the stretch and volume charges.
func ReplayPrismCharge(ctx context.Context, frame r3.Frame, placement r3.Transform, b *proofbound.ReplayBudget) (MapCharge, error) {
	if b == nil {
		return MapCharge{}, errPrismChargeReplay2
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return MapCharge{}, err
	}
	basis := placement.Basis()
	world := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	local := [3]r3.Vec{frame.U(), frame.V(), frame.N()}
	var a, c [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			wf := [3]float64{world[j].X, world[j].Y, world[j].Z}
			lf := [3]float64{local[j].X, local[j].Y, local[j].Z}
			var err error
			a[i][j], err = proofarith.ReplayFloatRat(ctx, wf[i], b)
			if err != nil {
				return MapCharge{}, err
			}
			c[i][j], err = proofarith.ReplayFloatRat(ctx, lf[i], b)
			if err != nil {
				return MapCharge{}, err
			}
		}
	}
	zero, err := proofarith.ReplayFloatRat(ctx, 0, b)
	if err != nil {
		return MapCharge{}, err
	}
	var matrix [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			sum := zero
			for k := range 3 {
				term, e := proofarith.ReplayRatArithmetic(ctx, a[i][k], c[k][j], '*', b)
				if e != nil {
					return MapCharge{}, e
				}
				sum, e = proofarith.ReplayRatArithmetic(ctx, sum, term, '+', b)
				if e != nil {
					return MapCharge{}, e
				}
			}
			matrix[i][j] = sum
		}
	}
	return replayMatrixCharge(ctx, matrix, b)
}

// ReplayPrismChord bounds the local chord so its image stays within chord.
func ReplayPrismChord(ctx context.Context, frame r3.Frame, placement r3.Transform, chord float64, b *proofbound.ReplayBudget) (float64, float64, MapCharge, error) {
	if b == nil {
		return 0, 0, MapCharge{}, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 8); err != nil {
		return 0, 0, MapCharge{}, err
	}
	if !(chord > 0) || math.IsInf(chord, 0) || math.IsNaN(chord) {
		return 0, 0, MapCharge{}, errPrismChargeReplay3
	}
	charge, err := ReplayPrismCharge(ctx, frame, placement, b)
	if err != nil {
		return 0, 0, MapCharge{}, err
	}
	if err := b.SpendWork(ctx, 16); err != nil {
		return 0, 0, MapCharge{}, err
	}
	lift := 1.0
	if charge.Stretch != 0 {
		lift = proofbound.AbsSumUpper(1, charge.Stretch)
	}
	if math.IsNaN(lift) || math.IsInf(lift, 0) || lift < 1 {
		return 0, 0, MapCharge{}, errPrismChargeReplay4
	}
	local := chord
	if lift != 1 {
		local = math.Nextafter(chord/lift, 0)
	}
	if !(local > 0) || math.IsInf(local, 0) {
		return 0, 0, MapCharge{}, errPrismChargeReplay5
	}
	return local, lift, charge, nil
}
