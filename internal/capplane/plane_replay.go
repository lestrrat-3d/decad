package capplane

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type replayArithmetic struct {
	ctx    context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	budget proof.ReplayKeyBudget
}

func (c replayArithmetic) operation(a, b *big.Rat, op byte) (*big.Rat, error) {
	return proof.ReplayRatArithmetic(c.ctx, a, b, op, c.budget)
}

func (c replayArithmetic) vector(v r3.Vec) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	for i, x := range [3]float64{v.X, v.Y, v.Z} {
		q, err := proof.ReplayFloatRat(c.ctx, x, c.budget)
		if err != nil {
			return out, err
		}
		out[i] = q
	}
	return out, nil
}

func (c replayArithmetic) cross(a, b [3]*big.Rat) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		left, err := c.operation(a[j], b[k], '*')
		if err != nil {
			return out, err
		}
		right, err := c.operation(a[k], b[j], '*')
		if err != nil {
			return out, err
		}
		out[i], err = c.operation(left, right, '-')
		if err != nil {
			return out, err
		}
	}
	return out, nil
}

func (c replayArithmetic) dot(a, b [3]*big.Rat) (*big.Rat, error) {
	var sum *big.Rat
	for i := range a {
		term, err := c.operation(a[i], b[i], '*')
		if err != nil {
			return nil, err
		}
		if sum == nil {
			sum = term
		} else {
			sum, err = c.operation(sum, term, '+')
			if err != nil {
				return nil, err
			}
		}
	}
	return sum, nil
}

func (c replayArithmetic) apply(x r3.Transform, v [3]*big.Rat, point bool) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	var err error
	if point {
		out, err = c.vector(x.Translation())
	} else {
		out, err = c.vector(r3.Vec{})
	}
	if err != nil {
		return out, err
	}
	basis := x.Basis()
	for i, axis := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		a, err := c.vector(axis)
		if err != nil {
			return out, err
		}
		for j := range out {
			term, err := c.operation(a[j], v[i], '*')
			if err != nil {
				return out, err
			}
			out[j], err = c.operation(out[j], term, '+')
			if err != nil {
				return out, err
			}
		}
	}
	return out, nil
}

// ReplayOfFace reads a cap's exact denoted plane with all rational arithmetic
// charged to budget. The face and its immutable record must not be mutated.
func ReplayOfFace(ctx context.Context, face *topo.Face, budget proof.ReplayKeyBudget) (Plane, bool, error) {
	if budget == nil {
		return Plane{}, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return Plane{}, false, err
	}
	if face == nil || face.CapRecord == nil {
		return Plane{}, false, nil
	}
	r := face.CapRecord
	z := r.ZDenoted
	if z == nil {
		if !face.HasAxialDelta || face.AxialDelta != 0 {
			return Plane{}, false, nil
		}
		var err error
		z, err = proof.ReplayFloatRat(ctx, r.Z, budget)
		if err != nil {
			return Plane{}, false, err
		}
	}
	c := replayArithmetic{ctx, budget}
	u, err := c.vector(r.Frame.U())
	if err != nil {
		return Plane{}, false, err
	}
	v, err := c.vector(r.Frame.V())
	if err != nil {
		return Plane{}, false, err
	}
	n, err := c.vector(r.Frame.N())
	if err != nil {
		return Plane{}, false, err
	}
	origin, err := c.vector(r.Frame.Origin())
	if err != nil {
		return Plane{}, false, err
	}
	u, err = c.apply(r.Xform, u, false)
	if err != nil {
		return Plane{}, false, err
	}
	v, err = c.apply(r.Xform, v, false)
	if err != nil {
		return Plane{}, false, err
	}
	normal, err := c.cross(u, v)
	if err != nil {
		return Plane{}, false, err
	}
	if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
		return Plane{}, false, nil
	}
	for i := range origin {
		part, err := c.operation(z, n[i], '*')
		if err != nil {
			return Plane{}, false, err
		}
		origin[i], err = c.operation(origin[i], part, '+')
		if err != nil {
			return Plane{}, false, err
		}
	}
	point, err := c.apply(r.Xform, origin, true)
	if err != nil {
		return Plane{}, false, err
	}
	return Plane{normal: normal, point: point}, true, nil
}

// ReplaySame compares two planes using charged exact arithmetic.
func ReplaySame(ctx context.Context, a, b Plane, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	for i := range a.normal {
		if a.normal[i] == nil || b.normal[i] == nil || a.point[i] == nil || b.point[i] == nil {
			return false, nil
		}
	}
	c := replayArithmetic{ctx, budget}
	cross, err := c.cross(a.normal, b.normal)
	if err != nil {
		return false, err
	}
	if cross[0].Sign() != 0 || cross[1].Sign() != 0 || cross[2].Sign() != 0 {
		return false, nil
	}
	var difference [3]*big.Rat
	for i := range difference {
		difference[i], err = c.operation(b.point[i], a.point[i], '-')
		if err != nil {
			return false, err
		}
	}
	dot, err := c.dot(a.normal, difference)
	if err != nil {
		return false, err
	}
	return dot.Sign() == 0, nil
}
