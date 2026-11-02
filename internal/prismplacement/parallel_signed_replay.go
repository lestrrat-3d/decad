package prismplacement

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// ReplaySignedMap is the exact section permutation, translation, and axial shift.
// The rational offsets belong to the caller and must not be mutated.
type ReplaySignedMap struct {
	// Source U and V each map to a signed target U or V: +/-1 and +/-2.
	U, V                    int
	OffsetU, OffsetV, Shift *big.Rat
}

type replayVec [3]*big.Rat

type replayCalc struct {
	ctx    context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	budget proof.ReplayKeyBudget
}

func (c replayCalc) f(v float64) (*big.Rat, error) { return proof.ReplayFloatRat(c.ctx, v, c.budget) }
func (c replayCalc) op(a, b *big.Rat, op byte) (*big.Rat, error) {
	return proof.ReplayRatArithmetic(c.ctx, a, b, op, c.budget)
}
func (c replayCalc) vec(v r3.Vec) (replayVec, error) {
	var out replayVec
	for i, x := range [3]float64{v.X, v.Y, v.Z} {
		q, e := c.f(x)
		if e != nil {
			return out, e
		}
		out[i] = q
	}
	return out, nil
}
func (c replayCalc) combine(a, b replayVec, op byte) (replayVec, error) {
	var out replayVec
	for i := range out {
		q, e := c.op(a[i], b[i], op)
		if e != nil {
			return out, e
		}
		out[i] = q
	}
	return out, nil
}
func (c replayCalc) mul(a, b *big.Rat) (*big.Rat, error) { return c.op(a, b, '*') }
func (c replayCalc) dot(a, b replayVec) (*big.Rat, error) {
	var sum *big.Rat
	for i := range a {
		q, e := c.mul(a[i], b[i])
		if e != nil {
			return nil, e
		}
		if sum == nil {
			sum = q
		} else {
			sum, e = c.op(sum, q, '+')
			if e != nil {
				return nil, e
			}
		}
	}
	return sum, nil
}
func (c replayCalc) cross(a, b replayVec) (replayVec, error) {
	var out replayVec
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		l, e := c.mul(a[j], b[k])
		if e != nil {
			return out, e
		}
		r, e := c.mul(a[k], b[j])
		if e != nil {
			return out, e
		}
		out[i], e = c.op(l, r, '-')
		if e != nil {
			return out, e
		}
	}
	return out, nil
}
func (c replayCalc) equal(a, b replayVec, sign int) (bool, error) {
	for i := range a {
		op := '-'
		if sign < 0 {
			op = '+'
		}
		diff, e := c.op(a[i], b[i], byte(op))
		if e != nil {
			return false, e
		}
		if diff.Sign() != 0 {
			return false, nil
		}
	}
	return true, nil
}
func (c replayCalc) apply(x r3.Transform, v r3.Vec, point bool) (replayVec, error) {
	basis := x.Basis()
	axes := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	coeff := [3]float64{v.X, v.Y, v.Z}
	var out replayVec
	if point {
		var e error
		out, e = c.vec(x.Translation())
		if e != nil {
			return out, e
		}
	} else {
		var e error
		out, e = c.vec(r3.Vec{})
		if e != nil {
			return out, e
		}
	}
	for i := range axes {
		axis, e := c.vec(axes[i])
		if e != nil {
			return out, e
		}
		q, e := c.f(coeff[i])
		if e != nil {
			return out, e
		}
		for j := range out {
			term, e := c.mul(axis[j], q)
			if e != nil {
				return out, e
			}
			out[j], e = c.op(out[j], term, '+')
			if e != nil {
				return out, e
			}
		}
	}
	return out, nil
}

// ReplayWorldIsometry proves that the exact composed world basis preserves
// lengths and orientation. Rounded rotations may fail this exact gate.
func ReplayWorldIsometry(ctx context.Context, operand Operand, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	c := replayCalc{ctx, budget}
	var basis [3]replayVec
	for i, axis := range [3]r3.Vec{operand.Frame.U(), operand.Frame.V(), operand.Frame.N()} {
		var err error
		basis[i], err = c.apply(operand.Xform, axis, false)
		if err != nil {
			return false, err
		}
	}
	zero, err := c.f(0)
	if err != nil {
		return false, err
	}
	one, err := c.f(1)
	if err != nil {
		return false, err
	}
	for i := range basis {
		for j := i; j < len(basis); j++ {
			dot, err := c.dot(basis[i], basis[j])
			if err != nil {
				return false, err
			}
			want := zero
			if i == j {
				want = one
			}
			order, err := proof.ReplayRatCompare(ctx, dot, want, budget)
			if err != nil || order != 0 {
				return false, err
			}
		}
	}
	cross, err := c.cross(basis[1], basis[2])
	if err != nil {
		return false, err
	}
	det, err := c.dot(basis[0], cross)
	if err != nil {
		return false, err
	}
	order, err := proof.ReplayRatCompare(ctx, det, one, budget)
	return order == 0 && err == nil, err
}

// ReplayParallelSignedMap certifies exact, equal directed sweep vectors and a
// signed-permutation section map. It charges all rational arithmetic to budget.
func ReplayParallelSignedMap(ctx context.Context, a, b Operand, budget proof.ReplayKeyBudget) (ReplaySignedMap, bool, error) {
	var result ReplaySignedMap
	if budget == nil {
		return result, false, nil
	}
	c := replayCalc{ctx, budget}
	au, e := c.apply(a.Xform, a.Frame.U(), false)
	if e != nil {
		return result, false, e
	}
	av, e := c.apply(a.Xform, a.Frame.V(), false)
	if e != nil {
		return result, false, e
	}
	an, e := c.apply(a.Xform, a.Frame.N(), false)
	if e != nil {
		return result, false, e
	}
	bu, e := c.apply(b.Xform, b.Frame.U(), false)
	if e != nil {
		return result, false, e
	}
	bv, e := c.apply(b.Xform, b.Frame.V(), false)
	if e != nil {
		return result, false, e
	}
	bn, e := c.apply(b.Xform, b.Frame.N(), false)
	if e != nil {
		return result, false, e
	}
	equal, e := c.equal(an, bn, 1)
	if e != nil || !equal {
		return result, false, e
	}
	for _, axis := range []replayVec{au, av} {
		d, e := c.dot(axis, an)
		if e != nil {
			return result, false, e
		}
		if d.Sign() != 0 {
			return result, false, nil
		}
	}
	for i, source := range []replayVec{bu, bv} {
		chosen := 0
		for j, target := range []replayVec{au, av} {
			for _, sign := range []int{1, -1} {
				yes, e := c.equal(source, target, sign)
				if e != nil {
					return result, false, e
				}
				if yes {
					if chosen != 0 {
						return result, false, nil
					}
					chosen = sign * (j + 1)
				}
			}
		}
		if chosen == 0 {
			return result, false, nil
		}
		if i == 0 {
			result.U = chosen
		} else {
			result.V = chosen
		}
	}
	if result.U == result.V || result.U == -result.V {
		return ReplaySignedMap{}, false, nil
	}
	ao, e := c.apply(a.Xform, a.Frame.Origin(), true)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	bo, e := c.apply(b.Xform, b.Frame.Origin(), true)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	d, e := c.combine(bo, ao, '-')
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	vn, e := c.cross(av, an)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	det, e := c.dot(au, vn)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	if det.Sign() == 0 {
		return ReplaySignedMap{}, false, nil
	}
	num, e := c.dot(d, vn)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	result.OffsetU, e = c.op(num, det, '/')
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	dn, e := c.cross(d, an)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	num, e = c.dot(au, dn)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	result.OffsetV, e = c.op(num, det, '/')
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	vd, e := c.cross(av, d)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	num, e = c.dot(au, vd)
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	result.Shift, e = c.op(num, det, '/')
	if e != nil {
		return ReplaySignedMap{}, false, e
	}
	return result, true, nil
}

// ReplaySameShiftedInterval checks both exact denoted sweep levels after the
// certified shift. The operands' level displacement records are preserved.
func ReplaySameShiftedInterval(ctx context.Context, a, b Operand, shift *big.Rat, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil || shift == nil {
		return false, nil
	}
	for _, pair := range []struct {
		ah, ad, bh, bd float64
		aDen, bDen     LevelDenotation
	}{{a.Z0, a.Z0Delta, b.Z0, b.Z0Delta, a.Z0Denoted, b.Z0Denoted},
		{a.Z1, a.Z1Delta, b.Z1, b.Z1Delta, a.Z1Denoted, b.Z1Denoted}} {
		av, ok, err := ReplayDenotedLevel(ctx, pair.ah, pair.ad, pair.aDen, budget)
		if err != nil || !ok {
			return false, err
		}
		bv, ok, err := ReplayDenotedLevel(ctx, pair.bh, pair.bd, pair.bDen, budget)
		if err != nil || !ok {
			return false, err
		}
		shifted, err := proof.ReplayRatArithmetic(ctx, bv, shift, '+', budget)
		if err != nil {
			return false, err
		}
		cmp, err := proof.ReplayRatCompare(ctx, av, shifted, budget)
		if err != nil || cmp != 0 {
			return false, err
		}
	}
	return true, nil
}
