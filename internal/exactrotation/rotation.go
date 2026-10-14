// Package exactrotation builds the exactly orthogonal rational rotation that an
// exact-rotation copy denotes. Plain data in, plain data out.
package exactrotation

import (
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// gapLimit is the largest entrywise-sum disagreement between the exact matrix
// and r3's float matrix that New tolerates. It only decides when to refuse.
const gapLimit = 0x1p-30

// Rotation is an exactly orthogonal rational rotation with a rational
// translation. Its fields are unexported; New is the only constructor. A
// Rotation is immutable and every accessor returns copies.
type Rotation struct {
	m [3][3]*big.Rat
	t [3]*big.Rat
}

// New returns the rotation about the axis through center along axis by angle,
// together with r3's float transform for the same rotation.
//
// The matrix is the rational quaternion rotation of (cos(angle/2), sin(angle/2)
// times the axis direction). It is exactly orthogonal, has determinant +1 and
// fixes the axis exactly. It agrees with angle to within a few float steps; it
// is not in general the rotation by exactly that transcendental angle. The
// translation is center minus the matrix applied to center, exactly.
//
// The axis is read as exact rationals and divided by its largest component, so
// an axis of any finite magnitude keeps its exact direction. No float norm of
// the raw axis is ever formed.
//
// The returned transform is r3.RotationAround(center, axis, angle). New refuses
// with ErrUnsupported when the exact matrix disagrees with it by more than
// 2^-30. That check only refuses: the matrix is exact by construction, so a
// small gap admits nothing.
func New(center, axis r3.Vec, angle units.Value) (*Rotation, r3.Transform, error) {
	d, err := r3.RotationAround(center, axis, angle)
	if err != nil {
		return nil, r3.Transform{}, mapR3Error(err)
	}
	r, err := newAgainst(center, axis, angle, d)
	if err != nil {
		return nil, r3.Transform{}, err
	}
	return r, d, nil
}

// newAgainst builds the rotation and applies the gap falsifier against d.
func newAgainst(center, axis r3.Vec, angle units.Value, d r3.Transform) (*Rotation, error) {
	theta, err := angle.In(units.Radian)
	if err != nil {
		return nil, fmt.Errorf("%w: the rotation angle is not an angle: %w", decaderr.ErrUnitKind, err)
	}
	a, ok := ratVec(axis)
	c, cok := ratVec(center)
	if !ok || !cok || !isFinite(theta) {
		return nil, fmt.Errorf("%w: rotation input is not finite", decaderr.ErrNotFinite)
	}
	u, err := scaledAxis(a)
	if err != nil {
		return nil, err
	}
	sh, ch := math.Sincos(theta / 2)
	// nu approximates |u|, which lies in [1, sqrt(3)]. It only sets the
	// quaternion's scale; the direction u is exact.
	nu := 0.0
	for _, q := range u {
		f, _ := q.Float64()
		nu += f * f
	}
	nu = math.Sqrt(nu)
	sigma := sh / nu
	if !isFinite(sh) || !isFinite(ch) || !isFinite(nu) || nu < 1 || !isFinite(sigma) {
		return nil, fmt.Errorf("%w: the rotation quaternion is not finite", decaderr.ErrNotFinite)
	}
	w := proof.FloatRat(ch)
	s := proof.FloatRat(sigma)
	v := [3]*big.Rat{}
	for i := range v {
		v[i] = new(big.Rat).Mul(s, u[i])
	}
	r := &Rotation{m: quaternionMatrix(w, v)}
	for i := range r.t {
		r.t[i] = new(big.Rat).Set(c[i])
		for j := range c {
			r.t[i].Sub(r.t[i], new(big.Rat).Mul(r.m[i][j], c[j]))
		}
	}
	linear, _, ok := r.Gap(d)
	if !ok {
		return nil, fmt.Errorf("%w: the rotation gap is not finite", decaderr.ErrNotFinite)
	}
	if linear > gapLimit {
		return nil, fmt.Errorf("%w: the exact rotation departs from r3's matrix by %g", decaderr.ErrUnsupported, linear)
	}
	return r, nil
}

// scaledAxis divides the exact axis by its largest component.
func scaledAxis(a [3]*big.Rat) ([3]*big.Rat, error) {
	var top *big.Rat
	for _, q := range a {
		abs := new(big.Rat).Abs(q)
		if top == nil || abs.Cmp(top) > 0 {
			top = abs
		}
	}
	if top.Sign() == 0 {
		return a, fmt.Errorf("%w: the rotation axis names no direction", decaderr.ErrDegenerate)
	}
	var u [3]*big.Rat
	for i, q := range a {
		u[i] = new(big.Rat).Quo(q, top)
	}
	return u, nil
}

// quaternionMatrix is the rotation matrix of the quaternion (w, v), divided by
// its squared norm so that w and v need not be a unit quaternion.
func quaternionMatrix(w *big.Rat, v [3]*big.Rat) [3][3]*big.Rat {
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	ww, xx, yy, zz := mul(w, w), mul(v[0], v[0]), mul(v[1], v[1]), mul(v[2], v[2])
	n := add(add(ww, xx), add(yy, zz))
	two := big.NewRat(2, 1)
	cross := func(a, b, c, d *big.Rat, plus bool) *big.Rat {
		// 2(ab ± cd)
		ab, cd := mul(a, b), mul(c, d)
		if plus {
			return mul(two, add(ab, cd))
		}
		return mul(two, sub(ab, cd))
	}
	x, y, z := v[0], v[1], v[2]
	m := [3][3]*big.Rat{
		{sub(add(ww, xx), add(yy, zz)), cross(x, y, w, z, false), cross(x, z, w, y, true)},
		{cross(x, y, w, z, true), sub(add(ww, yy), add(xx, zz)), cross(y, z, w, x, false)},
		{cross(x, z, w, y, false), cross(y, z, w, x, true), sub(add(ww, zz), add(xx, yy))},
	}
	for i := range m {
		for j := range m[i] {
			m[i][j].Quo(m[i][j], n)
		}
	}
	return m
}

// Linear returns fresh copies of the exact linear part, row-major.
func (r *Rotation) Linear() [3][3]*big.Rat {
	var out [3][3]*big.Rat
	for i := range out {
		for j := range out[i] {
			out[i][j] = new(big.Rat).Set(r.m[i][j])
		}
	}
	return out
}

// Translation returns fresh copies of the exact translation.
func (r *Rotation) Translation() [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat).Set(r.t[i])
	}
	return out
}

// FixesXAxisAtOrigin reports whether row 0 and column 0 of the matrix are
// exactly the X unit vector and every translation component is exactly zero.
// It is a structural check on an exact record, never a residual.
func (r *Rotation) FixesXAxisAtOrigin() bool {
	one := big.NewRat(1, 1)
	if r.m[0][0].Cmp(one) != 0 {
		return false
	}
	for k := 1; k < 3; k++ {
		if r.m[0][k].Sign() != 0 || r.m[k][0].Sign() != 0 {
			return false
		}
	}
	for _, q := range r.t {
		if q.Sign() != 0 {
			return false
		}
	}
	return true
}

// Gap bounds how far d lies from the exact record. linear is an upper bound on
// the entrywise absolute sum of m - D, so |(m - D)v| <= linear * max|v_j|.
// translation bounds the entrywise absolute sum of t - tD. ok is false when
// either bound is not finite.
func (r *Rotation) Gap(d r3.Transform) (linear, translation float64, ok bool) {
	basis := d.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	lin := new(big.Rat)
	for j, col := range cols {
		dv, vok := ratVec(col)
		if !vok {
			return 0, 0, false
		}
		for i := range dv {
			lin.Add(lin, absDiff(r.m[i][j], dv[i]))
		}
	}
	tv, tok := ratVec(d.Translation())
	if !tok {
		return 0, 0, false
	}
	tr := new(big.Rat)
	for i := range tv {
		tr.Add(tr, absDiff(r.t[i], tv[i]))
	}
	linear, translation = proofbound.RatFloatUp(lin), proofbound.RatFloatUp(tr)
	if !isFinite(linear) || !isFinite(translation) {
		return 0, 0, false
	}
	return linear, translation, true
}

func absDiff(a, b *big.Rat) *big.Rat {
	d := new(big.Rat).Sub(a, b)
	return d.Abs(d)
}

func ratVec(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}

func isFinite(f float64) bool { return !math.IsNaN(f) && !math.IsInf(f, 0) }

// mapR3Error maps r3's refusals onto the decad error kinds.
func mapR3Error(err error) error {
	switch {
	case errors.Is(err, r3.ErrDegenerateAxis):
		return fmt.Errorf("%w: the rotation axis names no direction: %w", decaderr.ErrDegenerate, err)
	case errors.Is(err, units.ErrIncompatible):
		return fmt.Errorf("%w: the rotation angle is not an angle: %w", decaderr.ErrUnitKind, err)
	default:
		return fmt.Errorf("%w: the rotation is not representable: %w", decaderr.ErrNotFinite, err)
	}
}
