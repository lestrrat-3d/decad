package anasurf

import (
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// ErrSingular marks three planes whose normals are linearly dependent in exact arithmetic.
var ErrSingular = errors.New("anasurf: the planes do not meet in one point")

// Plane is {x : N·x = c} for one c in C, with the material on the side N·x < c. N is exact and not unit.
type Plane struct {
	N [3]*big.Rat
	C proofbound.RatInterval
}

func ratVec(v r3.Vec) ([3]*big.Rat, bool) {
	x, okx := proofbound.RatOf(v.X)
	y, oky := proofbound.RatOf(v.Y)
	z, okz := proofbound.RatOf(v.Z)
	return [3]*big.Rat{x, y, z}, okx && oky && okz
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	s := new(big.Rat)
	for i := range a {
		s.Add(s, new(big.Rat).Mul(a[i], b[i]))
	}
	return s
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
}

// PlaneOf reads a tag: N = U×V of the frame in exact rationals, negated when reversed; C is the point interval N·O.
func PlaneOf(tag surfacegeom.Plane, reversed bool) (Plane, error) {
	u, oku := ratVec(tag.Frame.U())
	v, okv := ratVec(tag.Frame.V())
	o, oko := ratVec(tag.Frame.Origin())
	if !oku || !okv || !oko {
		return Plane{}, fmt.Errorf("%w: a plane frame holds a non-finite component", ErrSingular)
	}
	n := ratCross(u, v)
	if reversed {
		for i := range n {
			n[i] = new(big.Rat).Neg(n[i])
		}
	}
	if n[0].Sign() == 0 && n[1].Sign() == 0 && n[2].Sign() == 0 {
		return Plane{}, fmt.Errorf("%w: a plane frame has a zero normal", ErrSingular)
	}
	return Plane{N: n, C: proofbound.PointInterval(ratDot(n, o))}, nil
}

// Norm encloses |N| by proofbound.IntervalSqrt of N·N.
func (p Plane) Norm() (proofbound.RatInterval, error) {
	nn, ok := proofbound.IntervalSqrt(proofbound.PointInterval(ratDot(p.N, p.N)))
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf("%w: the normal length has no enclosure", ErrSingular)
	}
	return nn, nil
}

// Inward moves p a distance t into its material: C − t·|N|, |N| enclosed by proofbound.IntervalSqrt of N·N.
// t must be positive.
func (p Plane) Inward(t *big.Rat) (Plane, error) {
	if t.Sign() <= 0 {
		return Plane{}, fmt.Errorf("anasurf: the offset distance %v is not positive", t)
	}
	norm, err := p.Norm()
	if err != nil {
		return Plane{}, err
	}
	return Plane{N: p.N, C: proofbound.IntervalSub(p.C, proofbound.IntervalScale(norm, t))}, nil
}

// Point is an enclosure of one exact point.
type Point struct {
	Box proofbound.IvVec3
}

// Meet3 encloses the common point of a, b, c by Cramer's rule over exact N rows and interval C. ErrSingular when the
// exact determinant is zero.
func Meet3(a, b, c Plane) (Point, error) {
	m := [3][3]*big.Rat{a.N, b.N, c.N}
	cs := [3]proofbound.RatInterval{a.C, b.C, c.C}
	det := new(big.Rat)
	cof := [3][3]*big.Rat{}
	for i := range 3 {
		for j := range 3 {
			r0, r1 := (i+1)%3, (i+2)%3
			c0, c1 := (j+1)%3, (j+2)%3
			minor := new(big.Rat).Sub(
				new(big.Rat).Mul(m[r0][c0], m[r1][c1]),
				new(big.Rat).Mul(m[r0][c1], m[r1][c0]),
			)
			cof[i][j] = minor
		}
	}
	for j := range 3 {
		det.Add(det, new(big.Rat).Mul(m[0][j], cof[0][j]))
	}
	if det.Sign() == 0 {
		return Point{}, ErrSingular
	}
	var box proofbound.IvVec3
	for i := range 3 {
		// (M⁻¹)[i][j] = cof[j][i] / det.
		acc := proofbound.PointInterval(new(big.Rat))
		for j := range 3 {
			coef := new(big.Rat).Quo(cof[j][i], det)
			acc = proofbound.IntervalAdd(acc, proofbound.IntervalScale(cs[j], coef))
		}
		box[i] = acc
	}
	return Point{Box: box}, nil
}

// Held returns the nearest float vector to the box midpoint and an upward bound on its distance from every box point.
func (p Point) Held() (r3.Vec, float64) {
	var h [3]float64
	var hr [3]*big.Rat
	for i, iv := range p.Box {
		mid := new(big.Rat).Add(iv.Lo, iv.Hi)
		mid.Quo(mid, big.NewRat(2, 1))
		f, _ := mid.Float64()
		h[i] = f
		hr[i], _ = proofbound.RatOf(f)
	}
	sum := new(big.Rat)
	for i, iv := range p.Box {
		d := new(big.Rat).Abs(new(big.Rat).Sub(iv.Lo, hr[i]))
		if e := new(big.Rat).Abs(new(big.Rat).Sub(iv.Hi, hr[i])); e.Cmp(d) > 0 {
			d = e
		}
		sum.Add(sum, d.Mul(d, d))
	}
	return r3.NewVec(h[0], h[1], h[2]), proofbound.RatSqrtUp(sum)
}

// StrictlyInside reports whether every point of the box satisfies N·x < c for every c in p.C:
// max over the box of N·x is below C.Lo, decided exactly.
func StrictlyInside(pt Point, p Plane) bool {
	top := new(big.Rat)
	for i, iv := range pt.Box {
		lo := new(big.Rat).Mul(p.N[i], iv.Lo)
		hi := new(big.Rat).Mul(p.N[i], iv.Hi)
		if lo.Cmp(hi) > 0 {
			hi = lo
		}
		top.Add(top, hi)
	}
	return top.Cmp(p.C.Lo) < 0
}
