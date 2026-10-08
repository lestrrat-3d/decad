package stitchflux

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ExactRevolve is a revolved record whose axis, basis and meridian leaves
// read as exact rational points.
type ExactRevolve struct {
	O, W     [3]*big.Rat
	e0, e1   [3]*big.Rat
	Ends     [][2][2]*big.Rat
	Circular bool
	Centre   [2]*big.Rat
	Radius   *big.Rat
}

// ExactRevolveOf refuses a non-point leaf or a basis that is not exactly
// orthonormal. The returned values are the record's own exact rationals.
func ExactRevolveOf(r surfacenormal.Revolved) (ExactRevolve, bool) {
	var ex ExactRevolve
	vecs := [...]*[3]*big.Rat{&ex.O, &ex.e0, &ex.e1, &ex.W}
	for i, iv := range [...]proofbound.IvVec3{r.Origin, r.Basis[0], r.Basis[1], r.Basis[2]} {
		for k := range 3 {
			x, ok := RatPoint(iv[k])
			if !ok {
				return ExactRevolve{}, false
			}
			vecs[i][k] = x
		}
	}
	basis := [3][3]*big.Rat{ex.e0, ex.e1, ex.W}
	for i := range basis {
		for j := range basis {
			want := int64(0)
			if i == j {
				want = 1
			}
			if ratDot(basis[i], basis[j]).Cmp(big.NewRat(want, 1)) != 0 {
				return ExactRevolve{}, false
			}
		}
	}
	ex.Circular = r.Circular
	if r.Circular {
		zc, okZ := RatPoint(r.Centre[0])
		rc, okR := RatPoint(r.Centre[1])
		if !okZ || !okR {
			return ExactRevolve{}, false
		}
		ex.Centre = [2]*big.Rat{zc, rc}
		// The Sphere arm derives its radius from area; the Torus arm reads
		// this leaf only when it is a point.
		if radius, ok := RatPoint(r.Radius); ok {
			ex.Radius = radius
		}
		return ex, true
	}
	for _, seg := range r.Ends {
		var out [2][2]*big.Rat
		for i := range seg {
			for k := range 2 {
				x, ok := RatPoint(seg[i][k])
				if !ok {
					return ExactRevolve{}, false
				}
				out[i][k] = x
			}
		}
		ex.Ends = append(ex.Ends, out)
	}
	return ex, true
}

func (ex ExactRevolve) axisPoint(z *big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for k := range 3 {
		out[k] = new(big.Rat).Add(ex.O[k], new(big.Rat).Mul(ex.W[k], z))
	}
	return out
}

// AtAxis reports whether p is exactly the axis point at z.
func (ex ExactRevolve) AtAxis(p r3.Vec, z *big.Rat) bool {
	q, ok := RatVecOf(p)
	return ok && ratVecEqual(q, ex.axisPoint(z))
}

// OnAxis reports whether p lies exactly on the axis line.
func (ex ExactRevolve) OnAxis(p r3.Vec) bool {
	q, ok := RatVecOf(p)
	if !ok {
		return false
	}
	d := ratSub(q, ex.O)
	c := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(d[1], ex.W[2]), new(big.Rat).Mul(d[2], ex.W[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(d[2], ex.W[0]), new(big.Rat).Mul(d[0], ex.W[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(d[0], ex.W[1]), new(big.Rat).Mul(d[1], ex.W[0])),
	}
	return c[0].Sign() == 0 && c[1].Sign() == 0 && c[2].Sign() == 0
}

// AlongAxis reports whether a is exactly W or its negation.
func (ex ExactRevolve) AlongAxis(a r3.Vec) bool {
	q, ok := RatVecOf(a)
	if !ok {
		return false
	}
	neg := [3]*big.Rat{new(big.Rat).Neg(ex.W[0]), new(big.Rat).Neg(ex.W[1]), new(big.Rat).Neg(ex.W[2])}
	return ratVecEqual(q, ex.W) || ratVecEqual(q, neg)
}

// RimAtEnd checks a circular rim against the recorded meridian ends.
func (ex ExactRevolve) RimAtEnd(radius units.Value, center r3.Vec) bool {
	r, okR := RatMillimetres(radius)
	c, okC := RatVecOf(center)
	if !okR || !okC {
		return false
	}
	for _, seg := range ex.Ends {
		for _, end := range seg {
			if end[1].Cmp(r) == 0 && ratVecEqual(c, ex.axisPoint(end[0])) {
				return true
			}
		}
	}
	return false
}

// RatPoint returns an interval's exact value when both endpoints agree.
func RatPoint(iv proofbound.RatInterval) (*big.Rat, bool) {
	if iv.Lo == nil || iv.Hi == nil || iv.Lo.Cmp(iv.Hi) != 0 {
		return nil, false
	}
	return iv.Lo, true
}

// RatMillimetres reads a quantity's held millimetres as an exact rational.
func RatMillimetres(v units.Value) (*big.Rat, bool) {
	mm, err := v.In(units.Millimeter)
	if err != nil {
		return nil, false
	}
	r := proofarith.FloatRat(mm)
	return r, r != nil
}

// RatVecOf reads a held vector as exact rational coordinates.
func RatVecOf(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}

func ratVecEqual(a, b [3]*big.Rat) bool {
	return a[0].Cmp(b[0]) == 0 && a[1].Cmp(b[1]) == 0 && a[2].Cmp(b[2]) == 0
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]),
		new(big.Rat).Add(new(big.Rat).Mul(a[1], b[1]), new(big.Rat).Mul(a[2], b[2])))
}

func ratSub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}
