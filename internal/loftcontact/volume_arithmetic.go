package loftcontact

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RationalBits is the size ceiling of Arithmetic. It applies to the numerator
// and denominator of every source coordinate and every rational intermediate,
// including unreduced products.
const RationalBits = 8192

// Vec is an exact rational point or direction in loft-contact world space.
type Vec [3]*big.Rat

// Arithmetic refuses before allocating an intermediate whose numerator or
// denominator could exceed RationalBits. OK starts true and is sticky: the
// first refused operand or operation clears it and every later operation
// returns a zero result. Callers construct it as &Arithmetic{OK: true} and
// must read OK before they trust a result.
type Arithmetic struct{ OK bool }

// Valid reports whether a is non-nil and within RationalBits, clearing OK
// when it is not.
func (c *Arithmetic) Valid(a *big.Rat) bool {
	if !c.OK || a == nil || a.Num().BitLen() > RationalBits ||
		a.Denom().BitLen() > RationalBits {
		c.OK = false
	}
	return c.OK
}

// Add returns a+b, or zero after clearing OK when the sum could exceed
// RationalBits.
func (c *Arithmetic) Add(a, b *big.Rat) *big.Rat {
	if !c.Valid(a) || !c.Valid(b) ||
		max(a.Num().BitLen()+b.Denom().BitLen(), b.Num().BitLen()+a.Denom().BitLen())+1 > RationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > RationalBits {
		c.OK = false
		return new(big.Rat)
	}
	return new(big.Rat).Add(a, b)
}

// Sub returns a-b, or zero after clearing OK when the difference could exceed
// RationalBits.
func (c *Arithmetic) Sub(a, b *big.Rat) *big.Rat {
	if !c.Valid(a) || !c.Valid(b) ||
		max(a.Num().BitLen()+b.Denom().BitLen(), b.Num().BitLen()+a.Denom().BitLen())+1 > RationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > RationalBits {
		c.OK = false
		return new(big.Rat)
	}
	return new(big.Rat).Sub(a, b)
}

// Mul returns a*b, or zero after clearing OK when the product could exceed
// RationalBits.
func (c *Arithmetic) Mul(a, b *big.Rat) *big.Rat {
	if !c.Valid(a) || !c.Valid(b) ||
		a.Num().BitLen()+b.Num().BitLen() > RationalBits ||
		a.Denom().BitLen()+b.Denom().BitLen() > RationalBits {
		c.OK = false
		return new(big.Rat)
	}
	return new(big.Rat).Mul(a, b)
}

// Quo returns a/b, or zero after clearing OK when b is zero or the quotient
// could exceed RationalBits.
func (c *Arithmetic) Quo(a, b *big.Rat) *big.Rat {
	if !c.Valid(a) || !c.Valid(b) || b.Sign() == 0 ||
		a.Num().BitLen()+b.Denom().BitLen() > RationalBits ||
		a.Denom().BitLen()+b.Num().BitLen() > RationalBits {
		c.OK = false
		return new(big.Rat)
	}
	return new(big.Rat).Quo(a, b)
}

// Cmp compares a and b, or returns 0 after clearing OK when the cross
// products could exceed RationalBits.
func (c *Arithmetic) Cmp(a, b *big.Rat) int {
	if !c.Valid(a) || !c.Valid(b) ||
		a.Num().BitLen()+b.Denom().BitLen() > RationalBits ||
		b.Num().BitLen()+a.Denom().BitLen() > RationalBits {
		c.OK = false
		return 0
	}
	return a.Cmp(b)
}

// Diff returns a-b componentwise.
func (c *Arithmetic) Diff(a, b Vec) Vec {
	return Vec{c.Sub(a[0], b[0]), c.Sub(a[1], b[1]), c.Sub(a[2], b[2])}
}

// Cross returns the cross product a×b.
func (c *Arithmetic) Cross(a, b Vec) Vec {
	return Vec{
		c.Sub(c.Mul(a[1], b[2]), c.Mul(a[2], b[1])),
		c.Sub(c.Mul(a[2], b[0]), c.Mul(a[0], b[2])),
		c.Sub(c.Mul(a[0], b[1]), c.Mul(a[1], b[0])),
	}
}

// Dot returns the dot product a·b.
func (c *Arithmetic) Dot(a, b Vec) *big.Rat {
	return c.Add(c.Add(c.Mul(a[0], b[0]), c.Mul(a[1], b[1])), c.Mul(a[2], b[2]))
}

// Same reports whether a and b are the same point.
func (c *Arithmetic) Same(a, b Vec) bool {
	return c.Cmp(a[0], b[0]) == 0 && c.Cmp(a[1], b[1]) == 0 && c.Cmp(a[2], b[2]) == 0
}

// OnSegment reports whether p lies on the closed segment from a to b.
func (c *Arithmetic) OnSegment(p, a, b Vec) bool {
	delta := c.Diff(b, a)
	axis := 0
	for axis < 3 && delta[axis].Sign() == 0 {
		axis++
	}
	if axis == 3 {
		return c.Same(p, a)
	}
	t := c.Quo(c.Sub(p[axis], a[axis]), delta[axis])
	if c.Cmp(t, big.NewRat(0, 1)) < 0 || c.Cmp(t, big.NewRat(1, 1)) > 0 {
		return false
	}
	for k := range 3 {
		if c.Cmp(p[k], c.Add(a[k], c.Mul(t, delta[k]))) != 0 {
			return false
		}
	}
	return c.OK
}

// Float64 checks the alignment performed by big.Rat.Float64 before it can
// allocate a temporary wider than RationalBits.
func (c *Arithmetic) Float64(a *big.Rat) (float64, bool) {
	if !c.Valid(a) {
		return 0, false
	}
	if numeratorBits := a.Num().BitLen(); numeratorBits != 0 {
		denominatorBits := a.Denom().BitLen()
		if shift := 54 - (numeratorBits - denominatorBits); shift > 0 &&
			numeratorBits+shift > RationalBits {
			c.OK = false
			return 0, false
		}
	}
	return a.Float64()
}

// PointOf lifts a recorded plane-local point to exact rationals. It reports
// false when a coordinate is not finite or exceeds RationalBits.
func PointOf(p sectionrecord.Point2) (Point, bool) {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	arithmetic := &Arithmetic{OK: true}
	return Point{u, v}, arithmetic.Valid(u) && arithmetic.Valid(v)
}

// PointCross returns the plane cross product (b-a)×(p-a).
//
// The output replay keeps one arithmetic state through nested geometry checks.
// A failed operation must not be mistaken for an ordinary geometric miss.
func (c *Arithmetic) PointCross(a, b, p Point) *big.Rat {
	abu, abv := c.Sub(b.U, a.U), c.Sub(b.V, a.V)
	apu, apv := c.Sub(p.U, a.U), c.Sub(p.V, a.V)
	return c.Sub(c.Mul(abu, apv), c.Mul(abv, apu))
}

// SamePoint reports whether a and b are the same plane point and OK holds.
func (c *Arithmetic) SamePoint(a, b Point) bool {
	same := c.Cmp(a.U, b.U) == 0 && c.Cmp(a.V, b.V) == 0
	return c.OK && same
}

// LinePoint returns the point of line at parameter t.
func (c *Arithmetic) LinePoint(line sectionrecord.LineSeg, t *big.Rat) Point {
	a, okA := PointOf(line.Start)
	b, okB := PointOf(line.End)
	if !okA || !okB || !c.Valid(t) {
		c.OK = false
		return Point{}
	}
	return c.Interpolate(a, b, t)
}

// Interpolate returns a+t(b-a).
func (c *Arithmetic) Interpolate(a, b Point, t *big.Rat) Point {
	return Point{
		U: c.Add(a.U, c.Mul(t, c.Sub(b.U, a.U))),
		V: c.Add(a.V, c.Mul(t, c.Sub(b.V, a.V))),
	}
}

// LineSide returns the signed plane cross product of p against line.
func (c *Arithmetic) LineSide(line sectionrecord.LineSeg, p sectionrecord.Point2) *big.Rat {
	a, okA := PointOf(line.Start)
	b, okB := PointOf(line.End)
	point, okP := PointOf(p)
	if !okA || !okB || !okP {
		c.OK = false
		return new(big.Rat)
	}
	return c.PointCross(a, b, point)
}

// LineParameter returns the parameter in [0, 1] of p on line and whether p
// lies exactly on it.
func (c *Arithmetic) LineParameter(line sectionrecord.LineSeg, p Point) (*big.Rat, bool) {
	a, okA := PointOf(line.Start)
	b, okB := PointOf(line.End)
	if !okA || !okB {
		c.OK = false
		return nil, false
	}
	du, dv := c.Sub(b.U, a.U), c.Sub(b.V, a.V)
	var t *big.Rat
	if du.Sign() != 0 {
		t = c.Quo(c.Sub(p.U, a.U), du)
	} else if dv.Sign() != 0 {
		t = c.Quo(c.Sub(p.V, a.V), dv)
	} else {
		return nil, false
	}
	if c.Cmp(t, big.NewRat(0, 1)) < 0 || c.Cmp(t, big.NewRat(1, 1)) > 0 {
		return nil, false
	}
	q := c.LinePoint(line, t)
	return t, c.SamePoint(p, q)
}
