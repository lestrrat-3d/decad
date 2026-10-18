// Package loftcontact proves source contacts and moving-triangle hull bounds
// for axial Loft/prism Union.
package loftcontact

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

type point struct{ u, v *big.Rat }
type linear [2]*big.Rat
type quadratic [3]*big.Rat

// LineSeam records one transverse contact's exact source parameters as
// rational functions of height. It does not certify a Boolean result.
type LineSeam struct {
	lineNum, lineDen linear
	carrierNum       quadratic
	carrierDen       linear
}

// CertifyLineSeam proves an interior line-line crossing at every normalized
// axial height. The caller must authenticate the paired full-domain Loft
// lines, their axial frame, and the fixed prism source carrier. A false
// result means this sufficient certificate does not apply.
func CertifyLineSeam(ctx context.Context, budget *proofbound.WorkBudget,
	bottom, top [2]sectionrecord.Point2, carrier [2]sectionrecord.Point2) (LineSeam, bool, error) {
	if err := ctx.Err(); err != nil {
		return LineSeam{}, false, err
	}
	if err := budget.Step(); err != nil {
		return LineSeam{}, false, err
	}
	l0, ok := lift(bottom[0])
	if !ok {
		return LineSeam{}, false, nil
	}
	r0, ok := lift(bottom[1])
	if !ok {
		return LineSeam{}, false, nil
	}
	l1, ok := lift(top[0])
	if !ok {
		return LineSeam{}, false, nil
	}
	r1, ok := lift(top[1])
	if !ok {
		return LineSeam{}, false, nil
	}
	c, ok := lift(carrier[0])
	if !ok {
		return LineSeam{}, false, nil
	}
	d, ok := lift(carrier[1])
	if !ok {
		return LineSeam{}, false, nil
	}
	v := sub(d, c)
	e0 := sub(r0, l0)
	e1 := sub(sub(r1, l1), e0)
	f0 := sub(l0, c)
	f1 := sub(l1, l0)
	a0 := cross(v, sub(l0, c))
	a1 := cross(v, sub(l1, l0))
	b0 := cross(v, e0)
	b1 := cross(v, e1)
	seam := LineSeam{
		lineNum: linear{neg(a0), neg(a1)},
		lineDen: linear{b0, b1},
		carrierNum: quadratic{
			cross(e0, f0),
			add(cross(e0, f1), cross(e1, f0)),
			cross(e1, f1),
		},
		carrierDen: linear{neg(b0), neg(b1)},
	}
	if err := budget.Step(); err != nil {
		return LineSeam{}, false, err
	}
	sign := b0.Sign()
	if sign == 0 || !sameSignAtEnds(seam.lineDen, sign) ||
		!sameSignAtEnds(seam.lineNum, sign) ||
		!sameSignAtEnds(subLinear(seam.lineDen, seam.lineNum), sign) ||
		!sameSignBernstein(seam.carrierNum, -sign) ||
		!sameSignBernstein(subQuadratic(linearQuadratic(seam.carrierDen), seam.carrierNum), -sign) {
		return LineSeam{}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return LineSeam{}, false, err
	}
	return seam, true, nil
}

// At returns the exact line and carrier parameters at height t in [0,1].
func (s LineSeam) At(t *big.Rat) (*big.Rat, *big.Rat, bool) {
	if t == nil || t.Sign() < 0 || t.Cmp(big.NewRat(1, 1)) > 0 || s.lineDen[0] == nil {
		return nil, nil, false
	}
	line := new(big.Rat).Quo(evalLinear(s.lineNum, t), evalLinear(s.lineDen, t))
	carrier := new(big.Rat).Quo(evalQuadratic(s.carrierNum, t), evalLinear(s.carrierDen, t))
	return line, carrier, true
}

func lift(p sectionrecord.Point2) (point, bool) {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return point{u, v}, u != nil && v != nil
}

func sub(a, b point) point {
	return point{new(big.Rat).Sub(a.u, b.u), new(big.Rat).Sub(a.v, b.v)}
}

func cross(a, b point) *big.Rat {
	return new(big.Rat).Sub(new(big.Rat).Mul(a.u, b.v), new(big.Rat).Mul(a.v, b.u))
}

func add(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func neg(a *big.Rat) *big.Rat    { return new(big.Rat).Neg(a) }

func subLinear(a, b linear) linear {
	return linear{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1])}
}

func linearQuadratic(a linear) quadratic {
	return quadratic{a[0], a[1], new(big.Rat)}
}

func subQuadratic(a, b quadratic) quadratic {
	return quadratic{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func evalLinear(a linear, t *big.Rat) *big.Rat {
	return add(a[0], new(big.Rat).Mul(a[1], t))
}

func evalQuadratic(a quadratic, t *big.Rat) *big.Rat {
	return add(add(a[0], new(big.Rat).Mul(a[1], t)),
		new(big.Rat).Mul(a[2], new(big.Rat).Mul(t, t)))
}

func sameSignAtEnds(a linear, sign int) bool {
	return a[0].Sign() == sign && add(a[0], a[1]).Sign() == sign
}

// Strict Bernstein signs suffice to prove one quadratic's whole-interval sign.
func sameSignBernstein(a quadratic, sign int) bool {
	mid := add(a[0], new(big.Rat).Quo(a[1], big.NewRat(2, 1)))
	end := add(add(a[0], a[1]), a[2])
	return a[0].Sign() == sign && mid.Sign() == sign && end.Sign() == sign
}
