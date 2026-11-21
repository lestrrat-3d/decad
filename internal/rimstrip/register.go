// Package rimstrip registers finite cylindrical surface patches.
package rimstrip

import "math/big"

type point [3]*big.Rat

func rat(v float64) *big.Rat     { return new(big.Rat).SetFloat64(v) }
func sub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func add(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func mul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
func quo(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }
func abs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }
func maxRat(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) > 0 {
		return a
	}
	return b
}
func interpolate(a, b point, t *big.Rat) point {
	var p point
	for k := range p {
		p[k] = add(a[k], mul(t, sub(b[k], a[k])))
	}
	return p
}
func cutY(p []point, y *big.Rat, sense int) []point {
	var out []point
	for i, a := range p {
		b := p[(i+1)%len(p)]
		u, v := a[1].Cmp(y), b[1].Cmp(y)
		if u*sense >= 0 {
			out = append(out, a)
		}
		if u*v < 0 {
			out = append(out, interpolate(a, b, quo(sub(y, a[1]), sub(b[1], a[1]))))
		}
	}
	return out
}
