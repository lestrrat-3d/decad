// Package rimextension continues a stopped straight bevel along a cap edge.
package rimextension

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// Section describes a held bevel whose cap is normal to Y. Stop and End
// lie on its outer cap edge. Toe lies on the cap and Heel on the side.
type Section struct{ Stop, End, Toe, Heel r3.Vec }

// Groups maps the affected held facets to their owning source groups.
type Groups struct {
	Cap, Side, Boss, Bevel, Extension int
	Stop                              map[int]bool
	// OpenCylinderEnd allows a terminal boundary already closed by the cylinder.
	OpenCylinderEnd bool
}

type point [3]*big.Rat

func scalar(v float64) *big.Rat     { return new(big.Rat).SetFloat64(v) }
func plus(a, b *big.Rat) *big.Rat   { return new(big.Rat).Add(a, b) }
func minus(a, b *big.Rat) *big.Rat  { return new(big.Rat).Sub(a, b) }
func times(a, b *big.Rat) *big.Rat  { return new(big.Rat).Mul(a, b) }
func divide(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }
func lift(v r3.Vec) point           { return point{scalar(v.X), scalar(v.Y), scalar(v.Z)} }
func (p point) key() [3]string {
	return [3]string{p[0].RatString(), p[1].RatString(), p[2].RatString()}
}
func (p point) vec() r3.Vec {
	x, _ := p[0].Float64()
	y, _ := p[1].Float64()
	z, _ := p[2].Float64()
	return r3.NewVec(x, y, z)
}
func interpolate(a, b point, k *big.Rat) point {
	var p point
	for i := range p {
		p[i] = plus(a[i], times(k, minus(b[i], a[i])))
	}
	return p
}
func planarArea(a, b, c point, i, j int) *big.Rat {
	return minus(times(minus(b[i], a[i]), minus(c[j], a[j])), times(minus(c[i], a[i]), minus(b[j], a[j])))
}
func collinear(a, b, c point) bool {
	return planarArea(a, b, c, 0, 1).Sign() == 0 && planarArea(a, b, c, 1, 2).Sign() == 0 && planarArea(a, b, c, 2, 0).Sign() == 0
}

type chart struct {
	start, end      point
	slope, gradient *big.Rat
	heel            *big.Rat
	forward, rise   int
}

// The two existing seam vertices determine the new plane exactly. A
// nominal setback check against the native source belongs to admission.
func newChart(s Section) (chart, bool) {
	for _, v := range [...]r3.Vec{s.Stop, s.End, s.Toe, s.Heel} {
		if !chartCoordinate(v.X) || !chartCoordinate(v.Y) || !chartCoordinate(v.Z) {
			return chart{}, false
		}
	}
	a, b, u, v := lift(s.Stop), lift(s.End), lift(s.Toe), lift(s.Heel)
	if s.Stop.X == s.End.X || s.Stop.Y != s.End.Y || s.Toe.X != s.Stop.X || s.Heel.X != s.Stop.X || s.Toe.Y != s.Stop.Y || s.Heel.Z != s.Stop.Z || s.Heel.Z == s.Toe.Z || s.Heel.Y == s.Toe.Y {
		return chart{}, false
	}
	dx := minus(b[0], a[0])
	dy := minus(v[1], u[1])
	return chart{start: a, end: b, slope: divide(minus(b[2], a[2]), dx), gradient: divide(dy, minus(v[2], u[2])), heel: v[1], forward: dx.Sign(), rise: dy.Sign()}, true
}
func (c chart) height(p point) *big.Rat {
	side := plus(c.start[2], times(c.slope, minus(p[0], c.start[0])))
	return minus(c.heel, times(c.gradient, minus(side, p[2])))
}
func (c chart) distances(p point) [3]*big.Rat {
	q := [3]*big.Rat{minus(p[0], c.start[0]), minus(c.end[0], p[0]), minus(c.height(p), p[1])}
	if c.forward < 0 {
		q[0].Neg(q[0])
		q[1].Neg(q[1])
	}
	if c.rise < 0 {
		q[2].Neg(q[2])
	}
	return q
}

func chartCoordinate(x float64) bool {
	_, e := math.Frexp(x)
	return !math.IsNaN(x) && !math.IsInf(x, 0) && (x == 0 || e >= -96 && e <= 96)
}
