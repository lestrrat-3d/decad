// Package spiralsector checks the spiral first-join and inverse-copy gates.
package spiralsector

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func ratF(v float64) *big.Rat { return proof.FloatRat(v) }

func ratAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

func ratMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

func ratAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }

func rationalDirection(sumY, sumZ float64) (*big.Rat, *big.Rat, bool) {
	flip := sumY < -0.5*math.Hypot(sumY, sumZ)
	if flip {
		sumY, sumZ = -sumY, -sumZ
	}
	den := math.Hypot(sumY, sumZ) + sumY
	if den <= 0 || math.IsNaN(den) || math.IsInf(den, 0) {
		return nil, nil, false
	}
	t := sumZ / den
	if math.IsNaN(t) || math.IsInf(t, 0) || math.Abs(t) > 1e6 {
		return nil, nil, false
	}
	n := int64(math.Round(t * 1e6))
	d := int64(1e6)
	nn, dd := new(big.Int).Mul(big.NewInt(n), big.NewInt(n)),
		new(big.Int).Mul(big.NewInt(d), big.NewInt(d))
	denom := new(big.Int).Add(nn, dd)
	unitY := new(big.Rat).SetFrac(new(big.Int).Sub(dd, nn), denom)
	unitZ := new(big.Rat).SetFrac(new(big.Int).Mul(big.NewInt(2*n), big.NewInt(d)), denom)
	if flip {
		unitY.Neg(unitY)
		unitZ.Neg(unitZ)
	}
	return unitY, unitZ, true
}

// The rounded angle only selects an exact rational support direction. It
// never enters the final bound; the exact unit vector bounds each triangle.
func triangleDirection(v [3]r3.Vec) (*big.Rat, *big.Rat, bool) {
	base := math.Atan2(v[0].Z, v[0].Y)
	lo, hi := base, base
	for _, p := range v[1:] {
		a := math.Atan2(p.Z, p.Y)
		for a-base > math.Pi {
			a -= 2 * math.Pi
		}
		for a-base < -math.Pi {
			a += 2 * math.Pi
		}
		lo, hi = min(lo, a), max(hi, a)
	}
	if hi-lo >= math.Pi {
		return nil, nil, false
	}
	mid := (lo + hi) / 2
	return rationalDirection(math.Cos(mid), math.Sin(mid))
}

// TriangleMeridianError bounds, for the meridian line y = m*x + b, how far the
// triangle's vertices lie below the lower support radius or above the upper
// vertex radius. It reports false when no finite exact answer exists.
func TriangleMeridianError(points [3]r3.Vec, m, b *big.Rat) (*big.Rat, bool) {
	unitY, unitZ, ok := triangleDirection(points)
	if !ok {
		return nil, false
	}
	limit := new(big.Rat)
	for _, v := range points {
		meridian := ratAdd(ratMul(m, ratF(v.X)), b)
		lowerRadius := ratAdd(ratMul(unitY, ratF(v.Y)), ratMul(unitZ, ratF(v.Z)))
		below := ratSub(meridian, lowerRadius)
		if below.Cmp(limit) > 0 {
			limit = below
		}
		r2 := ratAdd(ratMul(ratF(v.Y), ratF(v.Y)), ratMul(ratF(v.Z), ratF(v.Z)))
		radialUp := proofbound.RatSqrtUp(r2)
		if proofbound.IsNonFinite(radialUp) {
			return nil, false
		}
		above := ratSub(ratF(radialUp), meridian)
		if above.Cmp(limit) > 0 {
			limit = above
		}
	}
	return limit, true
}
