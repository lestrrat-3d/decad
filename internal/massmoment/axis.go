package massmoment

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// AxisMoments returns a reader of ∫z^a·ρ^b dA, a + b ≤ 3, over the
// section, for the supplied exact axis coordinates:
// z = dU·(u − aU) + dV·(v − aV) and ρ = dU·(v − aV) − dV·(u − aU). Each is an
// affine form in (u, v) with exact rational coefficients, so z^a·ρ^b expands
// into a polynomial whose coefficients weight the plane-origin moments.
func AxisMoments(m [4][4]proofbound.RatInterval, aUFloat, aVFloat, dUFloat, dVFloat float64) func(a, b int) proofbound.RatInterval {
	aU, aV := proofarith.FloatRat(aUFloat), proofarith.FloatRat(aVFloat)
	dU, dV := proofarith.FloatRat(dUFloat), proofarith.FloatRat(dVFloat)
	zForm := [3]*big.Rat{dU, dV, new(big.Rat).Neg(proofbound.RatAdd(proofbound.RatMul(dU, aU), proofbound.RatMul(dV, aV)))}
	rhoForm := [3]*big.Rat{new(big.Rat).Neg(dV), dU, new(big.Rat).Sub(proofbound.RatMul(dV, aU), proofbound.RatMul(dU, aV))}
	return func(a, b int) proofbound.RatInterval {
		var poly [4][4]*big.Rat
		poly[0][0] = big.NewRat(1, 1)
		for k := range a + b {
			form := zForm
			if k >= a {
				form = rhoForm
			}
			var next [4][4]*big.Rat
			add := func(i, j int, value *big.Rat) {
				if next[i][j] == nil {
					next[i][j] = new(big.Rat)
				}
				next[i][j].Add(next[i][j], value)
			}
			for i := range 4 {
				for j := range 4 - i {
					c := poly[i][j]
					if c == nil {
						continue
					}
					add(i+1, j, proofbound.RatMul(form[0], c))
					add(i, j+1, proofbound.RatMul(form[1], c))
					add(i, j, proofbound.RatMul(form[2], c))
				}
			}
			poly = next
		}
		sum := proofbound.PointInterval(new(big.Rat))
		for i := range 4 {
			for j := range 4 - i {
				if poly[i][j] != nil && poly[i][j].Sign() != 0 {
					sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(m[i][j], poly[i][j]))
				}
			}
		}
		return sum
	}
}
