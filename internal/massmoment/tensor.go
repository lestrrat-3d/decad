package massmoment

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// RotateTensor forms M T Mᵀ entry by entry over exact rational
// coefficients, so every tensor inside the local box maps inside the result.
func RotateTensor(m [3][3]*big.Rat, local [3][3]proofbound.RatInterval) [3][3]proofbound.RatInterval {
	var out [3][3]proofbound.RatInterval
	for i := range out {
		for j := range out[i] {
			sum := proofbound.PointInterval(new(big.Rat))
			for k := range local {
				for l := range local[k] {
					coefficient := new(big.Rat).Mul(m[i][k], m[j][l])
					sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(local[k][l], coefficient))
				}
			}
			out[i][j] = sum
		}
	}
	return out
}

// OrthonormalityDefect is the entrywise absolute sum of MᵀM - I, an upper
// bound on its Frobenius norm that needs no square root.
func OrthonormalityDefect(m [3][3]*big.Rat) *big.Rat {
	defect := new(big.Rat)
	for i := range m {
		for j := range m {
			dot := new(big.Rat)
			for k := range m {
				dot.Add(dot, new(big.Rat).Mul(m[k][i], m[k][j]))
			}
			if i == j {
				dot.Sub(dot, big.NewRat(1, 1))
			}
			defect.Add(defect, dot.Abs(dot))
		}
	}
	return defect
}

// TensorMagnitude is the largest magnitude any entry of the box allows.
func TensorMagnitude(t [3][3]proofbound.RatInterval) *big.Rat {
	largest := new(big.Rat)
	for i := range t {
		for j := range t[i] {
			largest = proofbound.RatMax(largest, proofbound.IntervalAbsUpper(t[i][j]))
		}
	}
	return largest
}

// GershgorinLower is a lower bound on the smallest eigenvalue of every
// symmetric tensor inside the box: each diagonal lower end minus the largest
// magnitudes its row's off-diagonal entries allow, minimized over rows.
func GershgorinLower(t [3][3]proofbound.RatInterval) *big.Rat {
	var lower *big.Rat
	for i := range t {
		row := new(big.Rat).Set(t[i][i].Lo)
		for j := range t[i] {
			if i != j {
				row.Sub(row, proofbound.IntervalAbsUpper(t[i][j]))
			}
		}
		if lower == nil || row.Cmp(lower) < 0 {
			lower = row
		}
	}
	return lower
}

// PositiveDefinite proves every symmetric matrix in the interval
// tensor positive definite by Sylvester's criterion: each leading principal
// minor, evaluated in interval arithmetic, has a strictly positive lower end.
// The minors are polynomials in the entries, so their interval evaluation
// encloses the minor of every member; a positive lower end then holds for
// every member at once.
func PositiveDefinite(m [3][3]proofbound.RatInterval) bool {
	if m[0][0].Lo.Sign() <= 0 {
		return false
	}
	minor2 := proofbound.IntervalSub(proofbound.IntervalMul(m[0][0], m[1][1]), proofbound.IntervalMul(m[0][1], m[1][0]))
	if minor2.Lo.Sign() <= 0 {
		return false
	}
	cofactor := func(a, b, c, d proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a, b), proofbound.IntervalMul(c, d))
	}
	det := proofbound.IntervalAdd(
		proofbound.IntervalSub(
			proofbound.IntervalMul(m[0][0], cofactor(m[1][1], m[2][2], m[1][2], m[2][1])),
			proofbound.IntervalMul(m[0][1], cofactor(m[1][0], m[2][2], m[1][2], m[2][0])),
		),
		proofbound.IntervalMul(m[0][2], cofactor(m[1][0], m[2][1], m[1][1], m[2][0])),
	)
	return det.Lo.Sign() > 0
}
