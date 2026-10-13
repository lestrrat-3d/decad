package decad

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/stretchr/testify/require"
)

// legacyLfGateDerivRange is the four-quotient form of lfGateDerivRange.
func legacyLfGateDerivRange(corners [4]*lfGateNode, row, axis int) loftFaceInterval {
	var nums, dens [4]*big.Rat
	for i, c := range corners {
		nums[i], dens[i] = c.deriv[row][axis], c.g
	}
	num, den := lfGateCornerRange(nums), lfGateCornerRange(dens)
	lo2, hi2 := lfMul(den.lo, den.lo), lfMul(den.hi, den.hi)
	return lfGateCornerRange([4]*big.Rat{lfDiv(num.lo, lo2), lfDiv(num.lo, hi2),
		lfDiv(num.hi, lo2), lfDiv(num.hi, hi2)})
}

func TestLfGateDerivRangeTwoQuotients(t *testing.T) {
	rng := rand.New(rand.NewSource(12))
	for iter := range 2000 {
		var nodes [4]lfGateNode
		var corners [4]*lfGateNode
		sharedG := big.NewRat(rng.Int63n(50)+1, rng.Int63n(50)+1)
		for i := range nodes {
			g := big.NewRat(rng.Int63n(50)+1, rng.Int63n(50)+1)
			if iter%7 == 0 {
				g = sharedG
			}
			nodes[i].g = g
			for r := range nodes[i].deriv {
				for ax := range nodes[i].deriv[r] {
					d := big.NewRat(rng.Int63n(41)-20, rng.Int63n(9)+1)
					if iter%5 == 0 {
						d = new(big.Rat)
					}
					nodes[i].deriv[r][ax] = d
				}
			}
			corners[i] = &nodes[i]
		}
		for r := range 3 {
			for ax := range 2 {
				want := legacyLfGateDerivRange(corners, r, ax)
				got := lfGateDerivRange(corners, r, ax)
				require.Zero(t, want.lo.Cmp(got.lo), "iter %d lo", iter)
				require.Zero(t, want.hi.Cmp(got.hi), "iter %d hi", iter)
			}
		}
	}
}
