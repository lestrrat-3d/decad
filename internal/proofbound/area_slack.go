package proofbound

import (
	"math/big"

	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// AreaSlack returns an upper bound on how far the published area interval
// [area-bound, area+bound] lies from the held interval [heldLo, heldHi]: the
// larger of the two cross-endpoint distances. ok is false when heldLo is nil,
// or area or bound is non-finite, or bound is negative.
func AreaSlack(area, bound float64, heldLo, heldHi *big.Rat) (float64, bool) {
	if heldLo == nil || !FiniteVec(r3.Vec{X: area, Y: bound}) || bound < 0 {
		return 0, false
	}
	lo := new(big.Rat).Sub(proofarith.FloatRat(area), proofarith.FloatRat(bound))
	hi := new(big.Rat).Add(proofarith.FloatRat(area), proofarith.FloatRat(bound))
	d1 := new(big.Rat).Abs(new(big.Rat).Sub(lo, heldHi))
	d2 := new(big.Rat).Abs(new(big.Rat).Sub(hi, heldLo))
	return RatFloatUp(RatMax(d1, d2)), true
}
