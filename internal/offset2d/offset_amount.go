package offset2d

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// OffsetAmount is s·t* over every denoted thickness t* within tDelta of the
// held t: the signed offset the float build spells s*t.
func OffsetAmount(s, t, tDelta float64) (proofbound.RatInterval, error) {
	rt, rd := proofarith.FloatRat(t), proofarith.FloatRat(tDelta)
	if rt == nil || rd == nil || rd.Sign() < 0 {
		return proofbound.RatInterval{}, ErrUnbounded
	}
	amount := proofbound.Interval(new(big.Rat).Sub(rt, rd), new(big.Rat).Add(rt, rd))
	if s < 0 {
		amount = proofbound.IntervalNeg(amount)
	}
	return amount, nil
}
