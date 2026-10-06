package survey2d

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// RatAbsDiff is |r − f| rounded up to float64.
func RatAbsDiff(r *big.Rat, f float64) float64 {
	return proofarith.RationalFloatError(r, f)
}
