package tessellation

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// IntervalAbsUpper is the largest magnitude an enclosure allows.
func IntervalAbsUpper(a proofbound.RatInterval) *big.Rat {
	return survey2d.RatMax(new(big.Rat).Abs(a.Lo), new(big.Rat).Abs(a.Hi))
}
