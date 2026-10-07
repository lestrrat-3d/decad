package momentregion

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Accumulate adds one held moment and its proven error to a region sum.
func Accumulate(value, bound *float64, term, termBound float64) {
	next := *value + term
	*bound = proofbound.AbsSumUpper(*bound, termBound, proofarith.AddRoundError(*value, term, next))
	*value = next
}
