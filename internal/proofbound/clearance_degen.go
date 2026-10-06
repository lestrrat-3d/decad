package proofbound

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// FiniteVec guards exact rational lifts and every float result used by a
// certificate.
func FiniteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}
