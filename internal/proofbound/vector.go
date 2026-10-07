package proofbound

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

func VecMaxAbs(v r3.Vec) float64 {
	return max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
}

// FiniteVec guards exact rational lifts and every float result used by a
// certificate.
func FiniteVec(v r3.Vec) bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}
