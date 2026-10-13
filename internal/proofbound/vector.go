package proofbound

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func VecMaxAbs(v r3.Vec) float64 {
	return max(proofarith.Abs(v.X), proofarith.Abs(v.Y), proofarith.Abs(v.Z))
}

// FiniteVec guards exact rational lifts and every float result used by a
// certificate.
func FiniteVec(v r3.Vec) bool {
	return finiteFloat(v.X) && finiteFloat(v.Y) && finiteFloat(v.Z)
}

// finiteFloat is !math.IsNaN(x) && !math.IsInf(x, 0) by comparison.
func finiteFloat(x float64) bool {
	return x == x && x <= math.MaxFloat64 && x >= -math.MaxFloat64
}
