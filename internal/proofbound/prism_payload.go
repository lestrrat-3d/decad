package proofbound

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

func VecMaxAbs(v r3.Vec) float64 {
	return max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
}
