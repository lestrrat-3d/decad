package proofbound

import (
	"math"
)

func IsNonFinite(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }
