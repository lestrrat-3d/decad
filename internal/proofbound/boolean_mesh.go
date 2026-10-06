package proofbound

import (
	"math"
)

// Moved from the root package's boolean_mesh.go.

func IsNonFinite(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }
