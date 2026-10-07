package planar

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func dyMax(a, b proof.Dyadic) proof.Dyadic {
	if proof.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b proof.Dyadic) proof.Dyadic {
	if proof.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
