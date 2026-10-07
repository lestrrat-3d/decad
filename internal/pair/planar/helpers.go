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

func dvNeg(v proof.DyV3) proof.DyV3 {
	return proof.DyV3{proof.DyNeg(v[0]), proof.DyNeg(v[1]), proof.DyNeg(v[2])}
}
