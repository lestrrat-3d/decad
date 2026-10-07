package proofbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func RatAdd(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func RatMul(values ...*big.Rat) *big.Rat {
	out := big.NewRat(1, 1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func RatMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func RatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// RatAbsDiff is |r − f| rounded up to float64.
func RatAbsDiff(r *big.Rat, f float64) float64 {
	return proofarith.RationalFloatError(r, f)
}
