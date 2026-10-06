package proofbound

import (
	"math/big"
)

// Moved from the root package's clearance_poly.go.

func RatOf(f float64) (*big.Rat, bool) {
	r := new(big.Rat)
	if r.SetFloat64(f) == nil {
		return nil, false
	}
	return r, true
}
