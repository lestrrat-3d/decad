package proofbound

import (
	"math/big"
)

func RatOf(f float64) (*big.Rat, bool) {
	r := new(big.Rat)
	if r.SetFloat64(f) == nil {
		return nil, false
	}
	return r, true
}
