package revolveloftfaces

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func lfF(v float64) *big.Rat       { return proofarith.FloatRat(v) }
func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }
func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }
func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }
