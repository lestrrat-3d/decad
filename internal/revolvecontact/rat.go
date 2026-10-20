package revolvecontact

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func lfZero() *big.Rat { return new(big.Rat) }

func lfOne() *big.Rat { return big.NewRat(1, 1) }

func lfF(v float64) *big.Rat { return proofarith.FloatRat(v) }

func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }

func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }

func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }

func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }

func lfAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }
