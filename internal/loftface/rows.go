package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// The side of a LoftFaces cap row set against the radial limit.
const (
	SideInside  = -1
	SideCrossed = 0
	SideOutside = 1
)

// ClassifyRows classifies the Bernstein rows q as SideOutside, SideInside or
// SideCrossed, and returns the rows with the sign that makes them positive.
func ClassifyRows(q [3][]*big.Rat) (int, [3][]*big.Rat) {
	if polynomial.BernsteinRowsPositive(q) {
		return SideOutside, q
	}
	negative := NegateRows(q)
	if polynomial.BernsteinRowsPositive(negative) {
		return SideInside, negative
	}
	if polynomial.BernsteinRowsCross(q) {
		return SideCrossed, q
	}
	if polynomial.BernsteinRowsCross(negative) {
		return SideCrossed, negative
	}
	return 2, [3][]*big.Rat{}
}

// NegateRows returns q with every coefficient negated.
func NegateRows(q [3][]*big.Rat) [3][]*big.Rat {
	var out [3][]*big.Rat
	for i, row := range q {
		out[i] = NegateRow(row)
	}
	return out
}

// NegateRow returns row with every coefficient negated.
func NegateRow(row []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(row))
	for i, value := range row {
		out[i] = new(big.Rat).Neg(value)
	}
	return out
}
