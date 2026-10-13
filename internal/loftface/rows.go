package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
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

// A rational cap X map is monotone over this Bezier span if every exact
// derivative-numerator coefficient has one strict sign, or all are zero.
func CapXDirection(mapped Map, span freeform.BezierSpan) int {
	g, h := CapBernsteinRows(mapped, span)
	numerator := polynomial.BernsteinSub(polynomial.BernsteinMul(polynomial.BernsteinDeriv(h[0]), g),
		polynomial.BernsteinMul(h[0], polynomial.BernsteinDeriv(g)))

	negative, positive, zero := true, true, true
	for _, coefficient := range numerator {
		negative = negative && coefficient.Sign() < 0
		positive = positive && coefficient.Sign() > 0
		zero = zero && coefficient.Sign() == 0
	}
	switch {
	case negative:
		return -1
	case zero:
		return 0
	case positive:
		return 1
	default:
		return 2
	}
}

// DirectedCapXRange returns the X interval of cap map mapped over span.
func DirectedCapXRange(mapped Map, span freeform.BezierSpan,
	root RootCrossing, direction int) Interval {
	left, right := BezierPoint(span, root.Lo), BezierPoint(span, root.Hi)
	a, z := mapped.At(left[0], left[1])[0], mapped.At(right[0], right[1])[0]
	if direction < 0 {
		return Interval{Lo: z, Hi: a}
	}
	return Interval{Lo: a, Hi: z}
}
