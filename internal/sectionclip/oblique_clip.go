package sectionclip

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// SectionRatPoint is an exact rational point or vector in plane-local
// coordinates.
type SectionRatPoint struct {
	U, V *big.Rat
}

// SectionRatAdd returns a+b as a new value.
func SectionRatAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }

// SectionRatSub returns a-b as a new value.
func SectionRatSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

// SectionRatMul returns a*b as a new value.
func SectionRatMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }

// SectionRatQuo returns a/b as a new value.
func SectionRatQuo(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }

// SectionRatCross returns the 2D cross product a.U*b.V-a.V*b.U.
func SectionRatCross(a, b SectionRatPoint) *big.Rat {
	return SectionRatSub(SectionRatMul(a.U, b.V), SectionRatMul(a.V, b.U))
}

// SectionRatVector returns b-a.
func SectionRatVector(a, b SectionRatPoint) SectionRatPoint {
	return SectionRatPoint{SectionRatSub(b.U, a.U), SectionRatSub(b.V, a.V)}
}

// SectionMatchedConvex reports whether the rounded loop has the same vertex
// count as the exact source polygon and every vertex triple of the source and
// of the rounded loop, and their mixed term, is strictly counter-clockwise.
func SectionMatchedConvex(source []SectionRatPoint, rounded []planePoint) bool {
	if len(source) != len(rounded) {
		return false
	}
	proxy := make([]SectionRatPoint, len(rounded))
	for i, point := range rounded {
		proxy[i] = SectionRatPoint{proofarith.FloatRat(point.U), proofarith.FloatRat(point.V)}
	}
	for i := range source {
		next := (i + 1) % len(source)
		a0, a1 := SectionRatVector(source[i], source[next]), SectionRatVector(proxy[i], proxy[next])
		for j := range source {
			if j == i || j == next {
				continue
			}
			b0, b1 := SectionRatVector(source[i], source[j]), SectionRatVector(proxy[i], proxy[j])
			mixed := SectionRatAdd(SectionRatCross(a0, b1), SectionRatCross(a1, b0))
			if SectionRatCross(a0, b0).Sign() <= 0 || SectionRatCross(a1, b1).Sign() <= 0 || mixed.Sign() <= 0 {
				return false
			}
		}
	}
	return true
}
