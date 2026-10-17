package spiralsector

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// HeldXStrip encloses every straight held edge portion in a closed X
// interval. The caller already checked that all path vertices strictly
// increase in X, so exact endpoint interpolation covers the full strip.
func HeldXStrip(held []r3.Vec, xLo, xHi *big.Rat) ([3]loftface.Interval, bool) {
	var box [3]loftface.Interval
	if xLo == nil || xHi == nil || xLo.Cmp(xHi) > 0 {
		return box, false
	}
	seen := false
	for i := 1; i < len(held); i++ {
		a, z := held[i-1], held[i]
		ax, zx := proof.FloatRat(a.X), proof.FloatRat(z.X)
		lo, hi := xLo, xHi
		if lo.Cmp(ax) < 0 {
			lo = ax
		}
		if hi.Cmp(zx) > 0 {
			hi = zx
		}
		if lo.Cmp(hi) > 0 {
			continue
		}
		for _, x := range []*big.Rat{lo, hi} {
			t := shortRatDiv(shortRatSub(x, ax), shortRatSub(zx, ax))
			coordinates := [3]*big.Rat{x,
				shortRatAdd(proof.FloatRat(a.Y), shortRatMul(t, shortRatSub(proof.FloatRat(z.Y), proof.FloatRat(a.Y)))),
				shortRatAdd(proof.FloatRat(a.Z), shortRatMul(t, shortRatSub(proof.FloatRat(z.Z), proof.FloatRat(a.Z))))}
			for axis, coordinate := range coordinates {
				if !seen || coordinate.Cmp(box[axis].Lo) < 0 {
					box[axis].Lo = coordinate
				}
				if !seen || coordinate.Cmp(box[axis].Hi) > 0 {
					box[axis].Hi = coordinate
				}
			}
			seen = true
		}
	}
	return box, seen
}

// RootFlankBoxesBound compares opposite endpoints of the source and held boxes.
func RootFlankBoxesBound(trueBox, heldBox [3]loftface.Interval,
	minBound float64) (float64, bool) {
	squared := new(big.Rat)
	for axis := range trueBox {
		a, b := trueBox[axis], heldBox[axis]
		if a.Lo == nil || a.Hi == nil || b.Lo == nil || b.Hi == nil ||
			a.Lo.Cmp(a.Hi) > 0 || b.Lo.Cmp(b.Hi) > 0 {
			return 0, false
		}
		lo := new(big.Rat).Abs(shortRatSub(a.Lo, b.Hi))
		hi := new(big.Rat).Abs(shortRatSub(a.Hi, b.Lo))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		squared = shortRatAdd(squared, shortRatMul(lo, lo))
	}
	bound := proofbound.RatSqrtUp(squared)
	return bound, !proofbound.IsNonFinite(bound) && bound < minBound
}
