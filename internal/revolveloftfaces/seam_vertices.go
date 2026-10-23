package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ArcSeamVertex holds one shared source-mapped point and independent bounds
// for the finite Revolve side and the mapped LoftFaces side.
type ArcSeamVertex struct {
	Station    int
	Exact      loftface.RatVec
	Point      r3.Vec
	BlankBound float64
	LoftBound  float64
}

func seamZero() *big.Rat { return new(big.Rat) }

func seamDiv(a, b *big.Rat) *big.Rat {
	return proofarith.QuoRat(new(big.Rat), a, b)
}
func seamAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }

// HeldPoint rounds an exact source-mapped point once and charges its entire
// Euclidean movement by the rational L1 upper bound.
func HeldPoint(exact loftface.RatVec) (r3.Vec, float64, bool) {
	var held r3.Vec
	roundDelta := seamZero()
	for axis, coordinate := range exact {
		if coordinate == nil {
			return r3.Vec{}, 0, false
		}
		value, _ := coordinate.Float64()
		if proofbound.IsNonFinite(value) {
			return r3.Vec{}, 0, false
		}
		roundDelta = lfAdd(roundDelta, seamAbs(lfSub(lfF(value), coordinate)))
		switch axis {
		case 0:
			held.X = value
		case 1:
			held.Y = value
		case 2:
			held.Z = value
		}
	}
	bound := proofbound.RatFloatUp(roundDelta)
	return held, bound, !proofbound.IsNonFinite(bound)
}
