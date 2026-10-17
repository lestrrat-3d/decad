package compositesweep

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PrismUnionFilletBand bounds the removed radius band of a prism corner.
func PrismUnionFilletBand(origin r3.Vec, corner [2]float64, xOther, yOther, z0, z1, r float64) (
	AuditBox, bool,
) {
	axis := func(o, c, other float64) (float64, float64) {
		at := new(big.Rat).Add(proofarith.FloatRat(o), proofarith.FloatRat(c))
		inner := new(big.Rat).Set(at)
		if c < other {
			inner.Add(inner, proofarith.FloatRat(r))
		} else {
			inner.Sub(inner, proofarith.FloatRat(r))
		}
		if at.Cmp(inner) < 0 {
			return proofbound.RatFloatDown(at), proofbound.RatFloatUp(inner)
		}
		return proofbound.RatFloatDown(inner), proofbound.RatFloatUp(at)
	}
	x0, x1 := axis(origin.X, corner[0], xOther)
	y0, y1 := axis(origin.Y, corner[1], yOther)
	zA := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(z0))
	zB := new(big.Rat).Add(proofarith.FloatRat(origin.Z), proofarith.FloatRat(z1))
	if zA.Cmp(zB) > 0 {
		zA, zB = zB, zA
	}
	zLo, zHi := proofbound.RatFloatDown(zA), proofbound.RatFloatUp(zB)
	if !proofbound.FiniteVec(r3.Vec{X: x0, Y: y0, Z: zLo}) ||
		!proofbound.FiniteVec(r3.Vec{X: x1, Y: y1, Z: zHi}) {
		return AuditBox{}, false
	}
	return AuditBox{Min: r3.Vec{X: x0, Y: y0, Z: zLo},
		Max: r3.Vec{X: x1, Y: y1, Z: zHi}}, true
}
