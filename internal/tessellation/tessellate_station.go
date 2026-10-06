package tessellation

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// CapWallRadiusOffset is the exact rational a circular wall's cap contour adds
// to the wall's own radius: −insideSign·d, so a counter-clockwise wall (its
// material inside) shrinks and a clockwise one (a hole rim) grows — the same
// sign offsetRadius and ivExactOffsetRadius take. A setback that is not finite
// answers nil, which capOffsetStationBound refuses on.
func CapWallRadiusOffset(w survey2d.SideWalk, d float64) *big.Rat {
	rd := proofarith.FloatRat(d)
	if rd == nil {
		return nil
	}
	return new(big.Rat).Neg(new(big.Rat).Mul(InsideSignOf(w), rd))
}
