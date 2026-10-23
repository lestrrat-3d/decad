package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// VerticalTraceMapPoint maps a source point into the exact vertical side plane.
func VerticalTraceMapPoint(affine *loftface.VerticalAffine,
	point sectionrecord.Point2) [2]*big.Rat {
	u, v := lfF(point.U), lfF(point.V)
	return [2]*big.Rat{
		lfAdd(affine.Offset[0], lfAdd(lfMul(affine.Column[0][0], u), lfMul(affine.Column[1][0], v))),
		lfAdd(affine.Offset[1], lfAdd(lfMul(affine.Column[0][1], u), lfMul(affine.Column[1][1], v))),
	}
}
