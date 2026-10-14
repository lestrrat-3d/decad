package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// AxisDot returns the exact rational dot product of point and axis.
func AxisDot(point, axis r3.Vec) *big.Rat {
	term := func(a, b float64) *big.Rat {
		return proofarith.MulRat(new(big.Rat), proofarith.FloatRat(a), proofarith.FloatRat(b))
	}
	sum := proofarith.AddRat(new(big.Rat), term(point.X, axis.X), term(point.Y, axis.Y))
	return proofarith.AddRat(sum, sum, term(point.Z, axis.Z))
}

// CenterAxisInterval encloses the projection of a held center onto axis.
// centerBound is the center's nonnegative coordinate bound and axisLen an upper
// bound on the axis length. It reports false when the center, its bound or the
// resulting interval is not finite, or the bound is negative.
func CenterAxisInterval(center r3.Vec, centerBound float64, axis r3.Vec, axisLen float64) (float64, float64, bool) {
	if !FiniteVec(center) || IsNonFinite(centerBound) || centerBound < 0 {
		return 0, 0, false
	}
	dot := AxisDot(center, axis)
	delta := ProductUpper(centerBound, axisLen)
	lo := math.Nextafter(RatFloatDown(dot)-delta, math.Inf(-1))
	hi := math.Nextafter(RatFloatUp(dot)+delta, math.Inf(1))
	return lo, hi, !IsNonFinite(lo) && !IsNonFinite(hi)
}
