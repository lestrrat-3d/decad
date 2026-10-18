package tessellation

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChainSweepVertexGap bounds a recorded position's displacement from its nominal station.
func ChainSweepVertexGap(position r3.Vec, bound float64, ideal r3.Vec) float64 {
	if !proofbound.FiniteVec(ideal) {
		return math.Inf(1)
	}
	return ChainSweepVertexGapRat(position, bound, proofarith.FloatRat(ideal.X), proofarith.FloatRat(ideal.Y),
		proofarith.FloatRat(ideal.Z))
}

// ChainSweepVertexGapRat bounds a recorded position against exact station coordinates.
func ChainSweepVertexGapRat(position r3.Vec, bound float64, x, y, z *big.Rat) float64 {
	if !proofbound.FiniteVec(position) {
		return math.Inf(1)
	}
	dx := proofarith.RationalFloatError(x, position.X)
	dy := proofarith.RationalFloatError(y, position.Y)
	dz := proofarith.RationalFloatError(z, position.Z)
	return proofbound.AbsSumUpper(bound, dx, dy, dz)
}

// ChainSweepTriangleArea brackets a chorded facet's area with exact dyadic arithmetic.
func ChainSweepTriangleArea(vertices []r3.Vec, tri [3]int) (float64, float64, bool) {
	a, b, c := proofarith.DyVec(vertices[tri[0]]), proofarith.DyVec(vertices[tri[1]]),
		proofarith.DyVec(vertices[tri[2]])
	n := proofarith.DvCross(proofarith.DvSub(b, a), proofarith.DvSub(c, a))
	squared := proofarith.DyShift(proofarith.DvDot(n, n), -2)
	lo, hi := proofarith.DySqrtDown(squared), proofarith.DySqrtUp(squared)
	return lo, hi, lo > 0 && !proofbound.IsNonFinite(hi)
}

// ChainSweepFaceAreaSlack compares the facet sum with the source face's area interval.
func ChainSweepFaceAreaSlack(area, areaBound float64, held proofbound.RatInterval) (float64, error) {
	slack, ok := proofbound.AreaSlack(area, areaBound, held.Lo, held.Hi)
	if !ok {
		return 0, fmt.Errorf(`%w: a chain sweep face has no certified area interval`, decaderr.ErrUnsupported)
	}
	return slack, nil
}
