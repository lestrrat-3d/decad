package revolveboretess

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// onHeldCap proves that every held vertex lies in the exact plane denoted
// by the mapped tool frame and the given cap level.
func onHeldCap(tool prismrecord.Payload, level float64, points []r3.Vec) bool {
	bound, _, ok := capProjectionBound(tool, level, points)
	return ok && bound == 0
}

// capProjectionBound bounds the displacement of an exact projection onto the
// denoted cap plane along its largest normal component.
func capProjectionBound(tool prismrecord.Payload, level float64, points []r3.Vec) (float64, int, bool) {
	if proofbound.IsNonFinite(level) || !proofbound.FiniteVec(tool.Frame.Origin()) ||
		!proofbound.FiniteVec(tool.Frame.U()) || !proofbound.FiniteVec(tool.Frame.V()) {
		return 0, 0, false
	}
	u, v := proofarith.DyVec(tool.Frame.U()), proofarith.DyVec(tool.Frame.V())
	n := proofarith.DvCross(u, v)
	local := proofarith.DvAdd(proofarith.DyVec(tool.Frame.Origin()), proofarith.DvScale(n, proofarith.MustDyOf(level)))
	anchor := proofarith.DvTransform(tool.Xform, local)
	translation := proofarith.DyVec(tool.Xform.Translation())
	worldU := proofarith.DvSub(proofarith.DvTransform(tool.Xform, u), translation)
	worldV := proofarith.DvSub(proofarith.DvTransform(tool.Xform, v), translation)
	normal := proofarith.DvCross(worldU, worldV)
	if proofarith.DvIsZero(normal) {
		return 0, 0, false
	}
	axis := 0
	for i := 1; i < 3; i++ {
		if proofarith.DyCmp(proofarith.DyAbs(normal[i]), proofarith.DyAbs(normal[axis])) > 0 {
			axis = i
		}
	}
	normalAxis := normal[axis].Rat()
	maxGap := new(big.Rat)
	for _, point := range points {
		if !proofbound.FiniteVec(point) {
			return 0, 0, false
		}
		residual := proofarith.DvDot(normal, proofarith.DvSub(proofarith.DyVec(point), anchor)).Rat()
		gap := new(big.Rat).Abs(new(big.Rat).Quo(residual, normalAxis))
		if gap.Cmp(maxGap) > 0 {
			maxGap.Set(gap)
		}
	}
	bound := proofbound.RatFloatUp(maxGap)
	return bound, axis, !math.IsInf(bound, 0) && !math.IsNaN(bound)
}
