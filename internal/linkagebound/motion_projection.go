package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// MotionProjection bounds a moving body's gap from a static partner over an
// interval. The readers supply the mover at each endpoint and the static
// partner, first as box corners and then as hull points when available.
func MotionProjection(frame motionbound.MotionFrame, rho float64, a, b motionbound.MotionParam,
	mover func(end int, hull bool) (Bounds, bool), partner func(hull bool) (Bounds, bool)) *big.Rat {
	span := a.SpanUpper(b)
	step, ok := motionStep(a, b)
	if !ok {
		return nil
	}
	rem := motionRemainder(frame, rho, span)
	if rem == nil && frame.Kind != motionbound.MotionPrismatic {
		return nil
	}
	var best *big.Rat
	for _, hull := range []bool{false, true} {
		stationary, ok := partner(hull)
		if !ok {
			continue
		}
		for end := range 2 {
			mine, ok := mover(end, hull)
			if !ok {
				continue
			}
			signed := step
			if end == 1 {
				signed = proofbound.IntervalNeg(step)
			}
			side := Side{Corners: mine, H: []*big.Rat{span}, Seg: []proofbound.RatInterval{signed}, Rem: rem}
			var lower *big.Rat
			if hull {
				lower = LowerHull(side, Side{Corners: stationary})
			} else {
				lower = Lower(side, Side{Corners: stationary})
			}
			if lower != nil && (best == nil || lower.Cmp(best) > 0) {
				best = lower
			}
		}
	}
	return best
}

// motionStep is the parameter's signed change from a to b, 2π·Δturn + Δbase
// with π over its enclosure, widened to the floats around it.
func motionStep(a, b motionbound.MotionParam) (proofbound.RatInterval, bool) {
	turn := new(big.Rat).Sub(b.Turn, a.Turn)
	base := new(big.Rat).Sub(b.Base, a.Base)
	return RoundOut(proofbound.IntervalAdd(proofbound.IntervalScale(proofbound.TwoPiInterval(), turn),
		proofbound.PointInterval(base)))
}

// motionRemainder is ½·B·h², where B bounds the second derivative of every
// moved point. It is nil when rho is not finite or the path is prismatic.
func motionRemainder(frame motionbound.MotionFrame, rho float64, h *big.Rat) *big.Rat {
	if frame.Kind == motionbound.MotionPrismatic {
		return nil
	}
	w := proofarith.FloatRat(rho)
	if w == nil {
		return nil
	}
	if frame.Kind == motionbound.MotionBetween {
		theta := motionbound.ParamUpper(frame.Theta)
		w.Mul(w, theta.Mul(theta, theta))
	}
	rem := new(big.Rat).Mul(h, h)
	rem.Mul(rem, w)
	return rem.Quo(rem, big.NewRat(2, 1))
}

// MotionMoverPoints poses one rest reading and encloses each point's velocity
// per unit of the motion parameter.
func MotionMoverPoints(frame motionbound.MotionFrame, param motionbound.MotionParam, points Reading) (Bounds, bool) {
	ideal := frame.At(param)
	var unit motionbound.IvVec
	for d := range 3 {
		unit[d] = proofbound.IntervalScale(frame.Unit, frame.Axis[d])
	}
	centre := motionbound.PointVec(frame.Center)
	for c := range points.Pos {
		x := ApplyIdeal(ideal, points.Pos[c])
		points.Pos[c] = x
		var v motionbound.IvVec
		switch frame.Kind {
		case motionbound.MotionPrismatic:
			v = unit
		case motionbound.MotionRevolute:
			v = IvCross(unit, motionbound.IvVecSub(x, centre))
		default:
			theta := proofbound.IntervalOwned(motionbound.ParamLower(frame.Theta), motionbound.ParamUpper(frame.Theta))
			turn := IvCross(unit, motionbound.IvVecSub(x, centre))
			for d := range 3 {
				v[d] = proofbound.IntervalAdd(proofbound.IntervalMul(turn[d], theta),
					proofbound.IntervalScale(unit[d], frame.Slide))
			}
		}
		points.Vel[c] = []motionbound.IvVec{v}
	}
	return RoundCorners(points)
}
