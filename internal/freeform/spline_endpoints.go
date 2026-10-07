package freeform

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FreeformEndControls picks the converted chain's first and last CONTROL points
// in the recorded walk order. It is the single owner of that selection —
// freeformEndpoints rounds the pair it returns and freeformEndpointBounds
// measures that rounding, and the two readings must never disagree about which
// control point an end is.
func FreeformEndControls(spans []BezierSpan, reversed bool) (RatPoint, RatPoint, bool) {
	if len(spans) == 0 || len(spans[0]) == 0 || len(spans[len(spans)-1]) == 0 {
		return RatPoint{}, RatPoint{}, false
	}
	first := spans[0][0]
	last := spans[len(spans)-1][len(spans[len(spans)-1])-1]
	if reversed {
		first, last = last, first
	}
	return first, last, true
}

// EndTangents is a walk's pair of end tangent directions, each with the proven
// error bound it carries on EITHER of its two components — the pair a
// survey2d.SegmentWalk copies into tanIn/tanInBound and tanOut/tanOutBound.
type EndTangents struct {
	InU, InV   float64
	InBound    float64
	OutU, OutV float64
	OutBound   float64
}

// FreeformEndTangents returns the walk's tangent directions at its start and
// end. A Bézier's derivative at an end is degree·(the adjacent control leg), so
// the DIRECTION is an exact fact of the control net — no sampling, and no
// normalization (a survey2d.SegmentWalk tangent is a direction, not a unit vector).
//
// The float64 the walk holds is not that exact fact, though: the leg is formed
// over big.Rat and then rounded once on the way out, and a control point of an
// ordinary rational curve is a ratio no float64 lands on. So each tangent
// STATES its bound — the gap from the exact rational leg to the float that
// stands for it, the wider component of the two — rather than passing the
// rounding off as exactness.
//
// A reversed walk enters where the curve leaves, so both the order and the sign
// of the two legs flip. IEEE negation is exact, so each bound rides along with
// the leg it belongs to.
func FreeformEndTangents(spans []BezierSpan, reversed bool) (EndTangents, error) {
	if len(spans) == 0 {
		return EndTangents{}, fmt.Errorf(`%w: a converted free-form curve holds no span`, decaderr.ErrDegenerate)
	}
	first, last := spans[0], spans[len(spans)-1]
	if len(first) < 2 || len(last) < 2 {
		return EndTangents{}, fmt.Errorf(`%w: a converted free-form span holds fewer than two control points`, decaderr.ErrDegenerate)
	}
	leg := func(from, to RatPoint, degree int) (float64, float64, float64, bool) {
		scale := big.NewRat(int64(degree), 1)
		du := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.U, from.U))
		dv := new(big.Rat).Mul(scale, new(big.Rat).Sub(to.V, from.V))
		u, _ := du.Float64()
		v, _ := dv.Float64()
		if proofbound.IsNonFinite(u) || proofbound.IsNonFinite(v) {
			return 0, 0, 0, false
		}
		bound := math.Max(proofarith.RationalFloatError(du, u), proofarith.RationalFloatError(dv, v))
		return u, v, bound, true
	}
	inU, inV, inBound, okIn := leg(first[0], first[1], len(first)-1)
	outU, outV, outBound, okOut := leg(last[len(last)-2], last[len(last)-1], len(last)-1)
	if !okIn || !okOut {
		return EndTangents{}, fmt.Errorf(`%w: a free-form end tangent is not representable`, decaderr.ErrNotFinite)
	}
	if reversed {
		return EndTangents{
			InU: -outU, InV: -outV, InBound: outBound,
			OutU: -inU, OutV: -inV, OutBound: inBound,
		}, nil
	}
	return EndTangents{
		InU: inU, InV: inV, InBound: inBound,
		OutU: outU, OutV: outV, OutBound: outBound,
	}, nil
}

// FreeformControlExtent is an upper envelope on |u|+|v| over the curve, read
// off the control points. The convex hull property makes it a PROVEN envelope
// for the curve itself, not just for its control net.
func FreeformControlExtent(spans []BezierSpan) float64 {
	extent := 0.0
	for _, span := range spans {
		for _, point := range span {
			u, _ := new(big.Rat).Abs(point.U).Float64()
			v, _ := new(big.Rat).Abs(point.V).Float64()
			if sum := proofbound.AbsSumUpper(u, v); sum > extent {
				extent = sum
			}
		}
	}
	return extent
}
