package boundarywalk

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// LineWalkReplay admits natural endpoints and charges all exact line bounds.
func LineWalkReplay(ctx context.Context, seg LineSeg, b *proofbound.ReplayBudget) (survey2d.SegmentWalk, error) {
	if b == nil || (seg.TStart != 0 && seg.TStart != 1) || (seg.TEnd != 0 && seg.TEnd != 1) {
		return survey2d.SegmentWalk{}, fmt.Errorf("%w: bounded line requires natural endpoints", decaderr.ErrUnsupported)
	}
	for _, f := range [4]float64{seg.Start.U, seg.Start.V, seg.End.U, seg.End.V} {
		if proofbound.IsNonFinite(f) {
			return survey2d.SegmentWalk{}, fmt.Errorf("%w: nonfinite bounded line", decaderr.ErrUnsupported)
		}
	}
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	du, dv := u1-u0, v1-v0
	length := math.Hypot(du, dv)
	if proofbound.IsNonFinite(du) || proofbound.IsNonFinite(dv) || proofbound.IsNonFinite(length) {
		return survey2d.SegmentWalk{}, fmt.Errorf("%w: bounded line arithmetic is nonfinite", decaderr.ErrUnsupported)
	}
	op := func(a, c proof.Dyadic, k byte) (proof.Dyadic, error) {
		return proof.ReplayDyadicArithmetic(ctx, a, c, k, b)
	}
	a := [4]proof.Dyadic{proof.DyOfFinite(u0), proof.DyOfFinite(v0), proof.DyOfFinite(u1), proof.DyOfFinite(v1)}
	dx, e := op(a[2], a[0], '-')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	dy, e := op(a[3], a[1], '-')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	x2, e := op(dx, dx, '*')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	y2, e := op(dy, dy, '*')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	sq, e := op(x2, y2, '+')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	sumAbs := func(x, y proof.Dyadic) (float64, error) {
		a, e := proof.ReplayDyadicRat(ctx, x, b)
		if e != nil {
			return 0, e
		}
		c, e := proof.ReplayDyadicRat(ctx, y, b)
		if e != nil {
			return 0, e
		}
		a.Abs(a)
		c.Abs(c)
		q, e := proof.ReplayRatArithmetic(ctx, a, c, '+', b)
		if e != nil {
			return 0, e
		}
		f, e := proof.ReplayFractionFloat64(ctx, q.Num(), q.Denom(), b)
		if e != nil {
			return 0, e
		}
		if math.IsInf(f, 0) {
			return f, nil
		}
		r, e := proof.ReplayFloatRat(ctx, f, b)
		if e != nil {
			return 0, e
		}
		cmp, e := proof.ReplayRatCompare(ctx, r, q, b)
		if e != nil {
			return 0, e
		}
		if cmp != 0 {
			f = math.Nextafter(f, math.Inf(1))
		}
		return f, nil
	}
	c0, e := sumAbs(a[0], a[1])
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	c1, e := sumAbs(a[2], a[3])
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	coord := math.Max(c0, c1)
	tangentErr := func(x proof.Dyadic, held float64) (float64, error) {
		q, e := proof.ReplayDyadicRat(ctx, x, b)
		if e != nil {
			return 0, e
		}
		return proof.ReplayRationalFloatError(ctx, q, held, b)
	}
	eu, e := tangentErr(dx, du)
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	ev, e := tangentErr(dy, dv)
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	tangent := math.Max(eu, ev)
	heldDy := proof.DyOfFinite(length)
	heldSq, e := op(heldDy, heldDy, '*')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	difference, e := op(heldSq, sq, '-')
	if e != nil {
		return survey2d.SegmentWalk{}, e
	}
	var lengthBound, lengthUpper float64
	if difference.IsZero() {
		lengthUpper = length
	} else {
		lengthUpper, e = sumAbs(dx, dy)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		q, e := proof.ReplayDyadicRat(ctx, sq, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		lo, e := proof.ReplaySqrtDownRat(ctx, q, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		hi, e := proof.ReplaySqrtDyadicRat(ctx, q, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		l, e := proof.ReplayFloatRat(ctx, lo, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		h, e := proof.ReplayFloatRat(ctx, hi, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		el, e := proof.ReplayRationalFloatError(ctx, l, length, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		eh, e := proof.ReplayRationalFloatError(ctx, h, length, b)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		lengthBound = math.Min(proofbound.ConservativeValueError(length, lengthUpper), math.Max(el, eh))
	}
	return survey2d.SegmentWalk{StartU: u0, StartV: v0, EndU: u1, EndV: v1, TanInU: du, TanInV: dv, TanOutU: du, TanOutV: dv, TanInBound: tangent, TanOutBound: tangent, Length: length, LengthBound: lengthBound, LengthUpper: lengthUpper, CoordUpper: coord}, nil
}
