package boundarywalk

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// CircleWalkReplay reconstructs an ordinary natural full-circle walk
// with width-metered exact endpoint and length bounds.
func CircleWalkReplay(ctx context.Context, seg sectionrecord.CircleSeg, b *proofbound.ReplayBudget) (survey2d.SegmentWalk, error) {
	if b == nil {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 256); err != nil {
		return survey2d.SegmentWalk{}, err
	}
	if !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || seg.Radius.Unit() != units.Millimeter || proofbound.IsNonFinite(seg.Center.U) || proofbound.IsNonFinite(seg.Center.V) {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	r, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) || proofbound.IsNonFinite(r) {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	w := circularWalk(seg.Center.U, seg.Center.V, r, 0, 2*math.Pi, math.Abs(r), proofbound.CircularSweepUpper(0, 1))
	w.Closed = true
	if proofbound.IsNonFinite(w.StartU) || proofbound.IsNonFinite(w.StartV) || proofbound.IsNonFinite(w.EndU) || proofbound.IsNonFinite(w.EndV) || proofbound.IsNonFinite(w.Length) {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	rr, err := proofarith.ReplayFloatRat(ctx, r, b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	cu, err := proofarith.ReplayFloatRat(ctx, seg.Center.U, b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	cv, err := proofarith.ReplayFloatRat(ctx, seg.Center.V, b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	exactU, err := proofarith.ReplayRatArithmetic(ctx, cu, rr, '+', b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	bound := func(u, v float64) (proofbound.WalkEndBound, error) {
		x, e := proofarith.ReplayRationalFloatError(ctx, exactU, u, b)
		if e != nil {
			return proofbound.WalkEndBound{}, e
		}
		y, e := proofarith.ReplayRationalFloatError(ctx, cv, v, b)
		return proofbound.WalkEndBound{U: x, V: y}, e
	}
	w.StartBound, err = bound(w.StartU, w.StartV)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	w.EndBound, err = bound(w.EndU, w.EndV)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	iv := proofbound.TwoPiIv
	lo, err := proofarith.ReplayRatArithmetic(ctx, iv.Lo, rr, '*', b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	hi, err := proofarith.ReplayRatArithmetic(ctx, iv.Hi, rr, '*', b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	el, err := proofarith.ReplayRationalFloatError(ctx, lo, w.Length, b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	eh, err := proofarith.ReplayRationalFloatError(ctx, hi, w.Length, b)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	w.LengthBound = math.Min(w.LengthBound, math.Max(el, eh))
	return w, nil
}
