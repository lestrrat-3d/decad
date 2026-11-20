package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LoftWindowXMax bounds the X coordinate of a solid ruled between two native
// sections. The caller supplies the sealed original Loft and its exact X shift.
func LoftWindowXMax(ctx context.Context, p *loftrecord.Payload, shiftX float64, meter *proofbound.ReplayBudget) (*big.Rat, bool, error) {
	if meter == nil || p == nil || p.SourceIdentity == nil || !p.AuthenticatedReconstruction ||
		p.SurfaceResult || p.ThroughBoreCut || p.Xform != r3.Identity() || p.JoinDelta != 0 || p.JoinPlaneGap != 0 {
		return nil, false, nil
	}
	if err := meter.ReserveBytes(ctx, 8192); err != nil {
		return nil, false, err
	}
	delta, shift := scalar(p.ConstructionDelta), scalar(shiftX)
	if delta == nil || delta.Sign() < 0 || shift == nil {
		return nil, false, nil
	}
	var upper *big.Rat
	for i, profile := range [2]momentinput.Profile{p.Profile0, p.Profile1} {
		frame := [2]r3.Frame{p.Frame0, p.Frame1}[i]
		if frame.U() != (r3.Vec{X: 1}) || frame.V() != (r3.Vec{Z: 1}) || len(profile.Holes) > 0 || len(profile.Outer.Segments) == 0 {
			return nil, false, nil
		}
		origin := scalar(frame.Origin().X)
		if origin == nil {
			return nil, false, nil
		}
		for _, seg := range profile.Outer.Segments {
			if err := meter.SpendWork(ctx, 64); err != nil {
				return nil, false, err
			}
			if err := meter.ReserveBytes(ctx, 65536); err != nil {
				return nil, false, err
			}
			u := windowLoftSegmentUpper(seg)
			if u == nil {
				return nil, false, nil
			}
			x := plus(origin, u)
			if upper == nil || upper.Cmp(x) < 0 {
				upper = x
			}
		}
	}
	return plus(plus(upper, shift), delta), true, nil
}

func windowSegmentUpper(seg any) *big.Rat {
	switch s := seg.(type) {
	case momentinput.LineSeg:
		if !(s.TStart >= 0 && s.TStart <= 1 && s.TEnd >= 0 && s.TEnd <= 1) {
			return nil
		}
		a, b := scalar(s.Start.U), scalar(s.End.U)
		if a == nil || b == nil {
			return nil
		}
		if a.Cmp(b) > 0 {
			return a
		}
		return b
	case momentinput.ArcSeg:
		x, y := scalar(s.Center.U), scalar(s.Center.V)
		if x == nil || y == nil {
			return nil
		}
		var r2 big.Rat
		for _, end := range [2]momentinput.Point2{s.Start, s.End} {
			u, v := scalar(end.U), scalar(end.V)
			if u == nil || v == nil {
				return nil
			}
			a, b := minus(u, x), minus(v, y)
			q := plus(times(a, a), times(b, b))
			if r2.Cmp(q) < 0 {
				r2.Set(q)
			}
		}
		r := scalar(proofbound.RatSqrtUp(&r2))
		if r == nil {
			return nil
		}
		return plus(x, r)
	}
	return nil
}
