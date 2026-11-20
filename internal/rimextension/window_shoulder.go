package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ShoulderLoftStop reads the authenticated original Loft in its chart and
// locates the cap toe and side heel on its extremal section.
func ShoulderLoftStop(ctx context.Context, p *loftrecord.Payload, shift r3.Vec, wing int, capY, sideZ, lowZ, highZ *big.Rat, meter *proofbound.ReplayBudget) (station, toe, heel *big.Rat, ok bool, err error) {
	chart, ok, err := shoulderLoft(ctx, p, wing, meter)
	if !ok || err != nil {
		return nil, nil, nil, false, err
	}
	q, ok, err := loftWindowSeam(ctx, &chart, ShoulderPoint(shift, wing), capY, sideZ, lowZ, highZ, 1, meter)
	return q.x, q.toeZ, q.heelY, ok, err
}

// ShoulderRoundBeyondEnd certifies that the projected round connecting
// the two side lines stays beyond the native selected endpoint in the
// signed longitudinal coordinate. Its CCW sweep is minor and both
// endpoint rays point to the negative U half-plane; V decreases there.
// Equal squared radii bind the actual circular endpoints exactly.
func ShoulderRoundBeyondEnd(arc momentinput.ArcSeg, endpoint momentinput.Point2, wing int) bool {
	if (wing != 1 && wing != -1) || !wholeShoulderWalk(arc.TStart, arc.TEnd) {
		return false
	}
	x, y := scalar(arc.Center.U), scalar(arc.Center.V)
	ax, ay, bx, by := scalar(arc.Start.U), scalar(arc.Start.V), scalar(arc.End.U), scalar(arc.End.V)
	if x == nil || y == nil || ax == nil || ay == nil || bx == nil || by == nil {
		return false
	}
	ax, ay, bx, by = minus(ax, x), minus(ay, y), minus(bx, x), minus(by, y)
	if ax.Sign() >= 0 || bx.Sign() >= 0 || minus(times(ax, by), times(ay, bx)).Sign() <= 0 {
		return false
	}
	if plus(times(ax, ax), times(ay, ay)).Cmp(plus(times(bx, bx), times(by, by))) != 0 {
		return false
	}
	if wing > 0 {
		return endpoint == arc.Start
	}
	return endpoint == arc.End
}

// A source disk can enter the window after the stop. The early part of
// the chamfer is then a plane-only strip. The switch station lies strictly
// after the disk's pole, while the innermost toe remains outside the disk.
// Disk ordinates have a finite slope on the remainder. The terminal width
// is positive when the actual moved disk misses the side's native endpoint.
type shoulderDiskRange struct{ switchX, minOrdinate, terminalGapUpper, crossing *big.Rat }

func shoulderDiskDomain(ctx context.Context, n windowNative, start, span *big.Rat, meter *proofbound.ReplayBudget) (shoulderDiskRange, bool, error) {
	return shoulderDiskRangeAt(ctx, n, start, span, false, meter)
}

func shoulderDiskRangeAt(ctx context.Context, n windowNative, start, span *big.Rat, aligned bool, meter *proofbound.ReplayBudget) (shoulderDiskRange, bool, error) {
	var none shoulderDiskRange
	if meter == nil || n.out != 1 || n.r.Sign() <= 0 || span.Sign() <= 0 || start.Cmp(n.last) >= 0 {
		return none, false, nil
	}
	if err := meter.ReserveBundle(ctx, 1800, 0, 1<<20); err != nil {
		return none, false, err
	}
	pole := minus(n.cx, n.r)
	if start.Cmp(pole) >= 0 || pole.Cmp(n.last) >= 0 || n.last.Cmp(n.cx) >= 0 {
		return none, false, nil
	}
	r2 := times(n.r, n.r)
	// Every source point is outward of the toe; minimize its distance to the
	// disk along the finite toe line, including an interior stationary point.
	residual := func(x, offset *big.Rat) *big.Rat {
		dx, dz := minus(x, n.cx), minus(minus(n.side(x), offset), n.cz)
		return minus(plus(times(dx, dx), times(dz, dz)), r2)
	}
	minimum := func(left, right, offset *big.Rat) *big.Rat {
		h := minus(minus(n.side(n.cx), offset), n.cz)
		at := minus(n.cx, divide(times(n.m, h), plus(scalar(1), times(n.m, n.m))))
		return residual(wmin(right, wmax(left, at)), offset)
	}
	if !aligned && minimum(start, n.last, scalar(0)).Sign() <= 0 {
		return none, false, nil
	}
	// At the terminal toe the circle must actually enter the strip.
	if residual(n.last, span).Sign() >= 0 {
		return none, false, nil
	}
	for right := n.last; ; {
		at := divide(plus(pole, right), scalar(2))
		if minus(minus(n.side(at), span), n.cz).Sign() > 0 && minus(minus(n.side(pole), span), n.cz).Sign() > 0 && minimum(pole, at, span).Sign() > 0 {
			dx := minus(at, n.cx)
			minOrd := scalar(proofbound.RatSqrtDown(minus(r2, times(dx, dx))))
			dx = minus(n.last, n.cx)
			endOrd := scalar(proofbound.RatSqrtDown(minus(r2, times(dx, dx))))
			if minOrd == nil || endOrd == nil || minOrd.Sign() <= 0 || endOrd.Sign() <= 0 {
				return none, false, nil
			}
			gap := minus(minus(n.side(n.last), n.cz), endOrd)
			if aligned {
				cross, _, ok, err := windowCrossing(ctx, n, at, meter)
				if !ok || err != nil {
					return none, false, err
				}
				return shoulderDiskRange{at, minOrd, scalar(0), cross}, true, nil
			}
			if gap.Sign() <= 0 {
				return none, false, nil
			}
			return shoulderDiskRange{at, minOrd, gap, nil}, true, nil
		}
		if minus(at, pole).Cmp(divide(minus(n.last, pole), scalar(1<<20))) <= 0 {
			return none, false, nil
		}
		right = at
	}
}

// TransverseLoftStop reads the outward lobe beside the original side plane.
// The signed shoulder chart points toward decreasing world X.
func TransverseLoftStop(ctx context.Context, p *loftrecord.Payload, wing int, capY, sideX, lowX, highX *big.Rat, meter *proofbound.ReplayBudget) (station, toeX, heelY *big.Rat, ok bool, err error) {
	chart, ok, err := shoulderLoft(ctx, p, wing, meter)
	if !ok || err != nil {
		return nil, nil, nil, false, err
	}
	neg := func(a *big.Rat) *big.Rat { return new(big.Rat).Neg(a) }
	q, ok, err := loftWindowSeam(ctx, &chart, r3.Vec{}, capY, neg(sideX), neg(highX), neg(lowX), -1, meter)
	if !ok || err != nil {
		return nil, nil, nil, false, err
	}
	return signed(q.x, -wing), neg(q.toeZ), q.heelY, true, nil
}
