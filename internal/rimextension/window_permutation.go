package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ShoulderPoint takes world coordinates into a chart that runs from the
// previous stop toward the nose. Every component is copied or sign-flipped.
func ShoulderPoint(world r3.Vec, wing int) r3.Vec {
	return r3.Vec{X: -float64(wing) * world.Z, Y: world.Y, Z: -world.X}
}

// shoulderLoft expresses both original sealed source sections in that chart.
// Reflection reverses the outer loop. The local +U,+V axes become -Z,-X.
// Both sections undergo the identical segment reversal, preserving their
// original ruled correspondence. Accept only complete line and arc walks.
func shoulderLoft(ctx context.Context, src *loftrecord.Payload, wing int, meter *proofbound.ReplayBudget) (loftrecord.Payload, bool, error) {
	var none loftrecord.Payload
	if (wing != 1 && wing != -1) || src == nil || meter == nil || src.Xform != r3.Identity() ||
		len(src.Alignment) > 1 || len(src.Alignment) == 1 && src.Alignment[0] != 0 {
		return none, false, nil
	}
	out := *src
	mapPoint := func(p momentinput.Point2) momentinput.Point2 { return momentinput.Point2{U: -p.V, V: -p.U} }
	for j, p := range [2]momentinput.Profile{src.Profile0, src.Profile1} {
		frame := [2]r3.Frame{src.Frame0, src.Frame1}[j]
		if frame.U() != (r3.Vec{X: 1}) || frame.V() != (r3.Vec{Z: float64(wing)}) || len(p.Holes) != 0 ||
			len(p.Outer.Segments) < 3 || len(p.Outer.Segments) > 64 {
			return none, false, nil
		}
		n := len(p.Outer.Segments)
		if err := meter.ReserveBundle(ctx, uint64(n)*32, uint64(n), uint64(n)*256+1024); err != nil {
			return none, false, err
		}
		f, err := r3.NewFrame(ShoulderPoint(frame.Origin(), wing), r3.Vec{X: 1}, r3.Vec{Z: 1})
		if err != nil {
			return none, false, err
		}
		var transformed momentinput.Profile
		transformed.Outer.Segments = make([]momentinput.CurveSegment, n)
		for k, seg := range p.Outer.Segments {
			switch s := seg.(type) {
			case momentinput.LineSeg:
				if !wholeShoulderWalk(s.TStart, s.TEnd) {
					return none, false, nil
				}
				s.Start, s.End = mapPoint(s.Start), mapPoint(s.End)
				s.TStart, s.TEnd = s.TEnd, s.TStart
				transformed.Outer.Segments[n-1-k] = s
			case momentinput.ArcSeg:
				if !wholeShoulderWalk(s.TStart, s.TEnd) {
					return none, false, nil
				}
				s.Center, s.Start, s.End = mapPoint(s.Center), mapPoint(s.End), mapPoint(s.Start)
				// Full walks keep their parameter sense after reversing both
				// the carrier and the loop. The radius of the original arc
				// can differ slightly after swapping the endpoint. Both proof
				// readers below enclose the two endpoint radii.
				transformed.Outer.Segments[n-1-k] = s
			default:
				return none, false, nil
			}
		}
		if j == 0 {
			out.Profile0, out.Frame0 = transformed, f
		} else {
			out.Profile1, out.Frame1 = transformed, f
		}
	}
	return out, true, nil
}
func wholeShoulderWalk(a, b float64) bool { return a == 0 && b == 1 || a == 1 && b == 0 }

// A counterclockwise minor arc is contained in the cone of its endpoint
// rays. If both rays point left, the whole arc is at or left of its center,
// even when the two recorded endpoint radii round differently.
func windowLoftSegmentUpper(seg any) *big.Rat {
	s, ok := seg.(momentinput.ArcSeg)
	if !ok || !wholeShoulderWalk(s.TStart, s.TEnd) {
		return windowSegmentUpper(seg)
	}
	x, y := scalar(s.Center.U), scalar(s.Center.V)
	ax, ay, bx, by := scalar(s.Start.U), scalar(s.Start.V), scalar(s.End.U), scalar(s.End.V)
	if x == nil || y == nil || ax == nil || ay == nil || bx == nil || by == nil {
		return nil
	}
	ax, ay, bx, by = minus(ax, x), minus(ay, y), minus(bx, x), minus(by, y)
	if ax.Sign() <= 0 && bx.Sign() <= 0 && minus(times(ax, by), times(ay, bx)).Sign() > 0 {
		return x
	}
	return windowSegmentUpper(seg)
}
