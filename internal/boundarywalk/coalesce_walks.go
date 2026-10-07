package boundarywalk

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

func CoalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk { return coalesceWalks(walks) }
func CoalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return coalesceWalksBudget(walks, budget)
}
func CoalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksContext(ctx, walks)
}
func CoalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceChainWalksContext(ctx, walks)
}

// coalesceWalks merges consecutive collinear line walks, wrap-around
// included. Circular walks never merge; a loop that is entirely one straight
// line is degenerate and left to the area gate.
func coalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk {
	out, _ := coalesceWalksBudget(walks, nil)
	return out
}

func coalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(func() error { return survey2d.WallBudgetStep(budget) }, walks, true)
}

func coalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, true)
}

// coalesceChainWalksContext is coalesceWalksContext's OPEN-walk counterpart:
// it merges adjacent collinear segments exactly as a loop's coalescing does,
// but never wraps the last walk into the first. An open chain's two ends are
// free — they meet no neighbour to merge into
// (docs/surface-design.md §13.4).
func coalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return coalesceWalksWithPoll(ctx.Err, walks, false)
}

func coalesceWalksWithPoll(poll func() error, walks []survey2d.SideWalk, wrap bool) ([]survey2d.SideWalk, error) {
	collinear := func(a, b survey2d.SideWalk) bool {
		if !a.IsLine() || !b.IsLine() {
			return false
		}
		cross := a.TanOutU*b.TanInV - a.TanOutV*b.TanInU
		dot := a.TanOutU*b.TanInU + a.TanOutV*b.TanInV
		scale := math.Hypot(a.TanOutU, a.TanOutV) * math.Hypot(b.TanInU, b.TanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale
	}
	merge := func(a, b survey2d.SideWalk) survey2d.SideWalk {
		a.EndU, a.EndV = b.EndU, b.EndV
		// The merged walk leaves where b leaves, so it inherits b's leaving
		// tangent AND the bound b proved on it — never a's, and never zero.
		a.TanOutU, a.TanOutV = b.TanOutU, b.TanOutV
		a.TanOutBound = b.TanOutBound
		length := proofbound.BoundedAdd(proofbound.MeasuredScalar(a.Length, a.LengthBound), proofbound.MeasuredScalar(b.Length, b.LengthBound))
		a.Length, a.LengthBound = length.Value, length.Bound
		a.LengthUpper = proofbound.AbsSumUpper(a.LengthUpper, b.LengthUpper)
		a.CoordUpper = math.Max(a.CoordUpper, b.CoordUpper)
		a.AxisRadiusUpper = math.Max(a.AxisRadiusUpper, b.AxisRadiusUpper)
		a.AxisMomentUpper = proofbound.AbsSumUpper(a.AxisMomentUpper, b.AxisMomentUpper)
		a.Segs = append(a.Segs, b.Segs...)
		return a
	}
	out := make([]survey2d.SideWalk, 0, len(walks))
	for _, w := range walks {
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		if len(out) > 0 && collinear(out[len(out)-1], w) {
			out[len(out)-1] = merge(out[len(out)-1], w)
			continue
		}
		out = append(out, w)
	}
	// Wrap-around: a closed loop's last walk may continue into its first. An
	// open chain's never does (wrap is false), since its last segment meets
	// no neighbour at all.
	for wrap {
		if len(out) <= 1 || !collinear(out[len(out)-1], out[0]) {
			break
		}
		if poll != nil {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		out[0] = merge(out[len(out)-1], out[0])
		out = out[:len(out)-1]
	}
	return out, nil
}
