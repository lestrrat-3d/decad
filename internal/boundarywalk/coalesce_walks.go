package boundarywalk

import (
	"context"
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// line is degenerate and left to the area gate. Collinear means EXACTLY
// collinear (exactlyCollinear): a merged walk stands for every segment it
// covers, so a merge across a kink, however slight, would publish a wall the
// record does not have.
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
		// The tangent test is only a quick reject: every exactly collinear
		// pair passes it, since its tangents differ by their own rounding
		// alone. The merge itself is decided exactly (exactlyCollinear).
		cross := a.TanOutU*b.TanInV - a.TanOutV*b.TanInU
		dot := a.TanOutU*b.TanInU + a.TanOutV*b.TanInV
		scale := math.Hypot(a.TanOutU, a.TanOutV) * math.Hypot(b.TanInU, b.TanInV)
		return dot > 0 && math.Abs(cross) <= 1e-12*scale && exactlyCollinear(a, b)
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

// exactlyCollinear reports whether line walk b continues line walk a along one
// exact line: they meet at one float junction point, a's chord (from a's
// start, which is the first start of every walk already merged into a) and
// b's chord are parallel and point the same way over the rationals, and
// neither side's proven bound on that junction reaches off the line.
//
// The merged walk keeps a's start and b's end, with their bounds, and drops
// the junction, and every consumer — the topology's one face, the clearance
// carriers, the surveys and the tessellation — reads the chord between those
// two ends. The chord IS the recorded boundary only when the junction the
// record denotes lies on it. A junction bound lets the denoted point sit
// anywhere in a box about the float one, so the box must lie along the line:
// a bound may only be nonzero in a coordinate the chord itself moves in while
// the other stays fixed, as a cut fragment of an axis-aligned wall's is. A
// tolerance here would merge a kink and drop its vertex: a wall whose kink
// sits 1e-10 off the chord would read a clearance Exact at the chord's
// distance.
func exactlyCollinear(a, b survey2d.SideWalk) bool {
	if a.EndU != b.StartU || a.EndV != b.StartV {
		return false
	}
	var leaves [6]proofarith.Dyadic
	for i, f := range [...]float64{a.StartU, a.StartV, a.EndU, a.EndV, b.EndU, b.EndV} {
		d, ok := proofarith.DyOf(f)
		if !ok {
			return false
		}
		leaves[i] = d
	}
	au := proofarith.DySubScalar(leaves[2], leaves[0])
	av := proofarith.DySubScalar(leaves[3], leaves[1])
	bu := proofarith.DySubScalar(leaves[4], leaves[2])
	bv := proofarith.DySubScalar(leaves[5], leaves[3])
	cross := proofarith.DySubScalar(proofarith.DyMul(au, bv), proofarith.DyMul(av, bu))
	dot := proofarith.DyAdd(proofarith.DyMul(au, bu), proofarith.DyMul(av, bv))
	if !cross.IsZero() || dot.Sign() <= 0 {
		return false
	}
	// The chord is parallel to both walks, so a's own components say which
	// coordinates it moves in.
	alongLine := func(bound proofbound.WalkEndBound) bool {
		return bound.Derivable() && (bound.U == 0 || av.IsZero()) && (bound.V == 0 || au.IsZero())
	}
	return alongLine(a.EndBound) && alongLine(b.StartBound)
}
