package boundarywalk

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// JunctionVertexAt places the rim vertex where walk prev ends and walk next
// starts, and returns its plane position with the 3D bound it owes against the
// points BOTH walks denote there (JunctionVertex): next's
// recorded start and prev's recorded end, which differ at a cut junction
// (docs/evaluator-design.md §3, §4). segs are the recorded segments the walks'
// Segs index; a single whole closed walk passes itself as both neighbours.
// Every kind charges its own walk-end bound: a trimmed line's lerp rounding, a
// circle's or trimmed arc's trig enclosure, a free-form chain's control-point
// rounding, and an arc's natural t = 1 radial residual. A vertex is therefore
// Exact only where every end meeting there is a recorded coordinate it holds
// verbatim (topology.go's Vertex.Position contract). proofbound.WalkEndBoundAllow
// carries the per-component bound into world space.
func JunctionVertexAt(segs []sectionrecord.CurveSegment, prev, next survey2d.SideWalk) (float64, float64, float64) {
	u, v, bound := JunctionVertex(
		segs[prev.Segs[len(prev.Segs)-1]], prev.SegmentWalk, segs[next.Segs[0]], next.SegmentWalk)
	return u, v, proofbound.WalkEndBoundAllow(bound)
}

// RimConvexity decides one walk's rim-edge convexity — evaluator §3's
// walked-boundary rule (topology.go's Edge.IsConvex doc comment) —
// independently of the curve, surface and faceReversed construction
// prismrecord.BuildLoopSidesAs still runs for its own kind right after it. holeLoop is
// the loop's material side; a walk with no turn of its own (a straight wall,
// or a free-form chain §6.5 proves straight) reads it, and a walk that DOES
// turn — circular or genuinely curved free-form — decides convexity from its
// own turn instead, exactly as prismrecord.BuildLoopSidesAs's per-kind switch did before
// this was pulled out of it.
func RimConvexity(ctx context.Context, w survey2d.SideWalk, holeLoop bool, work *freeform.FreeformWork) (bool, error) {
	switch w.Kind {
	case survey2d.WalkCircular:
		// A clockwise walk's material lies outside its circle, so the
		// boundary turns into the metal there and the rim is concave — a
		// hole's rim (clockwise) and a concave outer bite's (also
		// clockwise) alike, while a counter-clockwise round keeps its
		// convex rim. The loop's role decides nothing here.
		return w.Th1 >= w.Th0, nil
	case survey2d.WalkFreeform:
		// §6.5's wall-edge convexity certificate (docs/spline-design.md
		// §6.5, Table R R19). It applies the one reversal negation
		// internally and states its verdict in the LOOP'S OWN WALK
		// direction, so the mapping below reads it verbatim — a
		// counter-clockwise turn convex, clockwise concave, the identical
		// convention the circular case above fixes. NEVER negate again for
		// a hole loop: a hole rim's concavity falls out of the clockwise
		// walk itself, exactly as it does for a circular hole wall.
		verdict, err := freeform.FreeformWallConvexityContext(ctx, w.Spans, w.Closed, w.Reversed, w.FitInterpolated, work)
		if err != nil {
			return false, err
		}
		switch verdict {
		case freeform.FreeformConvexityPositive:
			return true, nil
		case freeform.FreeformConvexityNegative:
			return false, nil
		case freeform.FreeformConvexityStraight:
			// Every live span lies on one line and no joint turns off it
			// (§6.5's Table K): the chain has no turn of its own, so
			// evaluator §3's straight-wall rule decides it by its loop's
			// role — the same expression the default arm below uses.
			return !holeLoop, nil
		}
		// The three verdicts above are exhaustive today, so this is
		// unreachable — but a future fourth verdict silently falling through
		// to !holeLoop would be a straight-wall answer for a wall this
		// evaluator never proved straight, exactly the quiet wrong answer
		// this project exists to prevent. Refuse explicitly rather than guess.
		return false, fmt.Errorf(`%w: free-form wall convexity verdict %d is not handled by this evaluator`, decaderr.ErrUnsupported, verdict)
	default:
		// A straight wall has no turn of its own to disagree with the
		// loop's: which side its material lies on is decided by the sense
		// the whole loop is walked, and that sense IS the loop's role —
		// the outer loop counter-clockwise, holes clockwise (moments.go).
		return !holeLoop, nil
	}
}
