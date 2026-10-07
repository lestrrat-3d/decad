package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/walkconvert"
)

// These adapters preserve root callers while walkconvert owns the per-segment builders.
func walkOf(seg CurveSegment, work *freeform.FreeformWork) (survey2d.SegmentWalk, error) {
	return walkconvert.WalkOf(seg, work)
}

func lineWalkTangentBound(seg LineSeg, u, v float64) float64 {
	return walkconvert.LineWalkTangentBound(seg, u, v)
}

func lineWalkEndBound(seg LineSeg, t, u, v float64) proofbound.WalkEndBound {
	return walkconvert.LineWalkEndBound(seg, t, u, v)
}

func circularWalkEndBound(seg CurveSegment, t, u, v float64) proofbound.WalkEndBound {
	return walkconvert.CircularWalkEndBound(seg, t, u, v)
}

func circularPointBound(seg CurveSegment, t *big.Rat, u, v float64) proofbound.WalkEndBound {
	return walkconvert.CircularPointBound(seg, t, u, v)
}

func arcWalkRadiusBound(seg ArcSeg, held float64) float64 {
	return walkconvert.ArcWalkRadiusBound(seg, held)
}

func pinArcWalkEnds(walk *survey2d.SegmentWalk, seg ArcSeg) {
	walkconvert.PinArcWalkEnds(walk, seg)
}

func circularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper float64) survey2d.SegmentWalk {
	return walkconvert.CircularWalk(cu, cv, r, th0, th1, radiusUpper, sweepUpper)
}

func lineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	return walkconvert.LineWalkBounds(seg, held)
}

func dySqrtIntervalError(lengthSquared proofarith.Dyadic, held float64) float64 {
	return walkconvert.DySqrtIntervalError(lengthSquared, held)
}

func ratL1Upper(values ...*big.Rat) float64 { return walkconvert.RatL1Upper(values...) }

func coalesceWalks(walks []survey2d.SideWalk) []survey2d.SideWalk {
	return walkconvert.CoalesceWalks(walks)
}

func coalesceWalksBudget(walks []survey2d.SideWalk, budget *proofbound.WorkBudget) ([]survey2d.SideWalk, error) {
	return walkconvert.CoalesceWalksBudget(walks, budget)
}

func coalesceWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return walkconvert.CoalesceWalksContext(ctx, walks)
}

func coalesceChainWalksContext(ctx context.Context, walks []survey2d.SideWalk) ([]survey2d.SideWalk, error) {
	return walkconvert.CoalesceChainWalksContext(ctx, walks)
}
