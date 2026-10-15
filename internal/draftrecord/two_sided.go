package draftrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file is the two-sided taper: two single-slab draft bodies built from
// the sketch plane outward and sewn at their shared middle rim
// (docs/draft-design.md). TwoSidedPayload implements topo.Payload
// (docs/evaluator-design.md §8).

// TwoSidedPayload keeps both halves of a taper measured from the sketch
// plane. The two near sections are the same recorded profile at level zero.
type TwoSidedPayload struct {
	Negative Payload
	Positive Payload
}

// Transform is the accumulated rigid placement.
func (dp TwoSidedPayload) Transform() r3.Transform { return dp.Positive.Xform }

// ExtentAlong is the through-all stop's reading of the two-sided draft body:
// the hull of both halves' readings.
func (dp TwoSidedPayload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	lo0, hi0, delta0, err := dp.Negative.ExtentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	lo1, hi1, delta1, err := dp.Positive.ExtentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	return math.Min(lo0, lo1), math.Max(hi0, hi1), math.Max(delta0, delta1), nil
}

// AxialDelta is the larger sweep-level displacement of either half.
func (dp TwoSidedPayload) AxialDelta() float64 {
	return math.Max(dp.Negative.AxialDelta(), dp.Positive.AxialDelta())
}

// Placed re-evaluates both halves under the composed motion.
func (dp TwoSidedPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	dp.Negative.Xform = composed
	dp.Positive.Xform = composed
	return BuildTwoSided(ctx, env, dp)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (docs/evaluator-design.md §8): the root type this payload replaced.
func (TwoSidedPayload) SurveyClass() string { return "twoSidedDraftPayload" }

// BuildTwoSided builds both halves under env, audits their shared middle rim
// and sews them into one body, never committing it.
func BuildTwoSided(ctx context.Context, env topo.Env, dp TwoSidedPayload) (*topo.Body, error) {
	negative, err := Build(ctx, env, dp.Negative)
	if err != nil {
		return nil, compositesweep.WrapStageError("against-side taper", err)
	}
	positive, err := Build(ctx, env, dp.Positive)
	if err != nil {
		return nil, compositesweep.WrapStageError("along-side taper", err)
	}
	negativePayload, ok := negative.Payload.(Payload)
	if !ok {
		return nil, fmt.Errorf(`%w: the against-side taper lost its draft record`, decaderr.ErrDegenerate)
	}
	positivePayload, ok := positive.Payload.(Payload)
	if !ok {
		return nil, fmt.Errorf(`%w: the along-side taper lost its draft record`, decaderr.ErrDegenerate)
	}
	parts := make([]compositesweep.SpanPart, 2)
	for i, part := range []*topo.Body{negative, positive} {
		parts[i], err = compositesweep.SpanPartOf(part)
		if err != nil {
			return nil, err
		}
	}
	if err := auditMiddleRim(ctx, parts[0].EndCap, parts[1].StartCap); err != nil {
		return nil, err
	}
	// The single-slab band records its wall loops from the sketch rim toward
	// its far rim on either side. Reverse the against-side published loops so
	// the two wall faces use their shared middle edge in opposite directions.
	for _, face := range topobuild.Faces(negative) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if face != parts[0].EndCap {
			reverseFaceLoops(face)
		}
	}
	body, err := compositesweep.Assemble(ctx, env, parts, false,
		compositesweep.AuditBodies, compositesweep.SewJoin, false)
	if err != nil {
		return nil, err
	}
	if err := compositesweep.AggregateBody(ctx, body, parts, false); err != nil {
		return nil, err
	}
	dp.Negative = negativePayload
	dp.Positive = positivePayload
	body.Payload = dp
	return body, nil
}

func reverseFaceLoops(face *topo.Face) {
	for _, loop := range face.Loops {
		for i, j := 0, len(loop.CoEdges)-1; i < j; i, j = i+1, j-1 {
			loop.CoEdges[i], loop.CoEdges[j] = loop.CoEdges[j], loop.CoEdges[i]
		}
		for i := range loop.CoEdges {
			loop.CoEdges[i].Forward = !loop.CoEdges[i].Forward
		}
	}
}

// The two builders read the same sketch section in the same frame at zero.
// Check their held rims before replacing either set of vertices or edges.
func auditMiddleRim(ctx context.Context, from, to *topo.Face) error {
	return brepgeom.AuditMiddleRim(ctx, rimEnds(from), rimEnds(to))
}

func rimEnds(face *topo.Face) [][]brepgeom.RimEdgeEnds {
	loops := make([][]brepgeom.RimEdgeEnds, len(face.Loops))
	for li, loop := range face.Loops {
		ends := make([]brepgeom.RimEdgeEnds, len(loop.CoEdges))
		for ci, ce := range loop.CoEdges {
			ends[ci] = brepgeom.RimEdgeEnds{Start: topobuild.CoEdgeStart(ce).Position, End: topobuild.CoEdgeEnd(ce).Position}
		}
		loops[li] = ends
	}
	return loops
}
