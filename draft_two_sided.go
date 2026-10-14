package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// twoSidedDraftPayload keeps both halves of a taper measured from the sketch
// plane. The two near sections are the same recorded profile at level zero.
type twoSidedDraftPayload struct {
	negative draftPayload
	positive draftPayload
}

func (dp twoSidedDraftPayload) Transform() r3.Transform { return dp.positive.xform }

func (dp twoSidedDraftPayload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	lo0, hi0, delta0, err := dp.negative.ExtentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	lo1, hi1, delta1, err := dp.positive.ExtentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	return math.Min(lo0, lo1), math.Max(hi0, hi1), math.Max(delta0, delta1), nil
}

func (dp twoSidedDraftPayload) AxialDelta() float64 {
	return math.Max(dp.negative.AxialDelta(), dp.positive.AxialDelta())
}

func (dp twoSidedDraftPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	dp.negative.xform = composed
	dp.positive.xform = composed
	return placedNode(evalTwoSidedDraftContext(ctx, d, ref, dp))
}

func evalTwoSidedDraftContext(ctx context.Context, d *Document, ref producerID, dp twoSidedDraftPayload) (*Body, error) {
	negative, err := evalDraftContext(ctx, d, ref, dp.negative)
	if err != nil {
		return nil, wrapStageError("against-side taper", err)
	}
	positive, err := evalDraftContext(ctx, d, ref, dp.positive)
	if err != nil {
		return nil, wrapStageError("along-side taper", err)
	}
	negativePayload, ok := negative.n.Payload.(draftPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: the against-side taper lost its draft record`, ErrDegenerate)
	}
	positivePayload, ok := positive.n.Payload.(draftPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: the along-side taper lost its draft record`, ErrDegenerate)
	}
	parts := make([]compositeSpanPart, 2)
	for i, part := range []*Body{negative, positive} {
		parts[i], err = compositeSpanPartOf(part)
		if err != nil {
			return nil, err
		}
	}
	if err := auditDraftMiddleRim(ctx, parts[0].endCap, parts[1].startCap); err != nil {
		return nil, err
	}
	// The single-slab band records its wall loops from the sketch rim toward
	// its far rim on either side. Reverse the against-side published loops so
	// the two wall faces use their shared middle edge in opposite directions.
	for _, face := range negative.Faces() {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if face != parts[0].endCap {
			reverseDraftFaceLoops(face)
		}
	}
	body, err := assembleCompositeSweepBody(ctx, d, ref, parts, false)
	if err != nil {
		return nil, err
	}
	if err := aggregateCompositeSweepMeasurements(ctx, body, parts, false); err != nil {
		return nil, err
	}
	dp.negative = negativePayload
	dp.positive = positivePayload
	body.n.Payload = dp
	return body, nil
}

func reverseDraftFaceLoops(face *Face) {
	for _, loop := range face.n.Loops {
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
func auditDraftMiddleRim(ctx context.Context, from, to *Face) error {
	return brepgeom.AuditMiddleRim(ctx, draftRimEnds(from), draftRimEnds(to))
}

func draftRimEnds(face *Face) [][]brepgeom.RimEdgeEnds {
	loops := make([][]brepgeom.RimEdgeEnds, len(face.n.Loops))
	for li, loop := range face.n.Loops {
		ends := make([]brepgeom.RimEdgeEnds, len(loop.CoEdges))
		for ci, ce := range loop.CoEdges {
			ends[ci] = brepgeom.RimEdgeEnds{Start: coedgeStart(ce).Position, End: coedgeEnd(ce).Position}
		}
		loops[li] = ends
	}
	return loops
}
