package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// twoSidedDraftPayload keeps both halves of a taper measured from the sketch
// plane. The two near sections are the same recorded profile at level zero.
type twoSidedDraftPayload struct {
	negative draftPayload
	positive draftPayload
}

func (dp twoSidedDraftPayload) transform() r3.Transform { return dp.positive.xform }

func (dp twoSidedDraftPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	lo0, hi0, delta0, err := dp.negative.extentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	lo1, hi1, delta1, err := dp.positive.extentAlong(g)
	if err != nil {
		return 0, 0, 0, err
	}
	return math.Min(lo0, lo1), math.Max(hi0, hi1), math.Max(delta0, delta1), nil
}

func (dp twoSidedDraftPayload) axialDelta() float64 {
	return math.Max(dp.negative.axialDelta(), dp.positive.axialDelta())
}

func (dp twoSidedDraftPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	dp.negative.xform = composed
	dp.positive.xform = composed
	return evalTwoSidedDraftContext(ctx, d, ref, dp)
}

func evalTwoSidedDraftContext(ctx context.Context, d *Document, ref producerID, dp twoSidedDraftPayload) (*Body, error) {
	negative, err := evalDraftContext(ctx, d, ref, dp.negative)
	if err != nil {
		return nil, fmt.Errorf("against-side taper: %w", err)
	}
	positive, err := evalDraftContext(ctx, d, ref, dp.positive)
	if err != nil {
		return nil, fmt.Errorf("along-side taper: %w", err)
	}
	negativePayload, ok := negative.payload.(draftPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: the against-side taper lost its draft record`, ErrDegenerate)
	}
	positivePayload, ok := positive.payload.(draftPayload)
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
	body.payload = dp
	return body, nil
}

func reverseDraftFaceLoops(face *Face) {
	for _, loop := range face.loops {
		for i, j := 0, len(loop.coedges)-1; i < j; i, j = i+1, j-1 {
			loop.coedges[i], loop.coedges[j] = loop.coedges[j], loop.coedges[i]
		}
		for i := range loop.coedges {
			loop.coedges[i].forward = !loop.coedges[i].forward
		}
	}
}

// The two builders read the same sketch section in the same frame at zero.
// Check their held rims before replacing either set of vertices or edges.
func auditDraftMiddleRim(ctx context.Context, from, to *Face) error {
	if len(from.loops) != len(to.loops) {
		return fmt.Errorf(`%w: two-sided taper halves have different middle loop counts`, ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	for li := range from.loops {
		if err := budget.Step(); err != nil {
			return err
		}
		left, right := from.loops[li].coedges, to.loops[li].coedges
		if len(left) != len(right) {
			return fmt.Errorf(`%w: two-sided taper halves have different middle rim edge counts`, ErrUnsupported)
		}
		for ci := range left {
			if err := budget.Step(); err != nil {
				return err
			}
			if left[ci].Start().position != right[ci].Start().position ||
				left[ci].End().position != right[ci].End().position {
				return fmt.Errorf(`%w: two-sided taper halves do not share the held middle rim`, ErrUnsupported)
			}
		}
	}
	return budget.Err()
}
