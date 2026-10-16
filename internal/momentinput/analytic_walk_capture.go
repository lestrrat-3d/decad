package momentinput

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// analyticWalkCapture transfers successful preflight walks into the prism's
// existing cache. A free-form segment or any measured charge disables the
// whole capture; none of its arrays or converted spans survive that decision.
type analyticWalkCapture struct {
	source                         Profile
	outer                          []survey2d.SegmentWalk
	holes                          [][]survey2d.SegmentWalk
	loopIndex, segmentIndex, count int
	disabled                       bool
}

func (c *analyticWalkCapture) active() bool { return c != nil && !c.disabled }

func (c *analyticWalkCapture) disable() {
	if c == nil {
		return
	}
	c.disabled = true
	c.outer, c.holes = nil, nil
	c.source = Profile{}
	c.count = 0
}

func (c *analyticWalkCapture) retain(w survey2d.SegmentWalk, before, beforeRecon, after, afterRecon uint64) {
	if !c.active() {
		return
	}
	if before != after || beforeRecon != afterRecon || w.Spans != nil ||
		(w.Kind != survey2d.WalkLine && w.Kind != survey2d.WalkCircular) {
		c.disable()
		return
	}
	// Allocate only after the walk has passed every moment preflight check.
	if c.loopIndex == 0 {
		if c.outer == nil {
			c.outer = make([]survey2d.SegmentWalk, len(c.source.Outer.Segments))
		}
		c.outer[c.segmentIndex] = w
	} else {
		if c.holes == nil {
			c.holes = make([][]survey2d.SegmentWalk, len(c.source.Holes))
		}
		hi := c.loopIndex - 1
		if c.holes[hi] == nil {
			c.holes[hi] = make([]survey2d.SegmentWalk, len(c.source.Holes[hi].Segments))
		}
		c.holes[hi][c.segmentIndex] = w
	}
	c.count++
}

func (c *analyticWalkCapture) take(normalized Profile) *ProfileWalks {
	if !c.active() {
		return nil
	}
	if len(c.outer) != len(normalized.Outer.Segments) || len(c.holes) != len(normalized.Holes) {
		c.disable()
		return nil
	}
	count := len(c.outer)
	for i, hole := range normalized.Holes {
		if len(c.holes[i]) != len(hole.Segments) {
			c.disable()
			return nil
		}
		count += len(c.holes[i])
	}
	if count == 0 || c.count != count {
		c.disable()
		return nil
	}
	result := &ProfileWalks{Profile: normalized, Outer: c.outer, Holes: c.holes, Metered: true}
	// The zero charges were measured around every original WalkOf call, not
	// inferred from the segment kind. Ownership of the arrays moves to result.
	c.disable()
	return result
}

// EvaluatorIntegralsWithAnalyticWalksContext also returns complete zero-charge
// analytic walks from the moment preflight. The caller must check Reusable
// against its original record before adopting them: normalization may change
// the record's dynamic types or units. No free-form walks are captured.
func (r Profile) EvaluatorIntegralsWithAnalyticWalksContext(ctx context.Context,
	order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (Integrals, *ProfileWalks, error) {
	capture := analyticWalkCapture{source: r}
	pre, err := validateFieldsWithCapture(ctx.Err, r, work, &capture)
	if err != nil {
		capture.disable()
		return Integrals{}, nil, err
	}
	ig, err := integrateMomentRecordUncheckedContext(ctx, pre, order)
	if err != nil {
		capture.disable()
		return Integrals{}, nil, err
	}
	return ig, capture.take(pre.Record), nil
}
