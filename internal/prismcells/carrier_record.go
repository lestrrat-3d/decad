package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// HasSharedSpans reports whether the arrangement withdrew a span from one
// operand's entity under the other's (agent-docs/general-boolean-design.md §3
// A3). Two such entities are near-identical carriers sketch merged, which the
// denoted section keeps as two walls.
func (d SceneDelta) HasSharedSpans() bool { return len(d.shared.Spans) > 0 }

// ExactOperandsRecord is the carrier record of a merged one-loop Union of two
// undisplaced operands under the identity re-expression with no walk charge
// and no shared span (agent-docs/carrier-record-design.md §5.2). Every
// carrier is an operand's own exact record, so every segment entry is zero.
// A junction where the arrangement computed a parameter on either side takes
// the larger cut allowance (CutDelta) of its two edges; a junction both sides
// reach at their recorded bounds is an operand's own recorded point and takes
// zero. chain holds the merged loop's edges in record order. ok=false means
// the record does not fit sectionDelta and the caller sets none.
func ExactOperandsRecord(chain []sketch.BoundaryEdge, merged momentinput.Profile,
	sectionDelta float64) (*prismrecord.CarrierRecord, bool, error) {
	segs := merged.Outer.Segments
	if len(merged.Holes) != 0 || len(chain) != len(segs) || len(segs) == 0 {
		return nil, false, nil
	}
	cr := prismrecord.ZeroCarrierRecord(merged)
	cr.SectionDelta = sectionDelta
	n := len(segs)
	for j := range n {
		k := (j + 1) % n
		endCut, endDelta, err := cutEnd(chain[j], segs[j], false)
		if err != nil {
			return nil, false, err
		}
		startCut, startDelta, err := cutEnd(chain[k], segs[k], true)
		if err != nil {
			return nil, false, err
		}
		delta := 0.0
		if endCut {
			delta = endDelta
		}
		if startCut {
			delta = math.Max(delta, startDelta)
		}
		if delta > sectionDelta {
			return nil, false, nil
		}
		cr.Junction[0][j] = delta
	}
	return &cr, true, nil
}

// cutEnd reports whether the arrangement computed the parameter at one end
// of a recorded edge (cutAt), and that edge's cut allowance.
func cutEnd(e sketch.BoundaryEdge, seg sectionrecord.CurveSegment, atStart bool) (bool, float64, error) {
	t0, t1, err := SegmentParamRange(seg)
	if err != nil {
		return false, 0, err
	}
	t := t1
	if atStart {
		t = t0
	}
	if !cutAt(e, seg, t) {
		return false, 0, nil
	}
	d, err := CutDelta(e, seg)
	return true, d, err
}
