package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// RecordOverlapCell records one arranged cell's closed boundary without
// merging it with other cells. The cell already has sketch's directed order,
// so no boundary assembly is needed. Each recorded edge is checked against
// its join, and the returned cut charge covers its fragments.
func RecordOverlapCell(budget *proofbound.WorkBudget, edges []sketch.BoundaryEdge) (momentinput.Profile, float64, error) {
	segs := make([]sectionrecord.CurveSegment, len(edges))
	joins := make([]sketchrecord.LoopJoin, len(edges))
	cutDelta := 0.0
	for i, e := range edges {
		if err := budget.Step(); err != nil {
			return momentinput.Profile{}, 0, err
		}
		seg, err := sketchrecord.RecordEdge(e)
		if err != nil {
			return momentinput.Profile{}, 0, err
		}
		segs[i] = seg
		join, err := sketchrecord.EdgeJoin(e, seg)
		if err != nil {
			return momentinput.Profile{}, 0, err
		}
		joins[i] = join
		delta, err := CutDelta(e, seg)
		if err != nil {
			return momentinput.Profile{}, 0, err
		}
		cutDelta = math.Max(cutDelta, delta)
	}
	if err := sketchrecord.FalsifyLoopJoins("overlap cell", joins); err != nil {
		return momentinput.Profile{}, 0, err
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}, cutDelta, nil
}
