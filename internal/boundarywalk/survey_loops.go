package boundarywalk

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ErrFreeformSection reports a boundary segment the analytic wall survey
// cannot read.
var ErrFreeformSection = fmt.Errorf(`%w: the wall survey does not support a free-form boundary segment`, decaderr.ErrUnsupported)

// SurveyLoops resolves a recorded profile into coalesced walks, matching the
// prism evaluator's side-face decomposition.
func SurveyLoops(budget *proofbound.WorkBudget, profile Profile) ([][]survey2d.SideWalk, error) {
	// One counter spans every loop because surveys read a built body's section
	// without a preflight counter to carry into this resolution.
	work := freeform.NewFreeformWork()
	var out [][]survey2d.SideWalk
	for _, loop := range append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		raw := make([]survey2d.SideWalk, len(loop.Segments))
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			w, err := WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			if err := RequireAnalyticWalk(w, "the wall survey"); err != nil {
				// This one refusal has the survey's own sentinel; other errors retain
				// the identity returned by WalkOf.
				return nil, ErrFreeformSection
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
		}
		walks, err := CoalesceWalksBudget(raw, budget)
		if err != nil {
			return nil, err
		}
		out = append(out, walks)
	}
	return out, nil
}

// SurveyLoopsBudget performs a cancellation check before resolving the profile.
func SurveyLoopsBudget(budget *proofbound.WorkBudget, profile Profile) ([][]survey2d.SideWalk, error) {
	if err := survey2d.WallBudgetErr(budget); err != nil {
		return nil, err
	}
	return SurveyLoops(budget, profile)
}
