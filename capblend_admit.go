package decad

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// capBlendOccupiedVolumeAdmission lets capband inspect the root payload's loops.
// The root builders stay here because they own the corner and offset geometry.
// The delayed join builder runs only after the wall checks, preserving the
// proof budget charges and the first refusal reported.
func capBlendOccupiedVolumeAdmission(budget *proofbound.WorkBudget, cbp capBlendPayload) (error, error) {
	return capband.OccupiedVolumeAdmission(budget, cbp.loops(), cbp.startLoops, cbp.endLoops,
		func(loop sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := oneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.walks, func() ([]bool, error) {
				joins, err := capOffsetJoins(budget, cl, cbp.d)
				if err != nil {
					return nil, err
				}
				arcs := make([]bool, len(joins))
				for i := range joins {
					arcs[i] = joins[i].arc
				}
				return arcs, nil
			}, nil
		})
}

func capBlendSegmentRefusal(seg CurveSegment) string { return capband.SegmentRefusal(seg) }
func capJoinIsG1(prev, cur CurveSegment) bool        { return capband.JoinIsG1(prev, cur) }
