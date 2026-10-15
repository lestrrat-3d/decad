package decad

import (
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// capBlendOccupiedVolumeAdmission lets capband inspect the root payload's loops.
// A draft view's corners are read under its own sharp rule (offsetJoins), so
// a reflex line-line corner there is the miter it is built as, never the
// chamfer's apex arc.
// The root builders stay here because they own the corner and offset geometry.
// The delayed join builder runs only after the wall checks, preserving the
// proof budget charges and the first refusal reported.
func capBlendOccupiedVolumeAdmission(budget *proofbound.WorkBudget, cbp capBlendPayload) (error, error) {
	return capband.OccupiedVolumeAdmission(budget, cbp.Noun(), cbp.Loops(), cbp.StartLoops, cbp.EndLoops,
		func(li int, loop sectionrecord.LoopRecord) ([]survey2d.SideWalk, func() ([]bool, error), error) {
			cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
			if err != nil {
				return nil, nil, err
			}
			return cl.Walks, func() ([]bool, error) {
				joins, err := cbp.OffsetJoins(budget, li, cl, cbp.LoopOffset(li))
				if err != nil {
					return nil, err
				}
				arcs := make([]bool, len(joins))
				for i := range joins {
					arcs[i] = joins[i].Arc
				}
				return arcs, nil
			}, nil
		})
}
