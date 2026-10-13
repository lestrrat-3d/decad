package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// CanonicalStackedCut rebuilds a stacked cut's section from the target's own
// records. The structural match already proved that every retained target loop survives whole.
// Keep those records verbatim so a wall shared across slabs stays one column;
// take only the new tool hole from the authenticated arranged profile.
func CanonicalStackedCut(budget *proofbound.WorkBudget, target momentinput.Profile, match *sketch.Profile,
	tags map[sketch.Entity]Origin, candidate momentinput.Profile) (momentinput.Profile, error) {
	if len(match.Holes) != len(candidate.Holes) || len(candidate.Holes) != len(target.Holes)+1 {
		return momentinput.Profile{}, fmt.Errorf(`%w: the cut profile has an unexpected hole count`, decaderr.ErrUnsupported)
	}
	toolEntities, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return momentinput.Profile{}, err
	}
	toolIndex := -1
	for j, hole := range match.Holes {
		isTool, err := LoopMatchesOrigin(budget, hole, toolEntities)
		if err != nil {
			return momentinput.Profile{}, err
		}
		if isTool {
			if toolIndex >= 0 {
				return momentinput.Profile{}, fmt.Errorf(`%w: more than one result hole matches the tool`, decaderr.ErrUnsupported)
			}
			toolIndex = j
		}
	}
	if toolIndex < 0 {
		return momentinput.Profile{}, fmt.Errorf(`%w: no result hole matches the tool`, decaderr.ErrUnsupported)
	}
	result := momentinput.Profile{Outer: target.Outer, Holes: append([]LoopRecord(nil), target.Holes...)}
	result.Holes = append(result.Holes, candidate.Holes[toolIndex])
	return result, nil
}
