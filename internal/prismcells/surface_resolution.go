package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// TrimNoCrossingSide checks the three whole-loop arrangements that settle
// Trim before the crossing classifier runs. A missing match leaves the side
// unresolved for Classify.
func TrimNoCrossingSide(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, receiverHoles int) (bool, bool, error) {
	rcvOuter, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return false, false, err
	}
	rcvHoles := make([]map[sketch.Entity]struct{}, receiverHoles)
	for i := 0; i < receiverHoles; i++ {
		hs, err := LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return false, false, err
		}
		rcvHoles[i] = hs
	}
	toolOuter, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return false, false, err
	}

	invalid := func(match *sketch.Profile) error {
		if match.Valid {
			return nil
		}
		return fmt.Errorf(`%w: the trim scene's arrangement reports an invalid region`, decaderr.ErrUnsupported)
	}

	if disjointMatch, ok, err := FindLoopMatch(budget, profiles, rcvOuter, rcvHoles); err != nil {
		return false, false, err
	} else if ok {
		return false, true, invalid(disjointMatch)
	}

	toolInsideHoles := append(append([]map[sketch.Entity]struct{}{}, rcvHoles...), toolOuter)
	if toolInsideMatch, ok, err := FindLoopMatch(budget, profiles, rcvOuter, toolInsideHoles); err != nil {
		return false, false, err
	} else if ok {
		return false, true, invalid(toolInsideMatch)
	}

	if rcvInsideMatch, ok, err := FindLoopMatch(budget, profiles, toolOuter, []map[sketch.Entity]struct{}{rcvOuter}); err != nil {
		return false, false, err
	} else if ok {
		return true, true, invalid(rcvInsideMatch)
	}

	return false, false, nil
}

// SplitUnchangedTargetCell finds the target's original loops reproduced by
// one sketch cell. A matching invalid cell keeps Split's existing refusal.
func SplitUnchangedTargetCell(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, targetHoles int) (bool, error) {
	outer, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return false, err
	}
	holes := make([]map[sketch.Entity]struct{}, targetHoles)
	for i := range holes {
		holes[i], err = LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return false, err
		}
	}
	match, found, err := FindLoopMatch(budget, profiles, outer, holes)
	if err != nil || !found {
		return false, err
	}
	if !match.Valid {
		return false, fmt.Errorf(`%w: the unchanged target cell is invalid`, decaderr.ErrUnsupported)
	}
	return true, nil
}

// ExtendSource returns the scene's sole receiver entity. The Extend scene
// contains only one receiver carrier, so map iteration cannot change the pick.
func ExtendSource(tags map[sketch.Entity]Origin) sketch.Entity {
	var source sketch.Entity
	for entity, tag := range tags {
		if !tag.IsB {
			source = entity
			break
		}
	}
	return source
}

// ExtendProfileFragments collects profile edges in the arrangement's order.
func ExtendProfileFragments(profiles []*sketch.Profile) ([]sketch.BoundaryEdge, error) {
	var fragments []sketch.BoundaryEdge
	for _, profile := range profiles {
		if !profile.Valid {
			return nil, fmt.Errorf(`%w: the extend arrangement has an invalid cell`, decaderr.ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			fragments = append(fragments, loop...)
		}
	}
	return fragments, nil
}

// AppendExtendChainFragments adds the chain publication after the profiles.
func AppendExtendChainFragments(fragments []sketch.BoundaryEdge,
	chains []*sketch.Chain) ([]sketch.BoundaryEdge, error) {
	for _, chain := range chains {
		if !chain.Valid {
			return nil, fmt.Errorf(`%w: the extend arrangement has an invalid chain`, decaderr.ErrUnsupported)
		}
		fragments = append(fragments, chain.Edges...)
	}
	return fragments, nil
}

// NearestExtendCut reads only sketch's own cut parameters on the receiver
// entity. It returns the first edge carrying the selected parameter for the
// caller's seam check, preserving the arrangement's publication order.
func NearestExtendCut(budget *proofbound.WorkBudget, fragments []sketch.BoundaryEdge,
	source sketch.Entity, t0, t1 float64, atStart bool) (float64, sketch.BoundaryEdge, bool, error) {
	old := t1
	if atStart {
		old = t0
	}
	forward := t1 > t0
	if atStart {
		forward = !forward
	}
	nearest := 0.0
	found := false
	for _, edge := range fragments {
		if err := budget.Step(); err != nil {
			return 0, sketch.BoundaryEdge{}, false, err
		}
		if edge.Entity != source || !edge.Partial {
			continue
		}
		for _, candidate := range []float64{edge.TStart, edge.TEnd} {
			if candidate == 0 || candidate == 1 {
				continue
			}
			if forward && candidate <= old || !forward && candidate >= old {
				continue
			}
			if !found || forward && candidate < nearest || !forward && candidate > nearest {
				nearest, found = candidate, true
			}
		}
	}
	if !found {
		return 0, sketch.BoundaryEdge{}, false, nil
	}
	for _, edge := range fragments {
		if edge.Entity != source || edge.TStart != nearest && edge.TEnd != nearest {
			continue
		}
		return nearest, edge, true, nil
	}
	return nearest, sketch.BoundaryEdge{}, true, nil
}
