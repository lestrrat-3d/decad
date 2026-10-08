package prismcells

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
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
	for i := range receiverHoles {
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

// ResolveSplitCells keeps the arranged cells on the target's material side.
// A cell reproducing the target or fewer than two selected cells is no split.
func ResolveSplitCells(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, targetHoles int) ([]*sketch.Profile, error) {
	unchanged, err := SplitUnchangedTargetCell(budget, tags, profiles, targetHoles)
	if err != nil {
		return nil, err
	}
	if unchanged {
		return nil, fmt.Errorf(`%w: the tool separates no part of the target`, decaderr.ErrDegenerate)
	}
	matterTarget, err := ClassifySplit(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	selected, err := Select(budget, profiles, matterTarget, make([]bool, len(profiles)),
		func(a, _ bool) bool { return a })
	if err != nil {
		return nil, err
	}
	if len(selected) < 2 {
		return nil, fmt.Errorf(`%w: the tool separates no part of the target`, decaderr.ErrDegenerate)
	}
	return selected, nil
}

// SplitCellCutDelta charges every arranged boundary edge in one selected
// cell, keeping the publication's outer-then-hole order.
func SplitCellCutDelta(budget *proofbound.WorkBudget, cell *sketch.Profile) (float64, error) {
	cutDelta := 0.0
	for _, loop := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
		for _, edge := range loop {
			if err := budget.Step(); err != nil {
				return 0, err
			}
			seg, err := sketchrecord.RecordEdge(edge)
			if err != nil {
				return 0, err
			}
			delta, err := CutDelta(edge, seg)
			if err != nil {
				return 0, err
			}
			cutDelta = math.Max(cutDelta, delta)
		}
	}
	return cutDelta, nil
}

// ResolveExtendCut reads the receiver carrier's nearest published cut from
// profile and chain fragments, in that order. The chain publication is called
// only after the profile fragments and source entity are accepted.
func ResolveExtendCut(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, chains func() ([]*sketch.Chain, error),
	t0, t1 float64, atStart bool) (float64, sketch.BoundaryEdge, bool, error) {
	source := ExtendSource(tags)
	if source == nil {
		return 0, sketch.BoundaryEdge{}, false,
			fmt.Errorf(`%w: the extended carrier has no scene entity`, decaderr.ErrUnsupported)
	}
	fragments, err := ExtendProfileFragments(profiles)
	if err != nil {
		return 0, sketch.BoundaryEdge{}, false, err
	}
	chainList, err := chains()
	if err != nil {
		return 0, sketch.BoundaryEdge{}, false, err
	}
	if err := budget.Err(); err != nil {
		return 0, sketch.BoundaryEdge{}, false, err
	}
	fragments, err = AppendExtendChainFragments(fragments, chainList)
	if err != nil {
		return 0, sketch.BoundaryEdge{}, false, err
	}
	return NearestExtendCut(budget, fragments, source, t0, t1, atStart)
}

// ResolveTrimWalks selects the receiver fragments on the requested side and
// chains them in the arrangement's order. It refuses unresolved or coincident
// boundaries before the caller records any fragment.
func ResolveTrimWalks(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, receiverHoles int, keepInside bool) ([][]sketch.BoundaryEdge, error) {
	insideTool, noCrossing, err := TrimNoCrossingSide(budget, tags, profiles, receiverHoles)
	if err != nil {
		return nil, err
	}
	if noCrossing {
		if insideTool == keepInside {
			return nil, fmt.Errorf(`%w: the tool separates no fragment of the receiver; every fragment is kept`, decaderr.ErrDegenerate)
		}
		return nil, fmt.Errorf(`%w: the tool separates no fragment of the receiver; none is kept`, decaderr.ErrDegenerate)
	}

	coincident, readable, err := CoincidentEdges(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	if !readable || len(coincident.Spans) > 0 {
		return nil, fmt.Errorf(`%w: a receiver boundary fragment coincides with the tool's own boundary`, decaderr.ErrUnsupported)
	}

	matterRcv, matterTool, resolved, err := Classify(budget, tags, profiles)
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, fmt.Errorf(`%w: the receiver and tool's arrangement is not one this evaluator's crossing classifier resolves`, decaderr.ErrUnsupported)
	}

	survivors, total, err := SurvivingFragments(budget, tags, matterRcv, matterTool, profiles, keepInside)
	if err != nil {
		return nil, err
	}
	if len(survivors) == 0 {
		return nil, fmt.Errorf(`%w: the tool separates no fragment of the receiver; none is kept`, decaderr.ErrDegenerate)
	}
	if len(survivors) == total {
		return nil, fmt.Errorf(`%w: the tool separates no fragment of the receiver; every fragment is kept`, decaderr.ErrDegenerate)
	}

	walks, resolved, err := ChainSurvivorWalks(budget, survivors)
	if err != nil {
		return nil, err
	}
	if !resolved {
		return nil, fmt.Errorf(`%w: the surviving fragments do not chain into open walks this evaluator resolves`, decaderr.ErrUnsupported)
	}
	return walks, nil
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
