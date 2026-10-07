package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// MatchCut finds the target cell with the tool's outer loop as one new hole.
func MatchCut(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile, targetHoleCount int) (*sketch.Profile, bool, error) {
	targetOuter, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, false, err
	}
	wantHoles := make([]map[sketch.Entity]struct{}, 0, targetHoleCount+1)
	for i := range targetHoleCount {
		hs, err := LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return nil, false, err
		}
		wantHoles = append(wantHoles, hs)
	}
	toolOuter, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, false, err
	}
	wantHoles = append(wantHoles, toolOuter) // the tool's own solid, as one new hole

	match, resolved, err := FindLoopMatch(budget, profiles, targetOuter, wantHoles)
	if err != nil {
		return nil, false, err
	}
	if !resolved {
		return nil, false, nil
	}
	if !match.Valid {
		// RB1, matching the Union path's own behaviour: a candidate region
		// the result depends on reports an invalid arrangement. Cut's matched
		// profile is both its nesting proof and its result, so this one check
		// covers both claims.
		return nil, false, InvalidRegionError("cut")
	}
	return match, true, nil
}

// MatchIntersect proves clean nesting in one direction and finds the nested
// operand's own result cell. Both cells must be valid.
func MatchIntersect(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile, bHoleCount int) (*sketch.Profile, bool, bool, error) {
	aOuter, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return nil, false, false, err
	}
	bOuter, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return nil, false, false, err
	}

	// The one-hole arm keeps A hole-free and needs only B-inside-A. The
	// hole-free arm searches in both directions as before.
	//
	// The proof cell carries the whole weight of the nesting claim, so its own
	// validity is checked exactly like the result cell's below: a cell sketch
	// reports degenerate proves nothing about which operand encloses which,
	// and reading a nesting off it would bless an arrangement sketch has
	// already disowned. That is RB1's "a candidate region the result depends
	// on", and this path depends on two.
	proofBNested, bNested, err := FindLoopMatch(budget, profiles, aOuter, []map[sketch.Entity]struct{}{bOuter})
	if err != nil {
		return nil, false, false, err
	}
	if bNested && !proofBNested.Valid {
		return nil, false, false, InvalidRegionError("intersect")
	}
	aNested := false
	if bHoleCount == 0 {
		proofANested, matched, err := FindLoopMatch(budget, profiles, bOuter, []map[sketch.Entity]struct{}{aOuter})
		if err != nil {
			return nil, false, false, err
		}
		if matched && !proofANested.Valid {
			return nil, false, false, InvalidRegionError("intersect")
		}
		aNested = matched
	}
	if bNested == aNested {
		// Both directions match (should not occur for a genuine pair) or
		// neither does (a disjoint or crossing pair, or any other topology
		// this increment does not cover): unresolved, §4.4.
		return nil, false, false, nil
	}

	// The nested operand's own region is a SEPARATE s.Profiles() candidate
	// from the nesting proof above. B may carry one hole in the new arm.
	wantOuter, nested := aOuter, false
	var wantHoles []map[sketch.Entity]struct{}
	if bNested {
		wantOuter, nested = bOuter, true
		for i := range bHoleCount {
			hole, err := LoopEntitySet(budget, tags, true, i)
			if err != nil {
				return nil, false, false, err
			}
			wantHoles = append(wantHoles, hole)
		}
	}
	result, resultResolved, err := FindLoopMatch(budget, profiles, wantOuter, wantHoles)
	if err != nil {
		return nil, false, false, err
	}
	if !resultResolved {
		return nil, false, false, nil
	}
	if !result.Valid {
		// RB1, matching the Union/Cut paths' own behaviour.
		return nil, false, false, InvalidRegionError("intersect")
	}
	return result, nested, true, nil
}
