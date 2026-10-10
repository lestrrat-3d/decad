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

// EnclosingCutMatch is the pair of sketch cells proving that a cut tool
// encloses some existing target holes. Outside is the target material beyond
// the tool; Inside is the material between the tool and those holes.
type EnclosingCutMatch struct {
	Outside, Inside *sketch.Profile
	OutsideHoles    []int
	EnclosedHoles   []int
}

// MatchEnclosingCut reads whole loops from the arrangement. It never infers
// nesting from coordinates: the outside cell must hold the tool as a hole, and
// the inside cell must hold exactly the target holes absent from that cell.
func MatchEnclosingCut(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, targetHoleCount int) (EnclosingCutMatch, bool, error) {
	targetOuter, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return EnclosingCutMatch{}, false, err
	}
	toolOuter, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return EnclosingCutMatch{}, false, err
	}
	targetHoles := make([]map[sketch.Entity]struct{}, targetHoleCount)
	for i := range targetHoles {
		targetHoles[i], err = LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return EnclosingCutMatch{}, false, err
		}
	}
	var result EnclosingCutMatch
	var outsideSet []bool
	for _, p := range profiles {
		if err := budget.Step(); err != nil {
			return EnclosingCutMatch{}, false, err
		}
		outer, err := LoopMatchesOrigin(budget, p.Outer, targetOuter)
		if err != nil {
			return EnclosingCutMatch{}, false, err
		}
		if !outer {
			continue
		}
		set, tool, matched, err := matchCutCellHoles(budget, p.Holes, targetHoles, toolOuter, true)
		if err != nil {
			return EnclosingCutMatch{}, false, err
		}
		if !matched || !tool || allMatched(set) {
			continue
		}
		if result.Outside != nil {
			return EnclosingCutMatch{}, false, nil
		}
		if !p.Valid {
			return EnclosingCutMatch{}, false, InvalidRegionError("cut")
		}
		result.Outside, outsideSet = p, set
	}
	if result.Outside == nil {
		return EnclosingCutMatch{}, false, nil
	}
	for _, p := range profiles {
		if err := budget.Step(); err != nil {
			return EnclosingCutMatch{}, false, err
		}
		outer, err := LoopMatchesOrigin(budget, p.Outer, toolOuter)
		if err != nil {
			return EnclosingCutMatch{}, false, err
		}
		if !outer {
			continue
		}
		set, _, matched, err := matchCutCellHoles(budget, p.Holes, targetHoles, nil, false)
		if err != nil {
			return EnclosingCutMatch{}, false, err
		}
		if !matched || !complements(outsideSet, set) {
			continue
		}
		if result.Inside != nil {
			return EnclosingCutMatch{}, false, nil
		}
		if !p.Valid {
			return EnclosingCutMatch{}, false, InvalidRegionError("cut")
		}
		result.Inside = p
	}
	if result.Inside == nil {
		return EnclosingCutMatch{}, false, nil
	}
	for i, outside := range outsideSet {
		if outside {
			result.OutsideHoles = append(result.OutsideHoles, i)
		} else {
			result.EnclosedHoles = append(result.EnclosedHoles, i)
		}
	}
	return result, true, nil
}

func matchCutCellHoles(budget *proofbound.WorkBudget, holes [][]sketch.BoundaryEdge,
	target []map[sketch.Entity]struct{}, tool map[sketch.Entity]struct{}, allowTool bool) ([]bool, bool, bool, error) {
	seen := make([]bool, len(target))
	toolSeen := false
	for _, hole := range holes {
		if err := budget.Step(); err != nil {
			return nil, false, false, err
		}
		if allowTool && !toolSeen {
			isTool, err := LoopMatchesOrigin(budget, hole, tool)
			if err != nil {
				return nil, false, false, err
			}
			if isTool {
				toolSeen = true
				continue
			}
		}
		found := false
		for i, want := range target {
			if seen[i] {
				continue
			}
			match, err := LoopMatchesOrigin(budget, hole, want)
			if err != nil {
				return nil, false, false, err
			}
			if match {
				seen[i], found = true, true
				break
			}
		}
		if !found {
			return nil, false, false, nil
		}
	}
	return seen, toolSeen, true, nil
}

func allMatched(found []bool) bool {
	for _, yes := range found {
		if !yes {
			return false
		}
	}
	return true
}

func complements(a, b []bool) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] == b[i] {
			return false
		}
	}
	return true
}

// MatchCutNoOp proves that a tool lies wholly inside one existing target
// hole. The target material cell is unchanged, while sketch reports the
// annulus between that hole and the tool as a separate valid cell.
func MatchCutNoOp(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile, targetHoleCount int) (bool, error) {
	if targetHoleCount == 0 {
		return false, nil
	}
	targetOuter, err := LoopEntitySet(budget, tags, false, -1)
	if err != nil {
		return false, err
	}
	toolOuter, err := LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return false, err
	}
	targetHoles := make([]map[sketch.Entity]struct{}, targetHoleCount)
	for i := range targetHoles {
		targetHoles[i], err = LoopEntitySet(budget, tags, false, i)
		if err != nil {
			return false, err
		}
	}
	material, found, err := FindLoopMatch(budget, profiles, targetOuter, targetHoles)
	if err != nil || !found {
		return false, err
	}
	if !material.Valid {
		return false, InvalidRegionError("cut")
	}
	annuli := 0
	for _, hole := range targetHoles {
		annulus, found, err := FindLoopMatch(budget, profiles, hole,
			[]map[sketch.Entity]struct{}{toolOuter})
		if err != nil {
			return false, err
		}
		if !found {
			continue
		}
		if !annulus.Valid {
			return false, InvalidRegionError("cut")
		}
		annuli++
	}
	return annuli == 1, nil
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
