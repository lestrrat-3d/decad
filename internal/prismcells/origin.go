package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// Origin is buildPrismScene's own tag map value (§4.1's "tagged, in
// a side map, with its origin"): which operand a created scene entity traces
// to, and which of that operand's loops it came from — -1 for Outer, else the
// index into ProfileRecord.Holes. Union's resolution never reads it — every
// operand it admits is hole-free (G6), so only "which operand" would ever
// vary. Cut/Intersect's clean-nesting match (§4.2) is what needs the loop half
// too: proving §4.2's nesting relation is a pure data comparison against this
// map, never a geometric test. AuthoredReversed is the crossing sub-case's own
// addition (membership.go): whether this operand's own recorded
// walk of the entity runs backwards relative to the entity's own natural
// parameterization — the fixed fact buildPrismScene computes once at creation
// time that Classify later compares against a returned edge's own
// Reversed flag, never a geometric test. For an operand B whose relative map
// is a reflection, it is read off the re-wound record buildPrismScene builds
// B from (docs/general-boolean-design.md §3 A4), whose loops keep the
// "outer CCW, holes CW" convention the comparison assumes.
//
// Region names which of the operand's regions the loop belongs to: 0 for a
// prism, the lump index for a prism group (docs/general-boolean-design.md §3
// A5), whose regions enter one scene together.
type Origin struct {
	IsB              bool
	Region           int
	Hole             int
	AuthoredReversed bool
}

// LoopEntitySet is the tag map's per-loop view: the set of entities
// buildPrismScene created for one operand's one loop (Outer at hole = -1,
// else Holes[hole]), read for the structural match (§4.2) alone.
func LoopEntitySet(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, isB bool, hole int) (map[sketch.Entity]struct{}, error) {
	out := map[sketch.Entity]struct{}{}
	for e, origin := range tags {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if origin.IsB == isB && origin.Hole == hole {
			out[e] = struct{}{}
		}
	}
	return out, nil
}

// RegionLoopEntitySet is LoopEntitySet narrowed to one region of the
// operand: the entities of that region's outer (hole = -1) or of one of its
// holes. A5's clean-nesting match reads one set per region of a prism group.
func RegionLoopEntitySet(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, isB bool, region, hole int) (map[sketch.Entity]struct{}, error) {
	out := map[sketch.Entity]struct{}{}
	for e, origin := range tags {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if origin.IsB == isB && origin.Region == region && origin.Hole == hole {
			out[e] = struct{}{}
		}
	}
	return out, nil
}

// LoopMatchesOrigin reports whether a candidate boundary loop
// structurally reproduces the wanted entity set (§4.2's "clean" sub-case):
// every edge is Whole (Partial == false — the arrangement cut nothing), the
// edge count equals the wanted set's size, and the edges' Entity values equal
// the wanted set. Comparing Entity by interface identity is the same
// discipline buildPrismScene's own dedup key already uses. Deliberately NOT a
// check on edge order or starting index: a simple loop's own walk is
// determined only up to rotation, and requiring an index would make the match
// fragile without proving anything more.
func LoopMatchesOrigin(budget *proofbound.WorkBudget, edges []sketch.BoundaryEdge, want map[sketch.Entity]struct{}) (bool, error) {
	if len(edges) != len(want) {
		return false, nil
	}
	seen := make(map[sketch.Entity]struct{}, len(edges))
	for _, e := range edges {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if e.Partial {
			return false, nil
		}
		if _, ok := want[e.Entity]; !ok {
			return false, nil
		}
		if _, dup := seen[e.Entity]; dup {
			return false, nil
		}
		seen[e.Entity] = struct{}{}
	}
	return true, nil
}

// holesMatchOrigin reports whether a candidate profile's Holes
// structurally reproduce EXACTLY the wanted hole entity sets, as an unordered
// set of sets: each candidate hole matches at most one wanted hole (by
// LoopMatchesOrigin), and every wanted hole is matched by exactly one
// candidate hole. The bipartite match is small (a handful of holes at most,
// bounded by the same arrangement cap as everything else here) and needs no
// index correspondence — sketch's own Holes order is not decad's to assume.
func holesMatchOrigin(budget *proofbound.WorkBudget, holes [][]sketch.BoundaryEdge, want []map[sketch.Entity]struct{}) (bool, error) {
	matched := make([]bool, len(want))
	for _, h := range holes {
		if err := budget.Step(); err != nil {
			return false, err
		}
		found := -1
		for i, w := range want {
			if matched[i] {
				continue
			}
			ok, err := LoopMatchesOrigin(budget, h, w)
			if err != nil {
				return false, err
			}
			if ok {
				found = i
				break
			}
		}
		if found == -1 {
			return false, nil
		}
		matched[found] = true
	}
	return true, nil
}

// FindLoopMatch is §4.2's clean-nesting structural search: the unique
// s.Profiles() result whose Outer structurally reproduces wantOuter and whose
// Holes structurally reproduce EXACTLY the entity sets in wantHoles — a pure
// data comparison against decad's own tag map, never a geometric test.
// resolved=false (err always nil in that case) means no such unique profile
// exists: zero candidates or more than one (ambiguous) are both §4.4's
// "unresolved," not a refusal.
func FindLoopMatch(budget *proofbound.WorkBudget, profiles []*sketch.Profile, wantOuter map[sketch.Entity]struct{}, wantHoles []map[sketch.Entity]struct{}) (*sketch.Profile, bool, error) {
	var found *sketch.Profile
	for _, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		outerOK, err := LoopMatchesOrigin(budget, p.Outer, wantOuter)
		if err != nil {
			return nil, false, err
		}
		if !outerOK {
			continue
		}
		if len(p.Holes) != len(wantHoles) {
			continue
		}
		holesOK, err := holesMatchOrigin(budget, p.Holes, wantHoles)
		if err != nil {
			return nil, false, err
		}
		if !holesOK {
			continue
		}
		if found != nil {
			return nil, false, nil // ambiguous: more than one candidate matches
		}
		found = p
	}
	if found == nil {
		return nil, false, nil
	}
	return found, true, nil
}
