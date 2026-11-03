package prismcells

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SectionWithin reports whether every point of the inner regions lies in the
// outer regions, both exact sections in one plane's coordinates. It arranges
// them in one private Sketch scene, reads each cell's membership from Sketch's
// containment verdicts against the declared loops, and lets the edge-direction
// reading only falsify them, as the section history's combine does
// (agent-docs/prism-boolean-design.md §4.1-§4.2). Any arrangement it cannot
// certify answers false, never true.
func SectionWithin(ctx context.Context, budget *proofbound.WorkBudget, inner, outer []momentinput.Profile) (bool, error) {
	return sectionCells(ctx, budget, inner, outer, func(inA, inB bool) bool { return !inA || inB })
}

// SectionsApart reports whether two exact sections in one plane's
// coordinates share no interior point: no cell of their arrangement lies in
// both. It certifies as SectionWithin does.
func SectionsApart(ctx context.Context, budget *proofbound.WorkBudget, a, b []momentinput.Profile) (bool, error) {
	return sectionCells(ctx, budget, a, b, func(inA, inB bool) bool { return !inA || !inB })
}

// sectionCells arranges a and b in one scene and reports whether every cell's
// membership pair satisfies keep.
func sectionCells(ctx context.Context, budget *proofbound.WorkBudget, inner, outer []momentinput.Profile,
	keep func(inA, inB bool) bool) (bool, error) {
	if len(inner) == 0 || len(outer) == 0 {
		return false, nil
	}
	if _, within, err := RegionsWithinWorkCap(budget, slices.Concat(inner, outer)...); err != nil || !within {
		return false, err
	}
	s, tags, delta, err := BuildProfileScene(budget, inner, outer, &Reexpression{Identity: true})
	if err != nil {
		// A scene Sketch refuses is an arrangement this check cannot certify;
		// only cancellation is the caller's error.
		return false, ctx.Err()
	}
	if delta.A != 0 || delta.B != 0 {
		return false, nil
	}
	loops, declared, err := DeclareOperandLoops(budget, s, tags)
	if err != nil || !declared {
		return false, err
	}
	profiles, err := CellProfiles(ctx, budget, s)
	if err != nil {
		return false, err
	}
	if ok, err := delta.ChargeCrossings(budget, tags, profiles, 0, 0, 0); err != nil || !ok || delta.Crossing != 0 {
		return false, err
	}
	inA, inB, resolved, err := ContainedMembership(budget, profiles, loops)
	if err != nil || !resolved {
		return false, err
	}
	if walked, err := LoopEntitiesWalked(budget, tags, profiles); err != nil || !walked {
		return false, err
	}
	reading, read, err := ClassifyNested(budget, tags, profiles)
	if err != nil || !read {
		return false, err
	}
	for i := range profiles {
		if reading.KnownA[i] && reading.A[i] != inA[i] || reading.KnownB[i] && reading.B[i] != inB[i] {
			return false, nil
		}
	}
	for i := range profiles {
		if !keep(inA[i], inB[i]) {
			return false, nil
		}
	}
	return true, nil
}
