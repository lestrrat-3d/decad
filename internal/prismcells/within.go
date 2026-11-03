package prismcells

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// SectionWithin reports whether every point of the inner regions lies in the
// outer regions, both exact sections in one plane's coordinates. It arranges
// them in one private Sketch scene, reads each cell's membership from Sketch's
// containment verdicts against the declared loops, and lets the edge-direction
// reading only falsify them, as the section history's combine does
// (agent-docs/prism-boolean-design.md §4.1-§4.2). Any arrangement it cannot
// certify answers false, never true.
//
// An inner region whose outer loop is one whole circle exactly equal to the
// outer loop of a hole-free outer region lies in it at once: the inner region
// lies in the closed disc its outer loop bounds, and that disc is the outer
// region. Closed containment is all a caller needs, since a body inside
// another's closed set shares no interior point with whatever misses that
// set's interior. Sketch is not asked to arrange two coincident circles.
func SectionWithin(ctx context.Context, budget *proofbound.WorkBudget, inner, outer []momentinput.Profile) (bool, error) {
	if identicalCircles(inner, outer) {
		return true, ctx.Err()
	}
	return sectionCells(ctx, budget, inner, outer, true, func(inA, inB bool) bool { return !inA || inB })
}

// SectionWithinUnion is SectionWithin for outer regions that may overlap one
// another, such as a hole and a cut tool's section whose union covers the
// inner region. Membership in the outer family is the union of its regions,
// read from Sketch's containment verdicts as SectionWithin reads it. The
// edge-direction reading of the outer family is skipped: it takes each outer
// edge to bound the family, which holds only where the regions are disjoint,
// and it only ever falsified a verdict, so skipping it admits nothing the
// verdicts do not.
func SectionWithinUnion(ctx context.Context, budget *proofbound.WorkBudget, inner, outer []momentinput.Profile) (bool, error) {
	return sectionCells(ctx, budget, inner, outer, false, func(inA, inB bool) bool { return !inA || inB })
}

// SectionsApart reports whether two exact sections in one plane's
// coordinates share no interior point: no cell of their arrangement lies in
// both. It certifies as SectionWithin does.
func SectionsApart(ctx context.Context, budget *proofbound.WorkBudget, a, b []momentinput.Profile) (bool, error) {
	return sectionCells(ctx, budget, a, b, true, func(inA, inB bool) bool { return !inA || !inB })
}

// sectionCells arranges a and b in one scene and reports whether every cell's
// membership pair satisfies keep.
func sectionCells(ctx context.Context, budget *proofbound.WorkBudget, inner, outer []momentinput.Profile,
	readB bool, keep func(inA, inB bool) bool) (bool, error) {
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
	if err != nil || !read && readB {
		return false, err
	}
	for i := range profiles {
		if !read {
			break
		}
		if reading.KnownA[i] && reading.A[i] != inA[i] || readB && reading.KnownB[i] && reading.B[i] != inB[i] {
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

// identicalCircles reports whether every inner region's outer loop is one
// whole circle equal, centre and radius bit for bit, to the outer loop of a
// hole-free outer region.
func identicalCircles(inner, outer []momentinput.Profile) bool {
	if len(inner) == 0 {
		return false
	}
	for _, in := range inner {
		c, ok := wholeCircleLoop(in.Outer)
		if !ok {
			return false
		}
		found := false
		for _, out := range outer {
			if len(out.Holes) != 0 {
				continue
			}
			if o, ok := wholeCircleLoop(out.Outer); ok && o.Center == c.Center && o.Radius == c.Radius {
				found = true
				break
			}
		}
		if !found {
			return false
		}
	}
	return true
}

// wholeCircleLoop returns loop's circle when the loop is one whole circle.
func wholeCircleLoop(loop sectionrecord.LoopRecord) (sectionrecord.CircleSeg, bool) {
	if len(loop.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := loop.Segments[0].(sectionrecord.CircleSeg)
	whole := c.CCW && c.TStart == 0 && c.TEnd == 1 || !c.CCW && c.TStart == 1 && c.TEnd == 0
	return c, ok && whole
}
