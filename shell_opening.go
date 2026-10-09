package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file is the prism side opening of docs/shell-opening-design.md: Shell
// on a straight prism that removes one proper connected run R of its outer
// side faces, with or without its caps. The kept walks K are offset as an open
// chain (shell_chain.go) whose two ends close by Table RO's rim, the removed
// walk's own carrier cut by the offset. Three regions follow (§3): the wall
// section W, the cavity section C and the cap slabs' region (P inward, the
// outer region O outward), and each faces modify §5's audit and an exact area
// identity before shell_opening_brep.go builds the body from them. Every walk
// of the section must be a line, along a section axis or oblique, or a
// circular arc.

// sideOpeningSection is the three regions of one side opening (§3) and what
// the record build reads beside them.
type sideOpeningSection struct {
	// wall is W, the wall section over the cavity's height.
	wall profileRecord
	// cavity is the region each kept cap's interface exposes: C inward (K'
	// then R'), the receiver's section P outward.
	cavity profileRecord
	// caps is the region of a kept cap's slab: P inward, O outward (K' then
	// R').
	caps profileRecord
	// corners lists each end vertex v the record must mark a vertex at both
	// cavity levels (§4.3): where the rim runs backward along the removed
	// walk's carrier, v lies inside the cavity's walk there; where that
	// walk is oblique, the rim column's split at q must meet a split of the
	// kept wall's edge through v.
	corners []Point2
	// delta is the offset's section displacement: three times
	// offset2d.ChainReach, zero where every join and cut encloses to its held
	// float.
	delta float64
	// cutGap bounds how far a circular rim's record names its cut from the
	// cut q the open chain holds (arcCutGap): the rim, R and R' walk r's own
	// parameterisation to a float parameter whose point need not be q. Only
	// a result that publishes these regions as its section charges it
	// (evalSideOpeningPrism); the engine writes every face of a kept cap's
	// stack from its canonical vertices and carriers instead (§4.4).
	cutGap float64
}

// classifyRemovedFaces sorts a prism shell's removed faces (§5, stage 2):
// the caps by their roles, and every other face by the side(0,j) roles of the
// receiver it carries, j being the recorded outer-loop segment the face
// sweeps. It reports whether each cap is removed and the removed outer-loop
// segments. A face that is neither a cap nor a side face of the outer loop —
// a hole loop's wall — is SO6, ErrUnsupported.
func classifyRemovedFaces(b *Body, caps prismCaps, removed []*Face) (bool, bool, map[int]struct{}, error) {
	var start, end bool
	sides := map[int]struct{}{}
	for _, f := range removed {
		switch {
		case caps.start != nil && f == caps.start:
			start = true
			continue
		case caps.end != nil && f == caps.end:
			end = true
			continue
		}
		named := false
		for _, o := range f.origins {
			var li, j int
			if o.producer != b.origin.producer {
				continue
			}
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n != 2 {
				continue
			}
			if li != 0 {
				return false, false, nil, fmt.Errorf(`%w: a side opening removes faces of the section's outer loop only, and this face lines a hole (modify-reach SX8, shell-opening SO6)`, ErrUnsupported)
			}
			sides[j] = struct{}{}
			named = true
		}
		if !named {
			return false, false, nil, fmt.Errorf(`%w: a removed face is neither a cap nor a side face of this prism (shell-opening SO6)`, ErrUnsupported)
		}
	}
	return start, end, sides, nil
}

// sideOpeningHeight is SO3's height half (§5, stage 3): inward, each kept cap
// eats t of the sweep h, so the cavity's height h − k·t must stay positive,
// read with the section limit's scale-relative rounding tolerance. Outward,
// or with both caps removed, no height limit applies.
func sideOpeningHeight(pp prismPayload, keptCaps int, s float64, t units.Value, tmm float64) error {
	if s < 0 || keptCaps == 0 {
		return nil
	}
	h := pp.z1 - pp.z0
	if maxT := h/float64(keptCaps) - shellTol*math.Max(1, h); tmm >= maxT {
		return fmt.Errorf(`%w: the shell thickness %s leaves no cavity between the %d kept cap(s) of a %s sweep (the accepted maximum is %s; shell-opening SO3)`,
			ErrDegenerate, t, keptCaps, units.Millimeters(h), units.Millimeters(math.Max(maxT, 0)))
	}
	return nil
}

// sideOpeningRegions builds and audits the three regions of a side opening
// (§3, §4.7 steps 1–3) in §5's gate order: SO6 (a holed section, then the
// run rule), the walk kinds this build takes (SO5), SO3's height half, the
// open chain's offset (S11a per walk, SO1, SO2 and SO4 per end), each cut on
// a removed arc placed in the arc's own parameterisation (SO5 where no range
// of its record names it), modify §5's audit of W and of C (O outward) — S8,
// where a C with no area is SO3, S11b and S9 — and the exact area identity,
// whose failure is SO5.
// keptCaps is the number of caps the shell keeps. pp's profile, z0 and z1 are
// read; route S (brep_shell.go) hands it a through-cut record's prism A and
// takes C as A ⊖ t's section (docs/modify-general-design.md §3.2).
func sideOpeningRegions(budget *proofbound.WorkBudget, pp prismPayload, sides map[int]struct{}, keptCaps int, s float64, t units.Value, tmm, tDelta float64) (sideOpeningSection, error) {
	if len(pp.profile.Holes) > 0 {
		return sideOpeningSection{}, fmt.Errorf(`%w: a side opening of a prism whose section holds %d hole(s) is not supported (modify-reach SX8, shell-opening SO6)`, ErrUnsupported, len(pp.profile.Holes))
	}
	loops, err := profileCornerLoopsBudget(budget, pp.profile)
	if err != nil {
		return sideOpeningSection{}, err
	}
	walks := loops[0].walks
	run, err := offset2d.PrismRemovedRun(walks, sides)
	if err != nil {
		return sideOpeningSection{}, err
	}
	if err := offset2d.RequireSideOpeningWalks(walks); err != nil {
		return sideOpeningSection{}, err
	}
	if err := sideOpeningHeight(pp, keptCaps, s, t, tmm); err != nil {
		return sideOpeningSection{}, err
	}

	// K runs from the walk after R to the walk before it: from vA, where R
	// ends, to vB, where R starts.
	n := len(walks)
	rFirst, rLast := walks[run[0]], walks[run[len(run)-1]]
	var chain []survey2d.SideWalk
	for k := 1; k < n-len(run)+1; k++ {
		chain = append(chain, walks[(run[len(run)-1]+k)%n])
	}
	segs := pp.profile.Outer.Segments
	var kept []curveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}
	first, last := chain[0], chain[len(chain)-1]
	start := offset2d.OpenEnd{Removed: rLast}
	end := offset2d.OpenEnd{Removed: rFirst}
	off, err := offset2d.OffsetOpenChain(budget, chain, offset2d.Curve{}, start, end, s, tmm, shellTol)
	if err != nil {
		return sideOpeningSection{}, offset2d.InLoop(err, 0)
	}
	vA := Point2{U: first.StartU, V: first.StartV}
	vB := Point2{U: last.EndU, V: last.EndV}
	qA, qB := off.QStart, off.QEnd

	// A cut runs forward into r's span from v, or backward along the
	// carrier's extension behind v (Table RO's reflex row inward);
	// offset2d.OffsetOpenChain has already read both corners.
	forwardA, err := offset2d.OpeningForward(first, rLast, false, s, shellTol)
	if err != nil {
		return sideOpeningSection{}, err
	}
	forwardB, err := offset2d.OpeningForward(last, rFirst, true, s, shellTol)
	if err != nil {
		return sideOpeningSection{}, err
	}
	sB, err := offset2d.CutStation(rFirst, segs, qB, forwardB, true)
	if err != nil {
		return sideOpeningSection{}, err
	}
	sA, err := offset2d.CutStation(rLast, segs, qA, forwardA, false)
	if err != nil {
		return sideOpeningSection{}, err
	}

	// W: the rims run v → q where the loop leaves K for K' (vB inward, vA
	// outward) and q → v where it returns. On a circular r each rim is a
	// parameter range of r's own record (removedPiece), as R and R' state
	// their pieces, so all of them key r's circle (§4.2).
	inward := s > 0
	atEnd, atStart, gap, err := offset2d.SideOpeningRims(walks, run, segs, last, first, sB, sA, off.Ends, s, inward)
	if err != nil {
		return sideOpeningSection{}, err
	}
	wallLoop, err := offset2d.OpenChainWallLoop(budget, kept, off.Segs, atEnd, atStart, inward)
	if err != nil {
		return sideOpeningSection{}, err
	}

	// R' is the removed run re-cut at both ends: its first walk starts at qB,
	// its last ends at qA, and every walk between is verbatim.
	recut, err := offset2d.RemovedPieces(walks, run, segs, sB, sA, forwardB, forwardA, true)
	if err != nil {
		return sideOpeningSection{}, err
	}
	offRegion := profileRecord{Outer: loopRecord{Segments: append(append([]curveSegment(nil), off.Segs...), recut...)}}
	// P as the record states it: the receiver's own section, or, where an
	// oblique or circular end walk takes a forward cut, K then R split at
	// that cut.
	section := pp.profile
	if (offset2d.SplitAtCuts(rFirst) && forwardB) || (offset2d.SplitAtCuts(rLast) && forwardA) {
		split, err := offset2d.RemovedPieces(walks, run, segs, sB, sA, forwardB, forwardA, false)
		if err != nil {
			return sideOpeningSection{}, err
		}
		section = profileRecord{Outer: loopRecord{Segments: append(append([]curveSegment(nil), kept...), split...)}}
	}
	sec := sideOpeningSection{wall: profileRecord{Outer: wallLoop}, cavity: offRegion, caps: section}
	if !inward {
		sec.cavity, sec.caps = section, offRegion
	}

	// §4.7 step 2: modify §5's audit of W and of the offset region (C inward,
	// O outward); together they hold every pair of a new segment with another.
	for _, region := range []profileRecord{sec.wall, offRegion} {
		if err := auditOffsetSectionBudget(budget, pp.profile, region); err != nil {
			return sideOpeningSection{}, err
		}
	}
	// §4.7 step 3: the regions tile exactly — area(P) = area(W) + area(C)
	// inward, area(O) = area(W) + area(P) outward — read in exact rational
	// arithmetic over the line walks.
	if err := offset2d.RequireAreaIdentity(sec.caps, sec.wall, sec.cavity); err != nil {
		return sideOpeningSection{}, err
	}

	// A rim cut backward along r's carrier leaves v strictly inside the
	// cavity's walk there (Table RO's reflex row). On an oblique r the rim
	// column v → q is one swept face over every slab, split at q where the
	// cavity begins: v is marked at the same levels, so the kept wall's edge
	// through v splits where the column's other side line does, and the two
	// faces' edges pair piece for piece. A circular r's rim column is marked
	// alike.
	if !forwardB || offset2d.SplitAtCuts(rFirst) {
		sec.corners = append(sec.corners, vB)
	}
	if !forwardA || offset2d.SplitAtCuts(rLast) {
		sec.corners = append(sec.corners, vA)
	}
	sec.delta, err = offset2d.ChainSectionDelta(budget, chain, offset2d.MirrorLine{}, off.Ends, s, tmm, tDelta, shellTol)
	if err != nil {
		return sideOpeningSection{}, err
	}
	sec.cutGap = gap
	return sec, nil
}

// shellSideOpening is Body.Shell's side opening on a hole-free straight
// prism (docs/shell-opening-design.md): sides names the removed outer-loop
// segments, and removedStart/removedEnd the removed caps. The regions are
// built and audited first (sideOpeningRegions); both caps removed records W
// as a prism over the receiver's sweep (BO1), and otherwise the slabs are
// stated as a brepPayload (BO2, shell_opening_brep.go).
func (b *Body) shellSideOpening(ctx context.Context, pp prismPayload, removedStart, removedEnd bool, sides map[int]struct{}, s float64, t units.Value, tmm, tDelta float64) (*Body, error) {
	d := b.doc
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	keptCaps := 0
	for _, removed := range []bool{removedStart, removedEnd} {
		if !removed {
			keptCaps++
		}
	}
	sec, err := sideOpeningRegions(budget, pp, sides, keptCaps, s, t, tmm, tDelta)
	if err != nil {
		return nil, shellCancelCause(err)
	}
	ref := d.nextProducerID()
	var body *Body
	if keptCaps == 0 {
		body, err = evalSideOpeningPrism(ctx, d, ref, pp, sec)
	} else {
		var bp brepPayload
		bp, err = sideOpeningBrep(ctx, budget, pp, sec, removedStart, removedEnd, s, tmm, tDelta)
		if err == nil {
			body, err = evalBrepContext(ctx, d, ref, bp)
		}
	}
	if err != nil {
		return nil, shellCancelCause(err)
	}
	return commitModifyResult(ctx, b, body)
}
