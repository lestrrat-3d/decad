package prismshell

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// SideOpeningInput is the record and held thickness used to build one side opening.
type SideOpeningInput struct {
	Profile        momentinput.Profile
	Height         float64
	Sides          map[int]struct{}
	KeptCaps       int
	Sense          float64
	Thickness      units.Value
	HeldThickness  float64
	ThicknessDelta float64
	Tolerance      float64
}

// Section is the three regions of one side opening (§3) and what
// the record build reads beside them.
type Section struct {
	// Wall is W, the wall section over the cavity's height.
	Wall momentinput.Profile
	// Cavity is the region each kept cap's interface exposes: C inward (K'
	// then R'), the receiver's section P outward.
	Cavity momentinput.Profile
	// Caps is the region of a kept cap's slab: P inward, O outward (K' then
	// R').
	Caps momentinput.Profile
	// Corners lists each end vertex v the record must mark a vertex at both
	// cavity levels (§4.3): where the rim runs backward along the removed
	// walk's carrier, v lies inside the cavity's walk there; where that
	// walk is oblique, the rim column's split at q must meet a split of the
	// kept wall's edge through v.
	Corners []sectionrecord.Point2
	// Delta is the offset's section displacement: three times
	// offset2d.ChainReach, zero where every join and cut encloses to its held
	// float.
	Delta float64
	// CutGap bounds how far a circular rim's record names its cut from the
	// cut q the open chain holds (arcCutGap): the rim, R and R' walk r's own
	// parameterisation to a float parameter whose point need not be q. Only
	// a result that publishes these regions as its section charges it
	// (evalSideOpeningPrism); the engine writes every face of a kept cap's
	// stack from its canonical vertices and carriers instead (§4.4).
	CutGap float64
}

// sideOpeningHeight is SO3's height half (§5, stage 3): inward, each kept cap
// eats t of the sweep h, so the cavity's height h − k·t must stay positive,
// read with the section limit's scale-relative rounding tolerance. Outward,
// or with both caps removed, no height limit applies.
func sideOpeningHeight(height float64, keptCaps int, s float64, t units.Value, tmm, tol float64) error {
	if s < 0 || keptCaps == 0 {
		return nil
	}
	h := height
	if maxT := h/float64(keptCaps) - tol*math.Max(1, h); tmm >= maxT {
		return fmt.Errorf(`%w: the shell thickness %s leaves no cavity between the %d kept cap(s) of a %s sweep (the accepted maximum is %s; shell-opening SO3)`,
			decaderr.ErrDegenerate, t, keptCaps, units.Millimeters(h), units.Millimeters(math.Max(maxT, 0)))
	}
	return nil
}

// SideOpeningRegions builds and audits the three regions of a side opening
// (§3, §4.7 steps 1–3) in §5's gate order: SO6 (a holed section, then the
// run rule), the walk kinds this build takes (SO5), SO3's height half, the
// open chain's offset (S11a per walk, SO1, SO2 and SO4 per end), each cut on
// a removed arc placed in the arc's own parameterisation (SO5 where no range
// of its record names it), modify §5's audit of W and of C (O outward) — S8,
// where a C with no area is SO3, S11b and S9 — and the exact area identity,
// whose failure is SO5.
// KeptCaps is the number of caps the shell keeps. Profile and Height are
// read; route S (brep_shell.go) hands it a through-cut record's prism A and
// takes C as A ⊖ t's section (docs/modify-general-design.md §3.2).
// audit runs the caller's shared modify section audit on W and the offset region.
func SideOpeningRegions(budget *proofbound.WorkBudget, in SideOpeningInput,
	audit func(*proofbound.WorkBudget, momentinput.Profile, momentinput.Profile) error) (Section, error) {
	profile, height, sides, keptCaps := in.Profile, in.Height, in.Sides, in.KeptCaps
	s, t, tmm, tDelta, tol := in.Sense, in.Thickness, in.HeldThickness, in.ThicknessDelta, in.Tolerance
	if len(profile.Holes) > 0 {
		return Section{}, fmt.Errorf(`%w: a side opening of a prism whose section holds %d hole(s) is not supported (modify-reach SX8, shell-opening SO6)`, decaderr.ErrUnsupported, len(profile.Holes))
	}
	loops, err := boundarywalk.ModifyLoopsBudget(budget, boundarywalk.Profile(profile))
	if err != nil {
		return Section{}, err
	}
	walks := loops[0]
	run, err := offset2d.PrismRemovedRun(walks, sides)
	if err != nil {
		return Section{}, err
	}
	if err := offset2d.RequireSideOpeningWalks(walks); err != nil {
		return Section{}, err
	}
	if err := sideOpeningHeight(height, keptCaps, s, t, tmm, tol); err != nil {
		return Section{}, err
	}

	// K runs from the walk after R to the walk before it: from vA, where R
	// ends, to vB, where R starts.
	n := len(walks)
	rFirst, rLast := walks[run[0]], walks[run[len(run)-1]]
	var chain []survey2d.SideWalk
	for k := 1; k < n-len(run)+1; k++ {
		chain = append(chain, walks[(run[len(run)-1]+k)%n])
	}
	segs := profile.Outer.Segments
	var kept []sectionrecord.CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}
	first, last := chain[0], chain[len(chain)-1]
	start := offset2d.OpenEnd{Removed: rLast}
	end := offset2d.OpenEnd{Removed: rFirst}
	off, err := offset2d.OffsetOpenChain(budget, chain, offset2d.Curve{}, start, end, s, tmm, tol)
	if err != nil {
		return Section{}, offset2d.InLoop(err, 0)
	}
	vA := sectionrecord.Point2{U: first.StartU, V: first.StartV}
	vB := sectionrecord.Point2{U: last.EndU, V: last.EndV}
	qA, qB := off.QStart, off.QEnd

	// A cut runs forward into r's span from v, or backward along the
	// carrier's extension behind v (Table RO's reflex row inward);
	// offset2d.OffsetOpenChain has already read both corners.
	forwardA, err := offset2d.OpeningForward(first, rLast, false, s, tol)
	if err != nil {
		return Section{}, err
	}
	forwardB, err := offset2d.OpeningForward(last, rFirst, true, s, tol)
	if err != nil {
		return Section{}, err
	}
	sB, err := offset2d.CutStation(rFirst, segs, qB, forwardB, true)
	if err != nil {
		return Section{}, err
	}
	sA, err := offset2d.CutStation(rLast, segs, qA, forwardA, false)
	if err != nil {
		return Section{}, err
	}

	// W: the rims run v → q where the loop leaves K for K' (vB inward, vA
	// outward) and q → v where it returns. On a circular r each rim is a
	// parameter range of r's own record (removedPiece), as R and R' state
	// their pieces, so all of them key r's circle (§4.2).
	inward := s > 0
	atEnd, atStart, gap, err := offset2d.SideOpeningRims(walks, run, segs, last, first, sB, sA, off.Ends, s, inward)
	if err != nil {
		return Section{}, err
	}
	wallLoop, err := offset2d.OpenChainWallLoop(budget, kept, off.Segs, atEnd, atStart, inward)
	if err != nil {
		return Section{}, err
	}

	// R' is the removed run re-cut at both ends: its first walk starts at qB,
	// its last ends at qA, and every walk between is verbatim.
	recut, err := offset2d.RemovedPieces(walks, run, segs, sB, sA, forwardB, forwardA, true)
	if err != nil {
		return Section{}, err
	}
	offRegion := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: append(append([]sectionrecord.CurveSegment(nil), off.Segs...), recut...)}}
	// P as the record states it: the receiver's own section, or, where an
	// oblique or circular end walk takes a forward cut, K then R split at
	// that cut.
	section := profile
	if (offset2d.SplitAtCuts(rFirst) && forwardB) || (offset2d.SplitAtCuts(rLast) && forwardA) {
		split, err := offset2d.RemovedPieces(walks, run, segs, sB, sA, forwardB, forwardA, false)
		if err != nil {
			return Section{}, err
		}
		section = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: append(append([]sectionrecord.CurveSegment(nil), kept...), split...)}}
	}
	sec := Section{Wall: momentinput.Profile{Outer: wallLoop}, Cavity: offRegion, Caps: section}
	if !inward {
		sec.Cavity, sec.Caps = section, offRegion
	}

	// §4.7 step 2: modify §5's audit of W and of the offset region (C inward,
	// O outward); together they hold every pair of a new segment with another.
	for _, region := range []momentinput.Profile{sec.Wall, offRegion} {
		if err := audit(budget, profile, region); err != nil {
			return Section{}, err
		}
	}
	// §4.7 step 3: the regions tile exactly — area(P) = area(W) + area(C)
	// inward, area(O) = area(W) + area(P) outward — read in exact rational
	// arithmetic over the line walks.
	if err := offset2d.RequireAreaIdentity(sec.Caps, sec.Wall, sec.Cavity); err != nil {
		return Section{}, err
	}

	// A rim cut backward along r's carrier leaves v strictly inside the
	// cavity's walk there (Table RO's reflex row). On an oblique r the rim
	// column v → q is one swept face over every slab, split at q where the
	// cavity begins: v is marked at the same levels, so the kept wall's edge
	// through v splits where the column's other side line does, and the two
	// faces' edges pair piece for piece. A circular r's rim column is marked
	// alike.
	if !forwardB || offset2d.SplitAtCuts(rFirst) {
		sec.Corners = append(sec.Corners, vB)
	}
	if !forwardA || offset2d.SplitAtCuts(rLast) {
		sec.Corners = append(sec.Corners, vA)
	}
	sec.Delta, err = offset2d.ChainSectionDelta(budget, chain, offset2d.MirrorLine{}, off.Ends, s, tmm, tDelta, tol)
	if err != nil {
		return Section{}, err
	}
	sec.CutGap = gap
	return sec, nil
}
