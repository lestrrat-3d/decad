package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// of the section must be a line, along a section axis or oblique: circular
// walks are that document's PR 3, refused here with SO5's sentinel.

// sideOpeningSection is the three regions of one side opening (§3) and what
// the record build reads beside them.
type sideOpeningSection struct {
	// wall is W, the wall section over the cavity's height.
	wall ProfileRecord
	// cavity is the region each kept cap's interface exposes: C inward (K'
	// then R'), the receiver's section P outward.
	cavity ProfileRecord
	// caps is the region of a kept cap's slab: P inward, O outward (K' then
	// R').
	caps ProfileRecord
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

// prismRemovedRun maps the removed outer-loop segments onto the section's
// coalesced walks and checks §5's SO6 run rule: every removed walk is removed
// whole, the removed walks are one proper connected run around the loop, and
// at least one walk is kept. It returns the run's walk indices in walk order.
func prismRemovedRun(walks []survey2d.SideWalk, segs map[int]struct{}) ([]int, error) {
	n := len(walks)
	removed := make([]bool, n)
	count := 0
	for i, w := range walks {
		hit := 0
		for _, si := range w.Segs {
			if _, ok := segs[si]; ok {
				hit++
			}
		}
		switch hit {
		case 0:
		case len(w.Segs):
			removed[i] = true
			count++
		default:
			return nil, fmt.Errorf(`%w: a removed side face covers only part of a section walk (modify-reach SX8, shell-opening SO6)`, ErrUnsupported)
		}
	}
	if count == 0 || count == n {
		return nil, fmt.Errorf(`%w: a side opening must remove a proper run of the prism's side faces, not all of them (modify-reach SX8, shell-opening SO6)`, ErrUnsupported)
	}
	first := -1
	runs := 0
	for i := range n {
		if removed[i] && !removed[(i+n-1)%n] {
			runs++
			first = i
		}
	}
	if runs != 1 {
		return nil, fmt.Errorf(`%w: a side opening's removed faces must be one connected run (modify-reach SX8, shell-opening SO6)`, ErrUnsupported)
	}
	run := make([]int, 0, count)
	for k := range count {
		run = append(run, (first+k)%n)
	}
	return run, nil
}

// requireLineWalks is the refusal docs/shell-opening-design.md §12 keeps
// until the circular cuts land: every walk of the section, kept or removed,
// must be a line, along a section axis or oblique. A circular walk is
// ErrUnsupported, SO5's sentinel.
func requireLineWalks(walks []survey2d.SideWalk) error {
	for _, w := range walks {
		if w.Kind == survey2d.WalkLine && !w.Closed {
			continue
		}
		return fmt.Errorf(`%w: this evaluator builds a prism side opening only where every section walk is a line; this section holds a circular walk (shell-opening SO5)`, ErrUnsupported)
	}
	return nil
}

// obliqueLine reports whether a line walk lies off both section axes. The
// record build keys an oblique line by its two recorded endpoints, so every
// region walking its carrier must state it in the same pieces (§4.2).
func obliqueLine(w survey2d.SideWalk) bool {
	return !w.IsCircular() && w.StartU != w.EndU && w.StartV != w.EndV
}

// removedPieces writes the removed run on carrier(r) as one region walks it
// (§3, §4.2): the receiver's own run R from vB to vA (recut false), or R',
// the run re-cut to start at qB and end at qA (recut true). forwardB and
// forwardA say each end's cut runs forward into r's span. An oblique end walk
// is stated in the pieces every other region needs: R's walk split at a
// forward cut, and R' split at v where its cut runs backward along the
// carrier, so the regions' pieces on that carrier are vB → qB, qB → qA and
// qA → vA, each keyed by its own endpoints. An axis-aligned end walk keeps
// its one piece: the engine reads its plane by level and splits it at the
// vertices it records. A removed oblique end walk recorded as more than one
// segment is SO5: the record cannot state its pieces alike in every region.
func removedPieces(walks []survey2d.SideWalk, run []int, segs []CurveSegment, qB, qA Point2, forwardB, forwardA, recut bool) ([]CurveSegment, error) {
	rFirst, rLast := walks[run[0]], walks[run[len(run)-1]]
	vB := Point2{U: rFirst.StartU, V: rFirst.StartV}
	vA := Point2{U: rLast.EndU, V: rLast.EndV}
	for _, w := range []survey2d.SideWalk{rFirst, rLast} {
		if obliqueLine(w) && len(w.Segs) != 1 {
			return nil, fmt.Errorf(`%w: a side opening's removed oblique face is recorded as %d collinear segments, which this record build does not state (shell-opening SO5)`, ErrUnsupported, len(w.Segs))
		}
	}
	// splitB and splitA are the extra points each end's walk takes on an
	// oblique carrier: the forward cut in R, the corner in R'.
	splitB := obliqueLine(rFirst) && forwardB != recut
	splitA := obliqueLine(rLast) && forwardA != recut
	var out []CurveSegment
	pieces := func(pts ...Point2) {
		for i := 0; i+1 < len(pts); i++ {
			out = append(out, LineSeg{Start: pts[i], End: pts[i+1], TStart: 0, TEnd: 1})
		}
	}
	from, to := vB, vA
	inB, inA := qB, qA
	if recut {
		from, to = qB, qA
		inB, inA = vB, vA
	}
	if len(run) == 1 {
		if !recut && !splitB && !splitA {
			return appendWalkSegs(out, rFirst, segs), nil
		}
		pts := []Point2{from}
		if splitB {
			pts = append(pts, inB)
		}
		if splitA {
			pts = append(pts, inA)
		}
		pieces(append(pts, to)...)
		return out, nil
	}
	eB := Point2{U: rFirst.EndU, V: rFirst.EndV}
	sA := Point2{U: rLast.StartU, V: rLast.StartV}
	switch {
	case splitB:
		pieces(from, inB, eB)
	case recut:
		pieces(from, eB)
	default:
		out = appendWalkSegs(out, rFirst, segs)
	}
	for _, ri := range run[1 : len(run)-1] {
		out = appendWalkSegs(out, walks[ri], segs)
	}
	switch {
	case splitA:
		pieces(sA, inA, to)
	case recut:
		pieces(sA, to)
	default:
		out = appendWalkSegs(out, rLast, segs)
	}
	return out, nil
}

// appendWalkSegs appends a walk's recorded segments verbatim.
func appendWalkSegs(out []CurveSegment, w survey2d.SideWalk, segs []CurveSegment) []CurveSegment {
	for _, si := range w.Segs {
		out = append(out, segs[si])
	}
	return out
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
// run rule), the line walks this build takes, SO3's height half, the open
// chain's offset (S11a per walk, SO1, SO2 and SO4 per end), modify §5's audit
// of W and of C (O outward) — S8, where a C with no area is SO3, S11b and
// S9 — and the exact area identity, whose failure is SO5. keptCaps is the
// number of caps the shell keeps.
func sideOpeningRegions(budget *proofbound.WorkBudget, pp prismPayload, sides map[int]struct{}, keptCaps int, s float64, t units.Value, tmm, tDelta float64) (sideOpeningSection, error) {
	if len(pp.profile.Holes) > 0 {
		return sideOpeningSection{}, fmt.Errorf(`%w: a side opening of a prism whose section holds %d hole(s) is not supported (modify-reach SX8, shell-opening SO6)`, ErrUnsupported, len(pp.profile.Holes))
	}
	loops, err := profileCornerLoopsBudget(budget, pp.profile)
	if err != nil {
		return sideOpeningSection{}, err
	}
	walks := loops[0].walks
	run, err := prismRemovedRun(walks, sides)
	if err != nil {
		return sideOpeningSection{}, err
	}
	if err := requireLineWalks(walks); err != nil {
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
	var kept []CurveSegment
	for _, w := range chain {
		for _, si := range w.Segs {
			kept = append(kept, segs[si])
		}
	}
	start := chainEnd{kind: openingEnd, removed: rLast}
	end := chainEnd{kind: openingEnd, removed: rFirst}
	off, err := offsetOpenChain(budget, chain, axisFrame{}, start, end, s, tmm)
	if err != nil {
		return sideOpeningSection{}, err
	}
	first, last := chain[0], chain[len(chain)-1]
	vA := Point2{U: first.StartU, V: first.StartV}
	vB := Point2{U: last.EndU, V: last.EndV}
	qA, qB := off.qStart, off.qEnd

	// W: the rims run v → q where the loop leaves K for K' (vB inward, vA
	// outward) and q → v where it returns.
	inward := s > 0
	atEnd, err := offset2d.RimSegment(last, rFirst, true, s, off.ends[1].Join, inward)
	if err != nil {
		return sideOpeningSection{}, err
	}
	atStart, err := offset2d.RimSegment(first, rLast, false, s, off.ends[0].Join, !inward)
	if err != nil {
		return sideOpeningSection{}, err
	}
	wallLoop, err := openChainWallLoop(budget, kept, off.segs, atEnd, atStart, inward)
	if err != nil {
		return sideOpeningSection{}, err
	}

	// R' is the removed run re-cut at both ends: its first walk starts at qB,
	// its last ends at qA, and every walk between is verbatim. A cut runs
	// forward where it lies along r's span from v, backward where it lies on
	// the carrier's extension behind v (Table RO's reflex row inward).
	forwardB := (qB.U-vB.U)*rFirst.TanInU+(qB.V-vB.V)*rFirst.TanInV > 0
	forwardA := (qA.U-vA.U)*rLast.TanOutU+(qA.V-vA.V)*rLast.TanOutV < 0
	recut, err := removedPieces(walks, run, segs, qB, qA, forwardB, forwardA, true)
	if err != nil {
		return sideOpeningSection{}, err
	}
	offRegion := ProfileRecord{Outer: LoopRecord{Segments: append(append([]CurveSegment(nil), off.segs...), recut...)}}
	// P as the record states it: the receiver's own section, or, where an
	// oblique end walk takes a forward cut, K then R split at that cut.
	section := pp.profile
	if (obliqueLine(rFirst) && forwardB) || (obliqueLine(rLast) && forwardA) {
		split, err := removedPieces(walks, run, segs, qB, qA, forwardB, forwardA, false)
		if err != nil {
			return sideOpeningSection{}, err
		}
		section = ProfileRecord{Outer: LoopRecord{Segments: append(append([]CurveSegment(nil), kept...), split...)}}
	}
	sec := sideOpeningSection{wall: ProfileRecord{Outer: wallLoop}, cavity: offRegion, caps: section}
	if !inward {
		sec.cavity, sec.caps = section, offRegion
	}

	// §4.7 step 2: modify §5's audit of W and of the offset region (C inward,
	// O outward); together they hold every pair of a new segment with another.
	for _, region := range []ProfileRecord{sec.wall, offRegion} {
		if err := auditOffsetSectionBudget(budget, pp.profile, region); err != nil {
			return sideOpeningSection{}, err
		}
	}
	// §4.7 step 3: the regions tile exactly — area(P) = area(W) + area(C)
	// inward, area(O) = area(W) + area(P) outward — read in exact rational
	// arithmetic over the line walks.
	if err := requireAreaIdentity(sec.caps, sec.wall, sec.cavity); err != nil {
		return sideOpeningSection{}, err
	}

	// A rim cut backward along r's carrier leaves v strictly inside the
	// cavity's walk there (Table RO's reflex row). On an oblique r the rim
	// column v → q is one swept face over every slab, split at q where the
	// cavity begins: v is marked at the same levels, so the kept wall's edge
	// through v splits where the column's other side line does, and the two
	// faces' edges pair piece for piece.
	if !forwardB || obliqueLine(rFirst) {
		sec.corners = append(sec.corners, vB)
	}
	if !forwardA || obliqueLine(rLast) {
		sec.corners = append(sec.corners, vA)
	}
	sec.delta, err = chainSectionDelta(budget, chain, offset2d.MirrorLine{}, off.ends, s, tmm, tDelta)
	if err != nil {
		return sideOpeningSection{}, err
	}
	return sec, nil
}

// requireAreaIdentity is §4.7 step 3: the cap region is the wall plus the
// cavity inward, and the wall plus the receiver's section outward, each
// loop's area read exactly from its recorded floats. An arc enters as its
// chord: every arc is a join of K', which W and the offset region both walk,
// so its bulge appears on both sides of the identity and cancels, and the
// chord identity is the exact one. A region holding another segment kind,
// or a sum that differs, is SO5.
func requireAreaIdentity(caps, wall, cavity ProfileRecord) error {
	area := func(p ProfileRecord) (*big.Rat, bool) {
		total := new(big.Rat)
		for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
			for _, seg := range loop.Segments {
				var from, to Point2
				switch l := seg.(type) {
				case LineSeg:
					from, to = l.Start, l.End
				case ArcSeg:
					// An ArcSeg walks Start to End counter-clockwise when
					// TStart < TEnd and End to Start otherwise.
					from, to = l.Start, l.End
					if l.TStart > l.TEnd {
						from, to = l.End, l.Start
					}
				default:
					return nil, false
				}
				su, sv := proofarith.FloatRat(from.U), proofarith.FloatRat(from.V)
				eu, ev := proofarith.FloatRat(to.U), proofarith.FloatRat(to.V)
				if su == nil || sv == nil || eu == nil || ev == nil {
					return nil, false
				}
				total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(su, ev), new(big.Rat).Mul(eu, sv)))
			}
		}
		return total, true
	}
	ac, okC := area(caps)
	aw, okW := area(wall)
	ax, okX := area(cavity)
	if !okC || !okW || !okX {
		return fmt.Errorf(`%w: a side opening's regions hold a segment whose area this evaluator does not read exactly (shell-opening SO5)`, ErrUnsupported)
	}
	// Inward the caps region is P and the cavity C, so P = W + C. Outward the
	// caps region is O and the cavity P, so O = W + P: the same sum.
	if ac.Cmp(new(big.Rat).Add(aw, ax)) != 0 {
		return fmt.Errorf(`%w: a side opening's wall and cavity regions do not tile the section exactly (shell-opening SO5)`, ErrUnsupported)
	}
	return nil
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
