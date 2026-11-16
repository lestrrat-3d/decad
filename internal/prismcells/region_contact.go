package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// contactSegment is one segment of a section read for CertifiedContact: its
// walk and its allowance c_e.
type contactSegment struct {
	loop, idx, n int
	walk         survey2d.SegmentWalk
	c            float64
	box          [4]float64
	boxed        bool
}

// CertifiedContact is the certified contact test of a section against its own
// denoted section (agent-docs/carrier-record-design.md §5.3 for the extended
// Union's merged profile, §7.6 CB7 for a rewritten section): with F the
// section's contact floor (sectionaudit.ContactFloor), hMax the largest
// h_loop of its record and c_e each segment's walk allowance (§6.2: Lemma H's
// h read with the walk's radius bound and end allowances, plus the closing
// path of each own junction), it requires 4·max(hMax, c_e) < F and, for every
// two segments that share no vertex (two loops' segments always share none),
// G = SegGapLower − c_e − c_f > F. Pairs between any two loops are included,
// so a kept loop is tested against a rewritten one. ok=false (err nil) is a
// refusal; the string names the failing pair or bound for the caller's message.
func CertifiedContact(budget *proofbound.WorkBudget, loops []sectionrecord.LoopRecord, hMax float64) (ok bool, why string, err error) {
	entries, err := sectionaudit.EntriesOf(budget, loops)
	if err != nil {
		return false, "", err
	}
	floor, err := sectionaudit.ContactFloor(budget, entries)
	if err != nil {
		return false, "", err
	}
	if !(proofbound.ProductUpper(4, hMax) < floor) {
		return false, fmt.Sprintf(`four times the largest loop displacement %g is not below the contact floor %g`, hMax, floor), nil
	}
	var segs []contactSegment
	for li, loop := range loops {
		n := len(loop.Segments)
		for j, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil || (!w.IsLine() && !w.IsCircular()) {
				return false, "a segment has no readable walk", nil //nolint:nilerr // an unreadable walk is a refusal
			}
			c := loopWalkAllowance(loop, j, w)
			if !(proofbound.ProductUpper(4, c) < floor) {
				return false, fmt.Sprintf(`four times the walk allowance %g of segment %d of loop %d is not below the contact floor %g`, c, j, li, floor), nil
			}
			cs := contactSegment{loop: li, idx: j, n: n, walk: w, c: c}
			cs.box, cs.boxed = widenedBox(arrangedEdge{walk: w, c: c})
			segs = append(segs, cs)
		}
	}
	for i, a := range segs {
		for _, b := range segs[i+1:] {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return false, "", err
			}
			if a.loop == b.loop {
				d := b.idx - a.idx
				if d <= 1 || d >= a.n-1 {
					continue // neighbours share a vertex
				}
			}
			if a.boxed && b.boxed && boxGapLower(a.box, b.box) > floor {
				continue
			}
			if !GapAbove(a.walk, b.walk, a.c, b.c, floor) {
				return false, fmt.Sprintf(`segment %d of loop %d and segment %d of loop %d are not proven more than the contact floor %g apart`,
					a.idx, a.loop, b.idx, b.loop, floor), nil
			}
		}
	}
	return true, "", nil
}

// loopWalkAllowance is c_e of segment j of a recorded loop (§6.2): Lemma H's h
// read with the walk's radius bound and end allowances, plus the exact closing
// path of each own junction, which no edge holds.
func loopWalkAllowance(loop sectionrecord.LoopRecord, j int, w survey2d.SegmentWalk) float64 {
	seg := loop.Segments[j]
	c := WalkAllowance(seg, w)
	n := len(loop.Segments)
	if n < 2 {
		return c
	}
	return proofbound.AbsSumUpper(c, junctionClosing(loop.Segments[(j+n-1)%n], seg), junctionClosing(seg, loop.Segments[(j+1)%n]))
}
