package sectionaudit

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Cutback holds a corner's claims on the incoming and outgoing walls.
type Cutback struct{ A, B float64 }

// RewriteLoop pairs coalesced section walks with their corner cutbacks.
type RewriteLoop struct {
	Walks    []survey2d.SideWalk
	Cutbacks map[int]Cutback
}

// AuditRewrite checks cutback overruns, orientation, boundary contact, and
// nesting in the order that decides the most specific refusal first.
func AuditRewrite(budget *proofbound.WorkBudget, original, rewritten momentinput.Profile,
	loops []RewriteLoop) error {
	origLoops := append([]LoopRecord{original.Outer}, original.Holes...)
	newLoops := append([]LoopRecord{rewritten.Outer}, rewritten.Holes...)

	// Compute S6 first so an overrun that also flips a loop keeps its S6 cause.
	overrunWalk := make([]int, len(loops))
	overrun := make([]bool, len(loops))
	for li, loop := range loops {
		var err error
		overrunWalk[li], overrun[li], err = firstOverrun(budget, loop)
		if err != nil {
			return err
		}
	}

	// S8: the rewrite preserves every loop's material orientation.
	for i := range origLoops {
		before, err := LoopSignedArea(budget, origLoops[i])
		if err != nil {
			return auditError(err, fmt.Sprintf(`original loop %d: %v`, i, err))
		}
		after, err := LoopSignedArea(budget, newLoops[i])
		if err != nil {
			return auditError(err, fmt.Sprintf(`rewritten loop %d: %v`, i, err))
		}
		if math.Signbit(before) != math.Signbit(after) || math.Abs(after) <= Tolerance*math.Abs(before) {
			if overrun[i] {
				return cutbackOverrun(i, overrunWalk[i], loops[i])
			}
			legacy := fmt.Errorf(`%w: the rewrite turned a loop inside out — the modification does not fit`, ErrDegenerate)
			return auditError(legacy,
				fmt.Sprintf(`%v: rewritten loop %d turned inside out — the modification does not fit`, ErrDegenerate, i))
		}
	}

	// S6: report an overrun on every loop S8 did not already reject.
	for li, loop := range loops {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return err
		}
		if overrun[li] {
			return cutbackOverrun(li, overrunWalk[li], loop)
		}
	}

	// S7 and S9 share the same resolved boundary entries.
	entries, err := EntriesOf(budget, newLoops)
	if err != nil {
		return err
	}
	if err := Crossing(budget, entries); err != nil {
		return err
	}
	return Nesting(budget, entries, len(newLoops))
}

func firstOverrun(budget *proofbound.WorkBudget, loop RewriteLoop) (int, bool, error) {
	n := len(loop.Walks)
	for i, walk := range loop.Walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		cut := loop.Cutbacks[i].B + loop.Cutbacks[(i+1)%n].A
		if cut >= walk.Length-Tolerance*math.Max(1, walk.Length) {
			return i, true, nil
		}
	}
	return 0, false, nil
}

func cutbackOverrun(loopIndex, walkIndex int, loop RewriteLoop) error {
	walk := loop.Walks[walkIndex]
	next := (walkIndex + 1) % len(loop.Walks)
	legacy := fmt.Errorf(`%w: a corner's setback reaches the far end of an adjacent wall; merging the rewrites there is not supported`, ErrUnsupported)
	detailed := fmt.Sprintf(`%v: loop %d walk %d from corner %d at (u, v) = (%s, %s) to corner %d at (u, v) = (%s, %s) is consumed by its corner setbacks; merging the rewrites there is not supported`,
		ErrUnsupported, loopIndex, walkIndex, walkIndex, renderCoord(walk.StartU), renderCoord(walk.StartV),
		next, renderCoord(walk.EndU), renderCoord(walk.EndV))
	return auditError(legacy, detailed)
}

// TrimmedWall rejects claims that consume a swept brep wall.
func TrimmedWall(walk survey2d.SegmentWalk, claimStart, claimEnd float64) error {
	if claimStart+claimEnd < walk.Length-Tolerance*math.Max(1, walk.Length) {
		return nil
	}
	legacy := fmt.Errorf(`%w: a corner's setback reaches the far end of an adjacent wall; merging the rewrites there is not supported`, ErrUnsupported)
	detailed := fmt.Sprintf(`%v: the swept wall from (u, v) = (%s, %s) to (%s, %s) is consumed by its blends' setbacks; merging the rewrites there is not supported`,
		ErrUnsupported, renderCoord(walk.StartU), renderCoord(walk.StartV), renderCoord(walk.EndU), renderCoord(walk.EndV))
	return auditError(legacy, detailed)
}

// EntriesOf resolves one section's loops into walks with their loop positions.
// One free-form work counter covers every segment in the section.
func EntriesOf(budget *proofbound.WorkBudget, loops []LoopRecord) ([]Entry, error) {
	work := freeform.NewFreeformWork()
	var entries []Entry
	for li, loop := range loops {
		n := len(loop.Segments)
		for i, seg := range loop.Segments {
			if err := survey2d.WallBudgetStep(budget); err != nil {
				return nil, err
			}
			walk, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, auditError(err, fmt.Sprintf(`loop %d segment %d: %v`, li, i, err))
			}
			if err := boundarywalk.RequireAnalyticWalk(walk, "the section audit"); err != nil {
				return nil, auditError(err, fmt.Sprintf(`loop %d segment %d: %v`, li, i, err))
			}
			entries = append(entries, NewEntry(li, i, n, walk))
		}
	}
	return entries, nil
}
