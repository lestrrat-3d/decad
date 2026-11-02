package prismcells

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// ReplayBlindSection is a whole-loop section and the source-region index of
// each new hole. The result is private to one replay and is not a Body.
type ReplayBlindSection struct {
	Profile      momentinput.Profile
	HoleRegions  []int
	SectionDelta float64
}

// ReplayBlindGroupSection records the matched target minus every group region.
// It retains original target loops and records only whole matched tool circles.
func ReplayBlindGroupSection(ctx context.Context, cert *ReplayBlindScene, budget *proofbound.ReplayBudget) (ReplayBlindSection, bool, error) {
	var out ReplayBlindSection
	if budget == nil {
		return out, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return out, false, err
	}
	if cert == nil || cert.budget != budget || cert.match == nil {
		return out, false, nil
	}
	clearance, err := ReplayGroupSourceClearance(ctx, cert, budget)
	if err != nil || !clearance {
		return out, false, err
	}
	count := len(cert.group)
	if count < 2 || len(cert.match.Holes) != len(cert.target.Holes)+count {
		return out, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(count*3+len(cert.target.Holes)+1)); err != nil {
		return out, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(cert.target.Holes)+count), 24); err != nil {
		return out, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(count), 8); err != nil {
		return out, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(count), 1); err != nil {
		return out, false, err
	}
	copyLoop := func(loop sectionrecord.LoopRecord) (sectionrecord.LoopRecord, error) {
		if err := budget.ReserveRecords(ctx, uint64(len(loop.Segments))); err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Segments)), 16); err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		cloned := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loop.Segments))}
		for i := range loop.Segments {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return sectionrecord.LoopRecord{}, err
			}
			cloned.Segments[i] = loop.Segments[i]
		}
		return cloned, nil
	}
	outer, err := copyLoop(cert.target.Outer)
	if err != nil {
		return out, false, err
	}
	out.Profile.Outer = outer
	out.Profile.Holes = make([]sectionrecord.LoopRecord, 0, len(cert.target.Holes)+count)
	out.HoleRegions = make([]int, 0, count)
	seen := make([]bool, count)
	for i := range cert.target.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindSection{}, false, err
		}
		cloned, err := copyLoop(cert.target.Holes[i])
		if err != nil {
			return ReplayBlindSection{}, false, err
		}
		out.Profile.Holes = append(out.Profile.Holes, cloned)
	}
	for i := range cert.match.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return ReplayBlindSection{}, false, err
		}
		hole := cert.match.Holes[i]
		if len(hole) == 0 {
			return ReplayBlindSection{}, false, nil
		}
		origin, ok := cert.scene.Tags[hole[0].Entity]
		if !ok {
			return ReplayBlindSection{}, false, nil
		}
		if !origin.IsB {
			continue
		}
		if len(hole) != 1 || hole[0].Partial || origin.Hole != -1 || origin.Segment != 0 || origin.Region < 0 || origin.Region >= count || seen[origin.Region] {
			return ReplayBlindSection{}, false, nil
		}
		if _, ok := hole[0].Entity.(*sketch.Circle); !ok {
			return ReplayBlindSection{}, false, nil
		}
		if err := budget.SpendWork(ctx, 128); err != nil {
			return ReplayBlindSection{}, false, err
		}
		if err := budget.ReserveRecords(ctx, 2); err != nil {
			return ReplayBlindSection{}, false, err
		}
		if err := budget.ReserveBytes(ctx, 512); err != nil {
			return ReplayBlindSection{}, false, err
		}
		segment, err := sketchrecord.RecordEdge(hole[0])
		if err != nil {
			return ReplayBlindSection{}, false, err
		}
		if _, ok := segment.(sectionrecord.CircleSeg); !ok {
			return ReplayBlindSection{}, false, nil
		}
		seen[origin.Region] = true
		out.Profile.Holes = append(out.Profile.Holes, sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{segment}})
		out.HoleRegions = append(out.HoleRegions, origin.Region)
	}
	if len(out.HoleRegions) != count {
		return ReplayBlindSection{}, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return ReplayBlindSection{}, false, err
	}
	inA, inB := cert.scene.Delta.Incoming(cert.targetDelta, cert.groupDelta, cert.scene.ReexpressionDelta)
	out.SectionDelta = math.Max(math.Max(inA, inB), cert.scene.Delta.Crossing)
	if math.IsNaN(out.SectionDelta) || math.IsInf(out.SectionDelta, 0) {
		return ReplayBlindSection{}, false, nil
	}
	return out, true, nil
}

// ReplayBlindSceneMatches binds a scene certificate back to the exact source
// records and placements that the caller authenticated.
func ReplayBlindSceneMatches(ctx context.Context, cert *ReplayBlindScene, target momentinput.Profile, group []momentinput.Profile,
	targetPlacement, groupPlacement prismplacement.Operand, targetDelta, groupDelta float64, budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if cert == nil || cert.budget != budget || targetPlacement.Frame != cert.targetPlacement.Frame || targetPlacement.Xform != cert.targetPlacement.Xform ||
		groupPlacement.Frame != cert.groupPlacement.Frame || groupPlacement.Xform != cert.groupPlacement.Xform || targetDelta != cert.targetDelta || groupDelta != cert.groupDelta || len(group) != len(cert.group) {
		return false, nil
	}
	sameHeld := func(a, b prismplacement.Operand) bool {
		return a.Z0 == b.Z0 && a.Z1 == b.Z1 && a.Z0Delta == b.Z0Delta && a.Z1Delta == b.Z1Delta
	}
	if !sameHeld(targetPlacement, cert.targetPlacement) || !sameHeld(groupPlacement, cert.groupPlacement) {
		return false, nil
	}
	zero, err := proof.ReplayFloatRat(ctx, 0, budget)
	if err != nil {
		return false, err
	}
	ok, err := prismplacement.ReplaySameShiftedInterval(ctx, targetPlacement, cert.targetPlacement, zero, budget)
	if err != nil || !ok {
		return false, err
	}
	ok, err = prismplacement.ReplaySameShiftedInterval(ctx, groupPlacement, cert.groupPlacement, zero, budget)
	if err != nil || !ok {
		return false, err
	}
	equalLoop := func(a, b sectionrecord.LoopRecord) (bool, error) {
		if len(a.Segments) != len(b.Segments) {
			return false, nil
		}
		for i := range a.Segments {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			switch x := a.Segments[i].(type) {
			case sectionrecord.LineSeg:
				y, ok := b.Segments[i].(sectionrecord.LineSeg)
				if !ok || x != y {
					return false, nil
				}
			case sectionrecord.CircleSeg:
				y, ok := b.Segments[i].(sectionrecord.CircleSeg)
				if !ok || x != y {
					return false, nil
				}
			case sectionrecord.ArcSeg:
				y, ok := b.Segments[i].(sectionrecord.ArcSeg)
				if !ok || x != y {
					return false, nil
				}
			default:
				return false, nil
			}
		}
		return true, nil
	}
	equalProfile := func(a, b momentinput.Profile) (bool, error) {
		if len(a.Holes) != len(b.Holes) {
			return false, nil
		}
		ok, err := equalLoop(a.Outer, b.Outer)
		if err != nil || !ok {
			return false, err
		}
		for i := range a.Holes {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			ok, err = equalLoop(a.Holes[i], b.Holes[i])
			if err != nil || !ok {
				return false, err
			}
		}
		return true, nil
	}
	ok, err = equalProfile(target, cert.target)
	if err != nil || !ok {
		return false, err
	}
	for i := range group {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		ok, err = equalProfile(group[i], cert.group[i])
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}
