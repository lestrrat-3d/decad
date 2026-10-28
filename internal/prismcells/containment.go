package prismcells

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// OperandLoop names one recorded loop of a scene: an operand, one of its
// regions, and the outer loop (Hole -1) or one hole of that region.
type OperandLoop struct {
	IsB    bool
	Region int
	Hole   int
}

// DeclareOperandLoops declares every recorded loop of both operands as a
// Sketch loop of the scene, its entities in recorded segment order, and returns
// the loops in declaration order. Containment verdicts are recorded only on
// profiles published after this call. ok=false (err nil) means a loop's
// entities are not one entity per recorded segment, such as where the scene
// shares one entity between two segments of an operand.
func DeclareOperandLoops(budget *proofbound.WorkBudget, s *sketch.Sketch,
	tags map[sketch.Entity]Origin) ([]OperandLoop, bool, error) {
	type member struct {
		loop    OperandLoop
		segment int
		entity  sketch.Entity
	}
	members := make([]member, 0, len(tags))
	for ent, origin := range tags {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		members = append(members, member{
			loop:    OperandLoop{IsB: origin.IsB, Region: origin.Region, Hole: origin.Hole},
			segment: origin.Segment, entity: ent,
		})
	}
	compareLoop := func(a, b OperandLoop) int {
		if a.IsB != b.IsB {
			if a.IsB {
				return 1
			}
			return -1
		}
		return cmp.Or(cmp.Compare(a.Region, b.Region), cmp.Compare(a.Hole, b.Hole))
	}
	slices.SortFunc(members, func(a, b member) int {
		return cmp.Or(compareLoop(a.loop, b.loop), cmp.Compare(a.segment, b.segment))
	})
	var loops []OperandLoop
	for i := 0; i < len(members); {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		j := i
		var entities []sketch.Entity
		for j < len(members) && members[j].loop == members[i].loop {
			if members[j].segment != j-i {
				return nil, false, nil
			}
			entities = append(entities, members[j].entity)
			j++
		}
		if _, err := s.DeclareLoop(entities...); err != nil {
			return nil, false, nil //nolint:nilerr // a loop Sketch will not declare is a miss
		}
		loops = append(loops, members[i].loop)
		i = j
	}
	return loops, true, nil
}

// ContainedMembership reads each cell's membership in A and in B from Sketch's
// containment verdicts against the loops DeclareOperandLoops declared: a cell
// is in an operand when it lies inside the outer loop of one of the operand's
// regions and outside every hole of that region. Sketch proves each verdict;
// decad only combines them. ok=false (err nil) means a verdict is missing or
// unproven, or the cell is invalid.
func ContainedMembership(budget *proofbound.WorkBudget, profiles []*sketch.Profile,
	loops []OperandLoop) ([]bool, []bool, bool, error) {
	matterA := make([]bool, len(profiles))
	matterB := make([]bool, len(profiles))
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, nil, false, err
		}
		verdicts := p.Containments()
		if !p.Valid || len(verdicts) < len(loops) {
			return nil, nil, false, nil
		}
		type regionKey struct {
			isB    bool
			region int
		}
		inOuter := map[regionKey]bool{}
		inHole := map[regionKey]bool{}
		for k, loop := range loops {
			if err := budget.Step(); err != nil {
				return nil, nil, false, err
			}
			var inside bool
			switch verdicts[k].State {
			case sketch.ContainmentInside:
				inside = true
			case sketch.ContainmentOutside:
			default:
				return nil, nil, false, nil
			}
			key := regionKey{isB: loop.IsB, region: loop.Region}
			if loop.Hole < 0 {
				inOuter[key] = inside
				continue
			}
			inHole[key] = inHole[key] || inside
		}
		for key, inside := range inOuter {
			if !inside || inHole[key] {
				continue
			}
			if key.isB {
				matterB[i] = true
			} else {
				matterA[i] = true
			}
		}
	}
	return matterA, matterB, true, nil
}

// loopVerdict is the verdict of cell against the declared loop at index k.
func loopVerdict(cell *sketch.Profile, k int) sketch.ContainmentState {
	verdicts := cell.Containments()
	if k < 0 || k >= len(verdicts) {
		return sketch.ContainmentUndeclared
	}
	return verdicts[k].State
}

// LoopEntitiesWalked reports whether every entity of every declared operand
// loop is walked over its whole domain by the published cells, counting the
// window a coincident span withdraws from it under the other operand's entity
// (CoincidentEdges). Sketch leaves out a cycle whose area falls below its
// per-cycle floor, and then publishes no cell that walks that cycle's edges,
// though its containment verdicts still read the rest of the scene as proven.
// A part of a recorded loop that no cell walks is such a loss, so a false
// answer refuses. The check reads only which parameter ranges Sketch reported;
// it compares no coordinate and can only refuse.
func LoopEntitiesWalked(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin,
	profiles []*sketch.Profile) (bool, error) {
	type interval struct{ t0, t1 float64 }
	walked := map[sketch.Entity][]interval{}
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return false, err
				}
				walked[e.Entity] = append(walked[e.Entity], interval{e.TStart, e.TEnd})
			}
		}
	}
	coincident, ok, err := CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return false, err
	}
	windows := map[sketch.Entity][]CoincidentSpan{}
	for _, span := range coincident.Spans {
		windows[span.Losing] = append(windows[span.Losing], span)
	}
	covered := func(ent sketch.Entity, g0, g1 float64) bool {
		for _, w := range windows[ent] {
			switch {
			case w.Whole:
				return true
			case w.LosingT[0] < 0:
				// A circle's window across its seam is the gap before its
				// first fragment together with the gap after its last.
				if g0 == 0 || g1 == 1 {
					return true
				}
			case w.LosingT[0] <= g0 && g1 <= w.LosingT[1]:
				return true
			}
		}
		return false
	}
	for ent := range tags {
		if err := budget.Step(); err != nil {
			return false, err
		}
		spans := walked[ent]
		slices.SortFunc(spans, func(a, b interval) int { return cmp.Or(cmp.Compare(a.t0, b.t0), cmp.Compare(a.t1, b.t1)) })
		reach := 0.0
		for _, s := range spans {
			if s.t0 > reach && !covered(ent, reach, s.t0) {
				return false, nil
			}
			reach = max(reach, s.t1)
		}
		if reach < 1 && !covered(ent, reach, 1) {
			return false, nil
		}
	}
	return true, nil
}
