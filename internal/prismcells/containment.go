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
