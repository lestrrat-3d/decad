package momentinput

import (
	"reflect"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ExactProfileEqual compares recorded profiles in their stored order. It is
// only a sufficient certificate of equal represented sets: cyclic loop
// starts, hole order and alternate segment spellings remain unequal. The
// budget is stepped once per segment comparison, including the first loop.
func ExactProfileEqual(budget *proofbound.WorkBudget, a, b Profile) (bool, error) {
	if err := survey2d.WallBudgetStep(budget); err != nil {
		return false, err
	}
	if len(a.Holes) != len(b.Holes) || (a.Holes == nil) != (b.Holes == nil) {
		return false, nil
	}
	same, err := ExactLoopEqual(budget, a.Outer, b.Outer)
	if err != nil || !same {
		return false, err
	}
	for i := range a.Holes {
		same, err := ExactLoopEqual(budget, a.Holes[i], b.Holes[i])
		if err != nil || !same {
			return false, err
		}
	}
	return true, nil
}

// ExactLoopEqual compares a loop segment by segment. Nil and empty segment
// slices differ, as they do in a whole-record structural comparison.
func ExactLoopEqual(budget *proofbound.WorkBudget, a, b LoopRecord) (bool, error) {
	if len(a.Segments) != len(b.Segments) || (a.Segments == nil) != (b.Segments == nil) {
		return false, nil
	}
	for i := range a.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return false, err
		}
		if !reflect.DeepEqual(a.Segments[i], b.Segments[i]) {
			return false, nil
		}
	}
	return true, nil
}
