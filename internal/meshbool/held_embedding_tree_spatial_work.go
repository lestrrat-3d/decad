package meshbool

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// sortHeldBoxFacetsWork uses an in-place heap so spatial partitioning needs
// no uncharged sorting scratch. Every comparison and swap is charged first.
func sortHeldBoxFacetsWork(ids []int, boxes [][2]r3.Vec, axis int, budget *proofbound.WorkBudget) error {
	less := func(a, b int) (bool, error) {
		if err := budget.Step(); err != nil {
			return false, err
		}
		x, y := heldBoxAxis(boxes[a][0], axis), heldBoxAxis(boxes[b][0], axis)
		return x < y || x == y && a < b, nil
	}
	swap := func(a, b int) error {
		if err := budget.Step(); err != nil {
			return err
		}
		ids[a], ids[b] = ids[b], ids[a]
		return nil
	}
	sift := func(root, end int) error {
		for root < end/2 {
			child := 2*root + 1
			if child+1 < end {
				right, err := less(ids[child], ids[child+1])
				if err != nil {
					return err
				}
				if right {
					child++
				}
			}
			ordered, err := less(ids[root], ids[child])
			if err != nil {
				return err
			}
			if !ordered {
				return nil
			}
			if err := swap(root, child); err != nil {
				return err
			}
			root = child
		}
		return nil
	}
	for start := len(ids) / 2; start > 0; {
		start--
		if err := sift(start, len(ids)); err != nil {
			return err
		}
	}
	for end := len(ids) - 1; end > 0; end-- {
		if err := swap(0, end); err != nil {
			return err
		}
		if err := sift(0, end); err != nil {
			return err
		}
	}
	return nil
}
