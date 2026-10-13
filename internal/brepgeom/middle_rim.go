package brepgeom

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RimEdgeEnds are the start and end positions of one loop use, in loop order.
type RimEdgeEnds struct{ Start, End r3.Vec }

// AuditMiddleRim checks that two faces built from the same sketch section in
// the same frame hold the same rim: the same number of loops, the same number
// of loop uses per loop, and equal end positions use by use. Each face is its
// loops, each loop its loop uses' end positions.
func AuditMiddleRim(ctx context.Context, from, to [][]RimEdgeEnds) error {
	if len(from) != len(to) {
		return fmt.Errorf(`%w: two-sided taper halves have different middle loop counts`, decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	for li := range from {
		if err := budget.Step(); err != nil {
			return err
		}
		left, right := from[li], to[li]
		if len(left) != len(right) {
			return fmt.Errorf(`%w: two-sided taper halves have different middle rim edge counts`, decaderr.ErrUnsupported)
		}
		for ci := range left {
			if err := budget.Step(); err != nil {
				return err
			}
			if left[ci] != right[ci] {
				return fmt.Errorf(`%w: two-sided taper halves do not share the held middle rim`, decaderr.ErrUnsupported)
			}
		}
	}
	return budget.Err()
}
