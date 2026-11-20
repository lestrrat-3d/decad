package meshbool

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// separatedCoarseFaces checks every facet pair. The outward distance includes
// each facet's original finite-patch correspondence in both directions.
func separatedCoarseFaces(ctx context.Context, a, b *BoolMesh, left, right []int, memo *ContactMemo, meter *proofbound.ReplayBudget) (bool, error) {
	if len(left) == 0 || len(right) == 0 || len(left) > 65536 || len(right) > 65536 || len(left) > 262144/len(right) {
		return false, nil
	}
	if err := meter.SpendWork(ctx, uint64(len(left))*uint64(len(right))*8); err != nil {
		return false, err
	}
	for _, i := range left {
		for _, j := range right {
			reach := proofbound.AbsSumUpper(a.FacetBound[i], b.FacetBound[j])
			if !BoxesWithin(a.Boxes[i], b.Boxes[j], reach) {
				continue
			}
			contact, err := memo.Classify(i, j)
			if err != nil || contact.Kind != ContactNone {
				return false, err
			}
			far, err := triTriDistanceExceedsReplay(ctx, TriCorners(a, i), TriCorners(b, j), reach, meter)
			if err != nil || !far {
				return false, err
			}
		}
	}
	return true, nil
}
