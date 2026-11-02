package meshbool

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// replayHeldPairEmbedded decides the same exact contact rule as
// heldEmbedding.pairEmbeddedExact for prepared held facets.
func replayHeldPairEmbedded(ctx context.Context, m *BoolMesh, i, j int, budget *proofbound.ReplayBudget) (bool, error) {
	if err := budget.SpendWork(ctx, 9); err != nil {
		return false, err
	}
	a, b := m.Tris[i], m.Tris[j]
	var ai, bi [3]int
	n := 0
	for u, av := range a {
		for v, bv := range b {
			if av == bv {
				if n == len(ai) {
					return false, nil
				}
				ai[n], bi[n] = u, v
				n++
			}
		}
	}
	if n >= 3 || zeroXpt(m.Norms[i]) || zeroXpt(m.Norms[j]) {
		return false, nil
	}
	if err := budget.ReserveRecords(ctx, 2); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, uint64(unsafe.Sizeof(ContactPair{}))+uint64(unsafe.Sizeof(ContactBatchResult{}))); err != nil {
		return false, err
	}
	var result [1]ContactBatchResult
	if err := replayContactRunner(budget)(ctx, m, m, []ContactPair{{I: i, J: j}}, result[:], 0); err != nil {
		return false, err
	}
	if result[0].Err != nil {
		return false, result[0].Err
	}
	xa, xb := XtriCorners(m, i), XtriCorners(m, j)
	switch n {
	case 0:
		return result[0].Contact.Kind == ContactNone, nil
	case 1:
		switch result[0].Contact.Kind {
		case ContactPoint:
			return true, nil
		case ContactRegion:
			overlap, err := replayCoplanarOverlap(ctx, xa, xb, m.Norms[i], budget)
			return !overlap, err
		default:
			return false, nil
		}
	default:
		switch result[0].Contact.Kind {
		case ContactSegment:
			return true, nil
		case ContactRegion:
			u, v, err := replayProjectionAxes(ctx, m.Norms[i], budget)
			if err != nil {
				return false, err
			}
			p, err := replayCross2Numerator(ctx, xa[ai[0]], xa[ai[1]], xa[3-ai[0]-ai[1]], u, v, budget)
			if err != nil {
				return false, err
			}
			q, err := replayCross2Numerator(ctx, xa[ai[0]], xa[ai[1]], xb[3-bi[0]-bi[1]], u, v, budget)
			if err != nil {
				return false, err
			}
			return p.Sign()*q.Sign() < 0, nil
		default:
			return false, nil
		}
	}
}
