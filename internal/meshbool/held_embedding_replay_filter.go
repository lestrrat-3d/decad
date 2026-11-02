package meshbool

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// replayHeldCachedSign uses an exact cached float decision or the existing
// bounded orientation predicate when the cache cannot decide.
func replayHeldCachedSign(ctx context.Context, plane *proof.FloatOrientationPlane, a, b, c, d r3.Vec, xa, xd, n proof.Xpt, budget proof.ReplayKeyBudget) (int, error) {
	if err := budget.SpendWork(ctx, 64); err != nil {
		return 0, err
	}
	if sign, certain := plane.Sign(d); certain {
		return sign, nil
	}
	return proof.ReplayOrientSignPrepared(ctx, a, b, c, d, xa, xd, n, budget)
}

// replayHeldCachedContactSigns preserves the classifier's sign and early-exit order.
func replayHeldCachedContactSigns(ctx context.Context, ta, tb [3]r3.Vec, xa, xb [3]proof.Xpt, na, nb proof.Xpt, pa, pb *proof.FloatOrientationPlane, budget proof.ReplayKeyBudget) (sa, sb [3]int, separated bool, err error) {
	for i := range 3 {
		sb[i], err = replayHeldCachedSign(ctx, pa, ta[0], ta[1], ta[2], tb[i], xa[0], xb[i], na, budget)
		if err != nil {
			return
		}
	}
	if AllOneSide(sb) {
		separated = true
		return
	}
	for i := range 3 {
		sa[i], err = replayHeldCachedSign(ctx, pb, tb[0], tb[1], tb[2], ta[i], xb[0], xa[i], nb, budget)
		if err != nil {
			return
		}
	}
	separated = AllOneSide(sa)
	return
}
