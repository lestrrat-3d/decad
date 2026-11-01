package facetproof

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// firstMomentsReplay evaluates the same exact determinant-weighted sums
// through the shared fixed-limb rational meter.
func FirstMomentsReplay(ctx context.Context, b *proofbound.WorkBudget, audit *MeshAudit, tris [][3]int) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	if b.ReplayID == nil || audit == nil {
		return out, errBrakeReplayMissingReplayMomentInput
	}
	meter := replayExactMeter{b}
	for i := range out {
		if err := b.ReserveBytes(256); err != nil {
			return out, err
		}
		out[i] = new(big.Rat)
	}
	for _, t := range tris {
		if err := b.Step(); err != nil {
			return out, err
		}
		for _, i := range t {
			if i < 0 || i >= len(audit.XVerts) {
				return out, errBrakeReplayInvalidReplayMomentIndex
			}
		}
		a, c, d := audit.XVerts[t[0]], audit.XVerts[t[1]], audit.XVerts[t[2]]
		_, det, err := proof.ReplayDetDyadic(ctx, a, c, d, meter)
		if err != nil {
			return out, err
		}
		for axis := range 3 {
			coord := func(p proof.Xpt) *big.Int {
				switch axis {
				case 0:
					return p.X
				case 1:
					return p.Y
				default:
					return p.Z
				}
			}
			x, err := proof.ReplayReducedFraction(ctx, coord(a), a.W, meter)
			if err != nil {
				return out, err
			}
			y, err := proof.ReplayReducedFraction(ctx, coord(c), c.W, meter)
			if err != nil {
				return out, err
			}
			z, err := proof.ReplayReducedFraction(ctx, coord(d), d.W, meter)
			if err != nil {
				return out, err
			}
			sum, err := proof.ReplayRatArithmetic(ctx, x, y, '+', meter)
			if err != nil {
				return out, err
			}
			sum, err = proof.ReplayRatArithmetic(ctx, sum, z, '+', meter)
			if err != nil {
				return out, err
			}
			term, err := proof.ReplayRatArithmetic(ctx, det, sum, '*', meter)
			if err != nil {
				return out, err
			}
			out[axis], err = proof.ReplayRatArithmetic(ctx, out[axis], term, '+', meter)
			if err != nil {
				return out, err
			}
		}
	}
	return out, nil
}
