package facetproof

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FirstMomentsReplay evaluates exact determinant-weighted sums through the
// shared meter. It recomputes determinants from the supplied triangles so a
// stale or mutated audit determinant cache cannot change the result.
func FirstMomentsReplay(ctx context.Context, b *proofbound.WorkBudget, audit *MeshAudit, tris [][3]int) ([3]*big.Rat, error) {
	var out [3]*big.Rat
	if b.ReplayID == nil || audit == nil {
		return out, errBrakeReplayMissingReplayMomentInput
	}
	meter := replayExactMeter{b}
	var sums [3]proof.Dyadic
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
		det, _, err := proof.ReplayDetDyadic(ctx, a, c, d, meter)
		if err != nil {
			return out, err
		}
		// ReplayDetDyadic has validated all coordinate widths and weights.
		// Reserve for the three dyadic lifts before their potentially wide copies.
		if err := b.ReserveBytes(3 * 2048); err != nil {
			return out, err
		}
		if err := meter.ReserveRecords(ctx, 9); err != nil {
			return out, err
		}
		var liftWork uint64
		for _, p := range [3]proof.Xpt{a, c, d} {
			for _, v := range [4]*big.Int{p.X, p.Y, p.Z, p.W} {
				// Two scans, a copy, normalization and a shift are bounded by
				// eight passes over the admitted input words.
				liftWork += uint64((v.BitLen()+bits.UintSize-1)/bits.UintSize+1) * 8
			}
		}
		if err := meter.SpendWork(ctx, liftWork); err != nil {
			return out, err
		}
		av, ok := proof.XptDyV3(a)
		if !ok {
			return out, errBrakeReplayInvalidReplayMomentIndex
		}
		cv, ok := proof.XptDyV3(c)
		if !ok {
			return out, errBrakeReplayInvalidReplayMomentIndex
		}
		dv, ok := proof.XptDyV3(d)
		if !ok {
			return out, errBrakeReplayInvalidReplayMomentIndex
		}
		for axis := range 3 {
			sum, err := proof.ReplayDyadicArithmetic(ctx, av[axis], cv[axis], '+', meter)
			if err != nil {
				return out, err
			}
			sum, err = proof.ReplayDyadicArithmetic(ctx, sum, dv[axis], '+', meter)
			if err != nil {
				return out, err
			}
			term, err := proof.ReplayDyadicArithmetic(ctx, det, sum, '*', meter)
			if err != nil {
				return out, err
			}
			sums[axis], err = proof.ReplayDyadicArithmetic(ctx, sums[axis], term, '+', meter)
			if err != nil {
				return out, err
			}
		}
	}
	for axis := range out {
		var err error
		out[axis], err = proof.ReplayDyadicRat(ctx, sums[axis], meter)
		if err != nil {
			return out, err
		}
	}
	return out, nil
}
