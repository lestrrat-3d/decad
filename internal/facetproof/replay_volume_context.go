package facetproof

import (
	"context"
	"errors"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// VolumeFromAuditReplay verifies the recorded volume using the caller context.
func VolumeFromAuditReplay(ctx context.Context, budget *proofbound.WorkBudget, audit *MeshAudit,
	volSymDiff float64) (float64, float64, *big.Rat, bool, error) {
	if budget == nil || !budget.Replay {
		return 0, 0, nil, false, errors.New("bounded volume requires budget")
	}
	if ok, err := boundedFacetPoll(budget); err != nil || !ok {
		return 0, 0, nil, false, err
	}
	if audit == nil || len(audit.CompVol) != 1 || audit.CompVol[0] == nil ||
		proofbound.IsNonFinite(volSymDiff) || volSymDiff < 0 {
		return 0, 0, nil, false, nil
	}
	component := audit.CompVol[0]
	if component.Sign() <= 0 || component.Num().BitLen() > boundedVolumeRatBits ||
		component.Denom().BitLen() > boundedVolumeRatBits {
		return 0, 0, nil, false, nil
	}
	ok, err := boundedFacetCharge(budget, boundedVolumeReadWork)
	if err != nil || !ok {
		if errors.Is(err, proofbound.ErrWallWorkBudget) {
			return 0, 0, nil, false, nil
		}
		return 0, 0, nil, false, err
	}
	if err := budget.ReserveBytes(256); err != nil {
		return 0, 0, nil, false, err
	}
	zero := new(big.Rat)
	volumeRat, err := proof.ReplayRatArithmetic(ctx, component, zero, '+', replayExactMeter{budget})
	if err != nil {
		return 0, 0, nil, false, err
	}
	volumeFloat, err := proof.ReplayFractionFloat64(ctx, volumeRat.Num(), volumeRat.Denom(), replayExactMeter{budget})
	if err != nil {
		return 0, 0, nil, false, err
	}
	if proofbound.IsNonFinite(volumeFloat) || volumeFloat <= 0 {
		return 0, 0, nil, false, nil
	}
	rounding, err := proof.ReplayRationalFloatError(ctx, volumeRat, volumeFloat, replayExactMeter{budget})
	if err != nil {
		return 0, 0, nil, false, err
	}
	bound := proofbound.AbsSumUpper(volSymDiff, rounding)
	if proofbound.IsNonFinite(bound) {
		return 0, 0, nil, false, nil
	}
	// Reaching the ceiling on this final charged operation is permitted.
	// A cancellation reported by the shared operation budget still wins.
	if stop := budget.Err(); stop != nil && !errors.Is(stop, proofbound.ErrWallWorkBudget) {
		return 0, 0, nil, false, stop
	}
	return volumeFloat, bound, volumeRat, true, nil
}
