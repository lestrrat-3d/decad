package meshbool

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replayCoordDistance(ctx context.Context, b *proofbound.WorkBudget, p proof.Xpt, v r3.Vec) (*big.Rat, error) {
	meter := replayExactWork{b}
	var worst *big.Rat
	for i, f := range [3]float64{v.X, v.Y, v.Z} {
		a, err := proof.ReplayReducedFraction(ctx, XIntCoordOf(p, i), p.W, meter)
		if err != nil {
			return nil, err
		}
		c, err := proof.ReplayFloatRat(ctx, f, meter)
		if err != nil {
			return nil, err
		}
		d, err := proof.ReplayRatArithmetic(ctx, a, c, '-', meter)
		if err != nil {
			return nil, err
		}
		if d.Sign() < 0 {
			d.Neg(d)
		}
		if worst == nil {
			worst = d
			continue
		}
		cmp, err := proof.ReplayRatCompare(ctx, d, worst, meter)
		if err != nil {
			return nil, err
		}
		if cmp > 0 {
			worst = d
		}
	}
	return worst, nil
}
func replayGapBounds(ctx context.Context, b *proofbound.WorkBudget, gaps []*big.Rat, xbeta []float64, remap []int, n int) (*big.Rat, []float64, error) {
	if n < 0 || len(gaps) != len(xbeta) || len(gaps) != len(remap) {
		return nil, nil, errReplayReplayGapInputs
	}
	if err := b.Reserve(uint64(n)); err != nil {
		return nil, nil, err
	}
	if _, err := b.StepN(uint64(n)); err != nil {
		return nil, nil, err
	}
	if err := b.ReserveBytesMul(uint64(n), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, nil, err
	}
	bounds := make([]float64, n)
	var worst *big.Rat
	meter := replayExactWork{b}
	for i, g := range gaps {
		if err := b.Step(); err != nil {
			return nil, nil, err
		}
		if g == nil || remap[i] < 0 || remap[i] >= n {
			return nil, nil, errReplayReplayGapRecord
		}
		if worst == nil {
			worst = g
		} else {
			cmp, err := proof.ReplayRatCompare(ctx, g, worst, meter)
			if err != nil {
				return nil, nil, err
			}
			if cmp > 0 {
				worst = g
			}
		}
		value := xbeta[i]
		if g.Sign() > 0 {
			f, err := proof.ReplayFractionFloat64(ctx, g.Num(), g.Denom(), meter)
			if err != nil {
				return nil, nil, err
			}
			value = proofbound.AbsSumUpper(value, proofbound.Radius3D(proofbound.ProvenUpRound(f)))
		}
		if proofbound.IsNonFinite(value) {
			return nil, nil, errReplayAResultVertexSDisplacementBoundIs
		}
		bounds[remap[i]] = max(bounds[remap[i]], value)
	}
	if worst == nil {
		if err := b.Reserve(1); err != nil {
			return nil, nil, err
		}
		if err := b.Step(); err != nil {
			return nil, nil, err
		}
		if err := b.ReserveBytesMul(1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
			return nil, nil, err
		}
		worst = new(big.Rat)
	}
	return worst, bounds, nil
}
