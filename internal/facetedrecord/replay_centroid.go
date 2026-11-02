package facetedrecord

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type replayCentroidMeter struct{ b *proofbound.WorkBudget }

func (m replayCentroidMeter) ReserveRecords(_ context.Context, n uint64) error { return m.b.Reserve(n) }
func (m replayCentroidMeter) ReserveBytes(_ context.Context, n uint64) error {
	return m.b.ReserveBytes(n)
}
func (m replayCentroidMeter) SpendWork(_ context.Context, n uint64) error {
	_, err := m.b.StepN(n)
	return err
}
func replayCentroid(ctx context.Context, b *proofbound.WorkBudget, moments [3]*big.Rat, volume *big.Rat, sym, diam float64) (r3.Vec, float64, error) {
	meter := replayCentroidMeter{b}
	if err := b.ReserveBytes(256); err != nil {
		return r3.Vec{}, 0, err
	}
	twentyFour := big.NewRat(24, 1)
	var values, errors [3]float64
	for i, m := range moments {
		q, err := proof.ReplayRatArithmetic(ctx, m, twentyFour, '/', meter)
		if err != nil {
			return r3.Vec{}, 0, err
		}
		q, err = proof.ReplayRatArithmetic(ctx, q, volume, '/', meter)
		if err != nil {
			return r3.Vec{}, 0, err
		}
		values[i], err = proof.ReplayRatFloat64(ctx, q, meter)
		if err != nil {
			return r3.Vec{}, 0, err
		}
		if math.IsInf(values[i], 0) || math.IsNaN(values[i]) {
			return r3.Vec{}, 0, errBoundedArithmetic
		}
		errors[i], err = proof.ReplayRationalFloatError(ctx, q, values[i], meter)
		if err != nil {
			return r3.Vec{}, 0, err
		}
	}
	v, err := proof.ReplayRatFloat64(ctx, volume, meter)
	if err != nil {
		return r3.Vec{}, 0, err
	}
	floor := math.Nextafter(v, math.Inf(-1)) - sym
	if floor <= 0 {
		floor = 0
	}
	allowance := facetproof.FacetedCentroidAllowance(sym, diam, floor)
	round := proofbound.Radius3D(math.Max(errors[0], math.Max(errors[1], errors[2])))
	return r3.Vec{X: values[0], Y: values[1], Z: values[2]}, proofbound.AbsSumUpper(allowance, round), nil
}
