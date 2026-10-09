package massmoment

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PrismVolumeMoments integrates section moments about the prism mid level.
func PrismVolumeMoments(ctx context.Context, section [6]proofbound.RatInterval, z0Held, z1Held float64,
	displaced bool, occupiedError func(proofbound.RatInterval, *big.Rat) (*big.Rat, *big.Rat, error)) (Moments, error) {
	a, mu, mv := section[0], section[1], section[2]
	if a.Lo.Sign() <= 0 {
		return Moments{}, fmt.Errorf("%w: section area interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	z0, z1 := proofarith.FloatRat(z0Held), proofarith.FloatRat(z1Held)
	if z0 == nil || z1 == nil {
		return Moments{}, fmt.Errorf("%w: prism levels are not finite", decaderr.ErrNotFinite)
	}
	h := new(big.Rat).Sub(z1, z0)
	if h.Sign() <= 0 {
		return Moments{}, fmt.Errorf("%w: axial interval does not prove positive volume", decaderr.ErrUnsupported)
	}

	zero := proofbound.PointInterval(new(big.Rat))
	volume := proofbound.IntervalScale(a, h)
	first := [3]proofbound.RatInterval{proofbound.IntervalScale(mu, h), proofbound.IntervalScale(mv, h), zero}
	h3Over12 := new(big.Rat).Quo(new(big.Rat).Mul(h, new(big.Rat).Mul(h, h)), big.NewRat(12, 1))
	second := [3][3]proofbound.RatInterval{
		{proofbound.IntervalScale(section[3], h), proofbound.IntervalScale(section[4], h), zero},
		{proofbound.IntervalScale(section[4], h), proofbound.IntervalScale(section[5], h), zero},
		{zero, zero, proofbound.IntervalScale(a, h3Over12)},
	}
	if displaced {
		e, r, err := occupiedError(a, h)
		if err != nil {
			return Moments{}, err
		}
		re := new(big.Rat).Mul(r, e)
		r2e := new(big.Rat).Mul(r, re)
		volume = proofbound.IntervalWiden(volume, e)
		for i := range first {
			first[i] = proofbound.IntervalWiden(first[i], re)
			for j := range second[i] {
				second[i][j] = proofbound.IntervalWiden(second[i][j], r2e)
			}
		}
	}
	if volume.Lo.Sign() <= 0 {
		return Moments{}, fmt.Errorf("%w: volume interval does not prove positive volume", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return Moments{}, err
	}
	return Moments{Volume: volume, First: first, Second: second}, nil
}

// PrismOccupiedError bounds the prism's displaced volume and its coordinate radius.
func PrismOccupiedError(area proofbound.RatInterval, h *big.Rat, sectionDelta, z0Delta, z1Delta float64,
	count int, perimeter, coordUpper float64) (*big.Rat, *big.Rat, error) {
	displaced := proofbound.SectionDisplacementArea(sectionDelta, count, perimeter)
	if !NonNegativeFinite(displaced) || !NonNegativeFinite(coordUpper) {
		return nil, nil, fmt.Errorf("%w: prism displacement has no finite occupied-volume bound", decaderr.ErrUnsupported)
	}
	d0, d1, delta := proofarith.FloatRat(z0Delta), proofarith.FloatRat(z1Delta), proofarith.FloatRat(sectionDelta)
	axial := new(big.Rat).Add(d0, d1)
	e := new(big.Rat).Mul(proofarith.FloatRat(displaced), new(big.Rat).Add(h, axial))
	e.Add(e, new(big.Rat).Mul(proofbound.IntervalAbsUpper(area), axial))

	inPlane := new(big.Rat).Add(proofarith.FloatRat(coordUpper), delta)
	alongAxis := new(big.Rat).Quo(h, big.NewRat(2, 1))
	alongAxis.Add(alongAxis, proofbound.RatMax(d0, d1))
	return e, proofbound.RatMax(inPlane, alongAxis), nil
}
