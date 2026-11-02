package facetedrecord

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func replayPlacedCapCentroid(ctx context.Context, source measurement.VecMeasurement, motion r3.Transform, charge massmoment.MapCharge, b *proofbound.ReplayBudget) (measurement.VecMeasurement, error) {
	basis := motion.Basis()
	p, err := proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, motion.Translation(), source.Value, b)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	v, err := proof.ReplayXptVec(ctx, p, b)
	if err != nil {
		return measurement.VecMeasurement{}, err
	}
	var gap float64
	for i, n := range [3]*big.Int{p.X, p.Y, p.Z} {
		f := [3]float64{v.X, v.Y, v.Z}[i]
		if math.IsNaN(f) || math.IsInf(f, 0) {
			return measurement.VecMeasurement{}, fmt.Errorf("%w: a planar split vertex is not representable", decaderr.ErrUnsupported)
		}
		q, e := proof.ReplayReducedFraction(ctx, n, p.W, b)
		if e != nil {
			return measurement.VecMeasurement{}, e
		}
		d, e := proof.ReplayRationalFloatError(ctx, q, f, b)
		if e != nil {
			return measurement.VecMeasurement{}, e
		}
		gap = max(gap, d)
	}
	bound := proofbound.AbsSumUpper(proofbound.ProductUpper(source.Bound.Base(), proofbound.AbsSumUpper(1, charge.Stretch)), proofbound.Radius3D(gap))
	return measurement.VecMeasurement{Value: v, Exactness: measurement.ExactnessOf(bound), Bound: units.Millimeters(bound)}, nil
}
