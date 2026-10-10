package massmoment

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// SourceSphere integrates a complete ball already certified by its source
// semicircle and on-axis diameter.
func SourceSphere(ctx context.Context, radiusDy proofarith.Dyadic, center measurement.VecMeasurement,
	density units.Value) (MassReadings, error) {
	radius := radiusDy.Rat()
	if radius.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source sphere has no positive radius", decaderr.ErrUnsupported)
	}
	radius2 := new(big.Rat).Mul(radius, radius)
	radius3 := new(big.Rat).Mul(radius2, radius)
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	massFactor := new(big.Rat).Mul(rho, radius3)
	massFactor.Mul(massFactor, big.NewRat(4, 3))
	massInterval := proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), massFactor)
	if massInterval.Lo.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source sphere mass interval is not positive", decaderr.ErrUnsupported)
	}
	inertiaFactor := new(big.Rat).Mul(radius2, big.NewRat(2, 5))
	inertiaInterval := proofbound.IntervalScale(massInterval, inertiaFactor)
	if inertiaInterval.Lo.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source sphere inertia interval is not positive", decaderr.ErrUnsupported)
	}
	result := MassReadings{Center: center}
	var err error
	result.Mass, err = IntervalReading(massInterval, units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassReadings{}, fmt.Errorf("%w: source sphere mass reading is not positive", decaderr.ErrUnsupported)
	}
	diagonal, err := IntervalReading(inertiaInterval, units.KilogramSquareMillimeter)
	if err != nil {
		return MassReadings{}, err
	}
	if diagonal.Bound.Base() >= diagonal.Value.Base() {
		return MassReadings{}, fmt.Errorf("%w: source sphere inertia reading is not positive", decaderr.ErrUnsupported)
	}
	result.Tensor[0], result.Tensor[1], result.Tensor[2] = diagonal, diagonal, diagonal
	zero := measurement.Measurement{Value: units.KilogramSquareMillimeters(0),
		Bound: units.KilogramSquareMillimeters(0), Exactness: measurement.Exact}
	result.Tensor[3], result.Tensor[4], result.Tensor[5] = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}

// SourceCylinder integrates a certified complete cylinder whose axis is
// cardinal and whose proof box gives exact axial and radial extrema.
func SourceCylinder(ctx context.Context, axis int, lo, hi [3]proofarith.Dyadic,
	center measurement.VecMeasurement, density units.Value) (MassReadings, error) {
	height := proofarith.DySubScalar(hi[axis], lo[axis])
	if height.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source cylinder has no positive height", decaderr.ErrUnsupported)
	}
	var radius proofarith.Dyadic
	for i := range 3 {
		if i == axis {
			continue
		}
		width := proofarith.DySubScalar(hi[i], lo[i])
		if width.Sign() <= 0 {
			return MassReadings{}, fmt.Errorf("%w: source cylinder has no positive radius", decaderr.ErrUnsupported)
		}
		candidate := proofarith.DyMul(width, proofarith.MustDyOf(0.5))
		if radius.Sign() != 0 && proofarith.DyCmp(radius, candidate) != 0 {
			return MassReadings{}, fmt.Errorf("%w: source cylinder radial extents disagree", decaderr.ErrUnsupported)
		}
		radius = candidate
	}
	radius2 := proofarith.DyMul(radius, radius).Rat()
	height2 := proofarith.DyMul(height, height).Rat()
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	massFactor := new(big.Rat).Mul(rho, height.Rat())
	massFactor.Mul(massFactor, radius2)
	massInterval := proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), massFactor)
	if massInterval.Lo.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source cylinder mass interval is not positive", decaderr.ErrUnsupported)
	}
	axial := proofbound.IntervalScale(massInterval, new(big.Rat).Quo(radius2, big.NewRat(2, 1)))
	transverseFactor := new(big.Rat).Mul(radius2, big.NewRat(3, 1))
	transverseFactor.Add(transverseFactor, height2)
	transverse := proofbound.IntervalScale(massInterval, transverseFactor.Quo(transverseFactor, big.NewRat(12, 1)))
	if axial.Lo.Sign() <= 0 || transverse.Lo.Sign() <= 0 {
		return MassReadings{}, fmt.Errorf("%w: source cylinder inertia interval is not positive", decaderr.ErrUnsupported)
	}
	result := MassReadings{Center: center}
	var err error
	result.Mass, err = IntervalReading(massInterval, units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassReadings{}, fmt.Errorf("%w: source cylinder mass reading is not positive", decaderr.ErrUnsupported)
	}
	axialReading, err := IntervalReading(axial, units.KilogramSquareMillimeter)
	if err != nil {
		return MassReadings{}, err
	}
	transverseReading, err := IntervalReading(transverse, units.KilogramSquareMillimeter)
	if err != nil {
		return MassReadings{}, err
	}
	if axialReading.Bound.Base() >= axialReading.Value.Base() ||
		transverseReading.Bound.Base() >= transverseReading.Value.Base() {
		return MassReadings{}, fmt.Errorf("%w: source cylinder inertia reading is not positive", decaderr.ErrUnsupported)
	}
	diagonal := [3]*measurement.Measurement{&result.Tensor[0], &result.Tensor[1], &result.Tensor[2]}
	for i := range diagonal {
		*diagonal[i] = transverseReading
	}
	*diagonal[axis] = axialReading
	zero := measurement.Measurement{Value: units.KilogramSquareMillimeters(0),
		Bound: units.KilogramSquareMillimeters(0), Exactness: measurement.Exact}
	result.Tensor[3], result.Tensor[4], result.Tensor[5] = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}
