package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// sourceRevolvedCylinderMassProperties integrates the exact full cylinder
// admitted by sourceRevolvedCylinderAtPose. Its proof box gives the two axial
// ends and the radial extrema in exact dyadics. A cardinal axis leaves the
// centroidal tensor diagonal in world coordinates.
func sourceRevolvedCylinderMassProperties(ctx context.Context, b *Body,
	cylinder sourceCylinderContactProof, density units.Value) (MassProperties, error) {
	axis := cylinder.axis
	height := proofarith.DySubScalar(cylinder.box.hi[axis], cylinder.box.lo[axis])
	if height.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source cylinder has no positive height", ErrUnsupported)
	}
	var radius proofarith.Dyadic
	for i := range 3 {
		if i == axis {
			continue
		}
		width := proofarith.DySubScalar(cylinder.box.hi[i], cylinder.box.lo[i])
		if width.Sign() <= 0 {
			return MassProperties{}, fmt.Errorf("%w: source cylinder has no positive radius", ErrUnsupported)
		}
		candidate := proofarith.DyMul(width, proofarith.MustDyOf(0.5))
		if radius.Sign() != 0 && proofarith.DyCmp(radius, candidate) != 0 {
			return MassProperties{}, fmt.Errorf("%w: source cylinder radial extents disagree", ErrUnsupported)
		}
		radius = candidate
	}
	radius2 := proofarith.DyMul(radius, radius).Rat()
	height2 := proofarith.DyMul(height, height).Rat()
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	massFactor := new(big.Rat).Mul(rho, height.Rat())
	massFactor.Mul(massFactor, radius2)
	massInterval := intervalScale(interval(piLower, piUpper), massFactor)
	if massInterval.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source cylinder mass interval is not positive", ErrUnsupported)
	}
	axial := intervalScale(massInterval, new(big.Rat).Quo(radius2, big.NewRat(2, 1)))
	transverseFactor := new(big.Rat).Mul(radius2, big.NewRat(3, 1))
	transverseFactor.Add(transverseFactor, height2)
	transverse := intervalScale(massInterval, transverseFactor.Quo(transverseFactor, big.NewRat(12, 1)))
	if axial.lo.Sign() <= 0 || transverse.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source cylinder inertia interval is not positive", ErrUnsupported)
	}
	result := MassProperties{Center: b.centroid}
	var err error
	result.Mass, err = massIntervalReading(massInterval, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: source cylinder mass reading is not positive", ErrUnsupported)
	}
	axialReading, err := massIntervalReading(axial, units.KilogramSquareMillimeter)
	if err != nil {
		return MassProperties{}, err
	}
	transverseReading, err := massIntervalReading(transverse, units.KilogramSquareMillimeter)
	if err != nil {
		return MassProperties{}, err
	}
	if axialReading.Bound.Base() >= axialReading.Value.Base() ||
		transverseReading.Bound.Base() >= transverseReading.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: source cylinder inertia reading is not positive", ErrUnsupported)
	}
	diagonal := [3]*Measurement{&result.Inertia.XX, &result.Inertia.YY, &result.Inertia.ZZ}
	for i := range diagonal {
		*diagonal[i] = transverseReading
	}
	*diagonal[axis] = axialReading
	zero := Measurement{Value: units.KilogramSquareMillimeters(0),
		Bound: units.KilogramSquareMillimeters(0), Exactness: Exact}
	result.Inertia.XY, result.Inertia.XZ, result.Inertia.YZ = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}
