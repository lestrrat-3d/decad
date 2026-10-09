package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// sourceSphereMassProperties integrates the exact ball proved by its source
// semicircle and on-axis diameter. Its centroidal tensor is isotropic, so a
// later rigid placement changes the center but no tensor component.
func sourceSphereMassProperties(ctx context.Context, b *Body, sphere sourceSphereContactProof,
	density units.Value) (MassProperties, error) {
	radius := sphere.radius.Rat()
	if radius.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source sphere has no positive radius", ErrUnsupported)
	}
	radius2 := new(big.Rat).Mul(radius, radius)
	radius3 := new(big.Rat).Mul(radius2, radius)
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	massFactor := new(big.Rat).Mul(rho, radius3)
	massFactor.Mul(massFactor, big.NewRat(4, 3))
	massInterval := proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), massFactor)
	if massInterval.Lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source sphere mass interval is not positive", ErrUnsupported)
	}
	inertiaFactor := new(big.Rat).Mul(radius2, big.NewRat(2, 5))
	inertiaInterval := proofbound.IntervalScale(massInterval, inertiaFactor)
	if inertiaInterval.Lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source sphere inertia interval is not positive", ErrUnsupported)
	}
	result := MassProperties{Center: b.centroid}
	var err error
	result.Mass, err = massmoment.IntervalReading(massInterval, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: source sphere mass reading is not positive", ErrUnsupported)
	}
	diagonal, err := massmoment.IntervalReading(inertiaInterval, units.KilogramSquareMillimeter)
	if err != nil {
		return MassProperties{}, err
	}
	if diagonal.Bound.Base() >= diagonal.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: source sphere inertia reading is not positive", ErrUnsupported)
	}
	result.Inertia.XX, result.Inertia.YY, result.Inertia.ZZ = diagonal, diagonal, diagonal
	zero := Measurement{Value: units.KilogramSquareMillimeters(0),
		Bound: units.KilogramSquareMillimeters(0), Exactness: Exact}
	result.Inertia.XY, result.Inertia.XZ, result.Inertia.YZ = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}
