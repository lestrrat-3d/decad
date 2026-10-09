package massmoment

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// CupProperties subtracts the cavity's outward moment enclosure from the
// outer prism's after both are anchored at the outer prism's mid level.
func CupProperties(ctx context.Context, outer, cavity PrismRecord,
	center measurement.VecMeasurement, density units.Value) (MassProperties, error) {
	outerMid, err := PrismMidLevel(outer.Z0, outer.Z1)
	if err != nil {
		return MassProperties{}, err
	}
	cavityMid, err := PrismMidLevel(cavity.Z0, cavity.Z1)
	if err != nil {
		return MassProperties{}, err
	}
	solid, err := PrismProfileMoments(ctx, outer.Profile, outer.Z0, outer.Z1,
		outer.SectionDelta, outer.Z0Delta, outer.Z1Delta)
	if err != nil {
		return MassProperties{}, err
	}
	void, err := PrismProfileMoments(ctx, cavity.Profile, cavity.Z0, cavity.Z1,
		cavity.SectionDelta, cavity.Z0Delta, cavity.Z1Delta)
	if err != nil {
		return MassProperties{}, err
	}
	void = Shift(void, [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat).Sub(cavityMid, outerMid)})
	rotation, err := PrismRotation(outer.Frame, outer.Transform)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := AffineInertia(Sub(solid, void), rotation, density)
	if err != nil {
		return MassProperties{}, err
	}
	return Publish(ctx, center, massIv, world)
}
