package massmoment

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PrismRecord carries the mass inputs of one admitted solid prism.
type PrismRecord struct {
	Profile                        momentinput.Profile
	Frame                          r3.Frame
	Transform                      r3.Transform
	Z0, Z1                         float64
	SectionDelta, Z0Delta, Z1Delta float64
}

// GeneralPrismProperties integrates an admitted prism's displaced profile
// and levels, maps its moments through the frame and placement, then publishes
// the centroidal mass and inertia readings.
func GeneralPrismProperties(ctx context.Context, p PrismRecord, center measurement.VecMeasurement,
	density units.Value) (MassProperties, error) {
	moments, err := PrismProfileMoments(ctx, p.Profile, p.Z0, p.Z1,
		p.SectionDelta, p.Z0Delta, p.Z1Delta)
	if err != nil {
		return MassProperties{}, err
	}
	basis, err := PrismRotation(p.Frame, p.Transform)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := AffineInertia(moments, basis, density)
	if err != nil {
		return MassProperties{}, err
	}
	return Publish(ctx, center, massIv, world)
}

// CardinalPrismProperties integrates an undisplaced prism whose frame and
// placement have signed-permutation axes. The exact axis map reorders and
// negates the local tensor entries without another rounding charge.
func CardinalPrismProperties(ctx context.Context, p PrismRecord, center measurement.VecMeasurement,
	density units.Value) (MassProperties, error) {
	section, err := PrismSectionMoments(ctx, p.Profile)
	if err != nil {
		return MassProperties{}, err
	}
	massIv, world, err := CardinalPrismInertia(section, p.Z0, p.Z1, p.Frame, p.Transform, density)
	if err != nil {
		return MassProperties{}, err
	}
	result := MassProperties{Center: center}
	result.Mass, err = IntervalReading(massIv, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	entries := []struct {
		iv      proofbound.RatInterval
		reading *measurement.Measurement
	}{
		{world[0][0], &result.Inertia.XX}, {world[1][1], &result.Inertia.YY},
		{world[2][2], &result.Inertia.ZZ}, {world[0][1], &result.Inertia.XY},
		{world[0][2], &result.Inertia.XZ}, {world[1][2], &result.Inertia.YZ},
	}
	for _, entry := range entries {
		*entry.reading, err = IntervalReading(entry.iv, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// CardinalBoxProperties integrates a rectangular source prism with exact
// dyadic dimensions under a signed-permutation frame and placement.
func CardinalBoxProperties(ctx context.Context, p PrismRecord, center measurement.VecMeasurement,
	density units.Value) (MassProperties, error) {
	mass, readings, err := CardinalBoxMoments(ctx, p.Profile.Outer.Segments, p.Z0, p.Z1,
		p.Frame, p.Transform, density)
	if err != nil {
		return MassProperties{}, err
	}
	result := MassProperties{Center: center}
	result.Mass, err = Reading(mass, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Mag() >= result.Mass.Value.Mag() {
		return MassProperties{}, fmt.Errorf("%w: mass interval does not prove positive mass", decaderr.ErrUnsupported)
	}
	diagonal := [3]*measurement.Measurement{&result.Inertia.XX, &result.Inertia.YY, &result.Inertia.ZZ}
	for i, exact := range readings {
		*diagonal[i], err = Reading(exact, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
		if diagonal[i].Bound.Mag() >= diagonal[i].Value.Mag() {
			return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", decaderr.ErrUnsupported)
		}
	}
	zero := measurement.Measurement{
		Value: units.KilogramSquareMillimeters(0), Bound: units.KilogramSquareMillimeters(0),
		Exactness: measurement.Exact,
	}
	result.Inertia.XY, result.Inertia.XZ, result.Inertia.YZ = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}
