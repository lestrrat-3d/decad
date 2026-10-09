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
	density units.Value) (MassReadings, error) {
	moments, err := PrismProfileMoments(ctx, p.Profile, p.Z0, p.Z1,
		p.SectionDelta, p.Z0Delta, p.Z1Delta)
	if err != nil {
		return MassReadings{}, err
	}
	basis, err := PrismRotation(p.Frame, p.Transform)
	if err != nil {
		return MassReadings{}, err
	}
	world, massIv, err := AffineInertia(moments, basis, density)
	if err != nil {
		return MassReadings{}, err
	}
	return Publish(ctx, center, massIv, world)
}

// CardinalPrismProperties integrates an undisplaced prism whose frame and
// placement have signed-permutation axes. The exact axis map reorders and
// negates the local tensor entries without another rounding charge.
func CardinalPrismProperties(ctx context.Context, p PrismRecord, center measurement.VecMeasurement,
	density units.Value) (MassReadings, error) {
	section, err := PrismSectionMoments(ctx, p.Profile)
	if err != nil {
		return MassReadings{}, err
	}
	massIv, world, err := CardinalPrismInertia(section, p.Z0, p.Z1, p.Frame, p.Transform, density)
	if err != nil {
		return MassReadings{}, err
	}
	result := MassReadings{Center: center}
	result.Mass, err = IntervalReading(massIv, units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	entries := []struct {
		iv      proofbound.RatInterval
		reading *measurement.Measurement
	}{
		{world[0][0], &result.Tensor[0]}, {world[1][1], &result.Tensor[1]},
		{world[2][2], &result.Tensor[2]}, {world[0][1], &result.Tensor[3]},
		{world[0][2], &result.Tensor[4]}, {world[1][2], &result.Tensor[5]},
	}
	for _, entry := range entries {
		*entry.reading, err = IntervalReading(entry.iv, units.KilogramSquareMillimeter)
		if err != nil {
			return MassReadings{}, err
		}
	}
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}

// CardinalBoxProperties integrates a rectangular source prism with exact
// dyadic dimensions under a signed-permutation frame and placement.
func CardinalBoxProperties(ctx context.Context, p PrismRecord, center measurement.VecMeasurement,
	density units.Value) (MassReadings, error) {
	mass, readings, err := CardinalBoxMoments(ctx, p.Profile.Outer.Segments, p.Z0, p.Z1,
		p.Frame, p.Transform, density)
	if err != nil {
		return MassReadings{}, err
	}
	result := MassReadings{Center: center}
	result.Mass, err = Reading(mass, units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	if result.Mass.Bound.Mag() >= result.Mass.Value.Mag() {
		return MassReadings{}, fmt.Errorf("%w: mass interval does not prove positive mass", decaderr.ErrUnsupported)
	}
	diagonal := [3]*measurement.Measurement{&result.Tensor[0], &result.Tensor[1], &result.Tensor[2]}
	for i, exact := range readings {
		*diagonal[i], err = Reading(exact, units.KilogramSquareMillimeter)
		if err != nil {
			return MassReadings{}, err
		}
		if diagonal[i].Bound.Mag() >= diagonal[i].Value.Mag() {
			return MassReadings{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", decaderr.ErrUnsupported)
		}
	}
	zero := measurement.Measurement{
		Value: units.KilogramSquareMillimeters(0), Bound: units.KilogramSquareMillimeters(0),
		Exactness: measurement.Exact,
	}
	result.Tensor[3], result.Tensor[4], result.Tensor[5] = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}
