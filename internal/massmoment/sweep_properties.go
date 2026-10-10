package massmoment

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// SweepSpanRecord carries one certified prism or revolve reduction of a
// composite sweep. Every span must share one accumulated placement.
type SweepSpanRecord struct {
	Prism   PrismRecord
	Revolve RevolveRecord
	Arc     bool
}

func (s SweepSpanRecord) placement() r3.Transform {
	if s.Arc {
		return s.Revolve.Transform
	}
	return s.Prism.Transform
}

// SweepRecord carries the analytic reduction of a sweep. Spans is nonempty
// for a composite; otherwise Arc chooses the one-span prism or revolve path.
type SweepRecord struct {
	Prism   PrismRecord
	Revolve RevolveRecord
	Arc     bool
	Spans   []SweepSpanRecord
}

// SweepProperties integrates the recorded one-span or composite sweep and
// publishes its centroidal mass and inertia readings.
func SweepProperties(ctx context.Context, p SweepRecord, center measurement.VecMeasurement,
	density units.Value) (MassReadings, error) {
	if len(p.Spans) != 0 {
		return compositeSweepProperties(ctx, p.Spans, center, density)
	}
	if p.Arc {
		return RevolveProperties(ctx, p.Revolve, center, density)
	}
	return GeneralPrismProperties(ctx, p.Prism, center, density)
}

// compositeSweepProperties sums every span's moments about the first span's
// local origin, then publishes them through their shared placement.
func compositeSweepProperties(ctx context.Context, spans []SweepSpanRecord,
	center measurement.VecMeasurement, density units.Value) (MassReadings, error) {
	placement := spans[0].placement()
	var total Moments
	var anchor [3]*big.Rat
	for i, span := range spans {
		if err := ctx.Err(); err != nil {
			return MassReadings{}, err
		}
		if span.placement() != placement {
			return MassReadings{}, fmt.Errorf("%w: composite sweep spans do not share one placement", decaderr.ErrUnsupported)
		}
		local, frame, origin, err := sweepSpanMoments(ctx, span)
		if err != nil {
			return MassReadings{}, fmt.Errorf("sweep path span %d: %w", i, err)
		}
		if i == 0 {
			anchor = origin
		}
		var offset [3]*big.Rat
		for k := range offset {
			offset[k] = new(big.Rat).Sub(origin[k], anchor[k])
		}
		unplaced := Shift(Transform(local, frame), offset)
		if i == 0 {
			total = unplaced
			continue
		}
		total = Add(total, unplaced)
	}
	rotation, err := PlacementRotation(placement)
	if err != nil {
		return MassReadings{}, err
	}
	world, massIv, err := AffineInertia(total, rotation, density)
	if err != nil {
		return MassReadings{}, err
	}
	return Publish(ctx, center, massIv, world)
}

// sweepSpanMoments returns one span's local moments, the exact rational map
// to the composite's unplaced axes, and that map's local origin.
func sweepSpanMoments(ctx context.Context, span SweepSpanRecord) (Moments, [3][3]*big.Rat, [3]*big.Rat, error) {
	if span.Arc {
		p := span.Revolve
		local, err := RevolveProfileMoments(ctx, p)
		if err != nil {
			return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		frame, err := RevolveRotation(p.Frame, p.DU, p.DV, r3.Identity())
		if err != nil {
			return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		origin, err := RevolveAnchor(p.Frame, p.AU, p.AV)
		if err != nil {
			return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		return local, frame, origin, nil
	}
	p := span.Prism
	mid, err := PrismMidLevel(p.Z0, p.Z1)
	if err != nil {
		return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	local, err := PrismProfileMoments(ctx, p.Profile, p.Z0, p.Z1,
		p.SectionDelta, p.Z0Delta, p.Z1Delta)
	if err != nil {
		return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	// PrismProfileMoments is about (0, 0, zm); the span's rigid motion is
	// anchored at the frame origin.
	local = Shift(local, [3]*big.Rat{new(big.Rat), new(big.Rat), mid})
	frame, err := PrismRotation(p.Frame, r3.Identity())
	if err != nil {
		return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	origin, ok := ExactVec(p.Frame.Origin())
	if !ok {
		return Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, fmt.Errorf("%w: sweep span frame is not finite", decaderr.ErrNotFinite)
	}
	return local, frame, origin, nil
}
