package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the sweep's mass path (docs/multibody-dynamics-design.md §8.7,
// docs/dynamic-mass-design.md §2 and §3). A one-span sweep is its own
// analytic reduction: a straight span takes the rotated prism path (§8.1,
// §8.2) and an arc span the general revolve path (§8.6).
//
// A composite sweep is the union of its spans, whose interiors the build
// proved disjoint, so its V, P and Q are the sums of the spans' own, all
// about ONE shared anchor. Each span is integrated in its own local
// coordinates — a straight span's frame-local (u, v, z), an arc span's axis
// basis (w, e0, e1) about its axis anchor — and reaches the composite's
// unplaced coordinates through its own frame: the exact rational image of
// its local origin and the exact image under its rational frame matrix
// (massmoment.Transform), the map the span's readings denote through. The
// exact change of anchor (massmoment.Shift) carries the span's P and Q to
// the shared anchor, so no span is reduced to a centroidal tensor and moved
// by the parallel-axis rule. The summed moments then reach world axes as the
// image under the one placement basis every span shares
// (massmoment.AffineInertia).

// sweepMassProperties dispatches a sweep body to the path its reduction
// takes.
func sweepMassProperties(ctx context.Context, b *Body, sp sweepPayload, density units.Value) (MassProperties, error) {
	switch {
	case len(sp.spans) != 0:
		return compositeSweepMassProperties(ctx, b, sp, density)
	case sp.arc:
		return revolveMassProperties(ctx, b, sp.revolve, density)
	default:
		return rotatedPrismMassProperties(ctx, sp.prism, b.centroid, density)
	}
}

// compositeSweepMassProperties sums every span's moments about the first
// span's local origin, in the composite's unplaced coordinates, then publishes
// them through the shared placement. A span whose placement differs from the
// first's is refused: the sum is only one rigid solid if one motion places
// every span.
func compositeSweepMassProperties(ctx context.Context, b *Body, sp sweepPayload, density units.Value) (MassProperties, error) {
	placement := sp.spans[0].transform()
	var total massmoment.Moments
	var anchor [3]*big.Rat
	for i, span := range sp.spans {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		if span.transform() != placement {
			return MassProperties{}, fmt.Errorf("%w: composite sweep spans do not share one placement", ErrUnsupported)
		}
		local, frame, origin, err := sweepSpanMoments(ctx, span)
		if err != nil {
			return MassProperties{}, fmt.Errorf("sweep path span %d: %w", i, err)
		}
		if i == 0 {
			anchor = origin
		}
		var offset [3]*big.Rat
		for k := range offset {
			offset[k] = new(big.Rat).Sub(origin[k], anchor[k])
		}
		unplaced := massmoment.Shift(massmoment.Transform(local, frame), offset)
		if i == 0 {
			total = unplaced
			continue
		}
		total = massmoment.Add(total, unplaced)
	}
	rotation, err := massmoment.PlacementRotation(placement)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := massmoment.AffineInertia(total, rotation, density)
	if err != nil {
		return MassProperties{}, err
	}
	return massmoment.Publish(ctx, b.centroid, massIv, world)
}

// sweepSpanMoments returns one span's local moments, the exact rational
// matrix taking its local axes to the composite's unplaced axes, and the
// exact unplaced position of its local origin.
func sweepSpanMoments(ctx context.Context, span sweepSpanPayload) (massmoment.Moments, [3][3]*big.Rat, [3]*big.Rat, error) {
	if span.arc {
		rp := span.revolve
		local, err := revolveVolumeMoments(ctx, rp)
		if err != nil {
			return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		rp.xform = r3.Identity()
		frame, err := massmoment.RevolveRotation(rp.frame, rp.ax.dU, rp.ax.dV, rp.xform)
		if err != nil {
			return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		origin, err := massmoment.RevolveAnchor(rp.frame, rp.ax.aU, rp.ax.aV)
		if err != nil {
			return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		return local, frame, origin, nil
	}
	pp := span.prism
	mid, err := massmoment.PrismMidLevel(pp.z0, pp.z1)
	if err != nil {
		return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	local, err := prismVolumeMoments(ctx, pp)
	if err != nil {
		return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	// prismVolumeMoments is about (0, 0, zm); the span's rigid motion is
	// anchored at the frame origin.
	local = massmoment.Shift(local, [3]*big.Rat{new(big.Rat), new(big.Rat), mid})
	pp.xform = r3.Identity()
	frame, err := massmoment.PrismRotation(pp.frame, pp.xform)
	if err != nil {
		return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	origin, ok := massmoment.ExactVec(pp.frame.Origin())
	if !ok {
		return massmoment.Moments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, fmt.Errorf("%w: sweep span frame is not finite", ErrNotFinite)
	}
	return local, frame, origin, nil
}
