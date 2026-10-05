package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// unplaced coordinates through its own rigid motion: the exact rational image
// of its local origin and the rotation nearest its exact rational frame
// matrix, widened by that matrix's orthonormality defect
// (rotateVolumeMoments). The exact change of anchor (shiftVolumeMoments)
// carries the span's P and Q to the shared anchor, so no span is reduced to
// a centroidal tensor and moved by the parallel-axis rule. The summed
// moments then reach world axes through the one placement every span shares,
// as a single rotated solid (rigidMassProperties).

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
	var total volumeMoments
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
		unplaced := shiftVolumeMoments(rotateVolumeMoments(local, frame), offset)
		if i == 0 {
			total = unplaced
			continue
		}
		total = addVolumeMoments(total, unplaced)
	}
	rotation, err := placementRotation(placement)
	if err != nil {
		return MassProperties{}, err
	}
	return rigidMassProperties(ctx, b.centroid, total, rotation, density)
}

// sweepSpanMoments returns one span's local moments, the exact rational
// matrix taking its local axes to the composite's unplaced axes, and the
// exact unplaced position of its local origin.
func sweepSpanMoments(ctx context.Context, span sweepSpanPayload) (volumeMoments, [3][3]*big.Rat, [3]*big.Rat, error) {
	if span.arc {
		rp := span.revolve
		local, err := revolveVolumeMoments(ctx, rp)
		if err != nil {
			return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		rp.xform = r3.Identity()
		frame, err := revolveRotation(rp)
		if err != nil {
			return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		origin, err := revolveAnchor(rp)
		if err != nil {
			return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
		}
		return local, frame, origin, nil
	}
	pp := span.prism
	mid, err := prismMidLevel(pp)
	if err != nil {
		return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	local, err := prismVolumeMoments(ctx, pp)
	if err != nil {
		return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	// prismVolumeMoments is about (0, 0, zm); the span's rigid motion is
	// anchored at the frame origin.
	local = shiftVolumeMoments(local, [3]*big.Rat{new(big.Rat), new(big.Rat), mid})
	pp.xform = r3.Identity()
	frame, err := prismRotation(pp)
	if err != nil {
		return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, err
	}
	origin, ok := exactVec(pp.frame.Origin())
	if !ok {
		return volumeMoments{}, [3][3]*big.Rat{}, [3]*big.Rat{}, fmt.Errorf("%w: sweep span frame is not finite", ErrNotFinite)
	}
	return local, frame, origin, nil
}

// placementRotation is the exact rational linear part of a placement, its
// column k the image of world axis k, read from the held basis.
func placementRotation(placement r3.Transform) ([3][3]*big.Rat, error) {
	basis := placement.Basis()
	var out [3][3]*big.Rat
	for k, column := range []r3.Vec{basis.EX, basis.EY, basis.EZ} {
		exact, ok := exactVec(column)
		if !ok {
			return out, fmt.Errorf("%w: placement basis is not finite", ErrNotFinite)
		}
		for i := range exact {
			out[i][k] = exact[i]
		}
	}
	return out, nil
}

func exactVec(v r3.Vec) ([3]*big.Rat, bool) {
	out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	return out, out[0] != nil && out[1] != nil && out[2] != nil
}
