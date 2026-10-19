package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// splitPointPlane is a separate Split arm. It handles only authenticated
// point-loft fans and one exact chain-sheet class; other pairs continue to
// the shared-generator Split dispatcher.
func (d *Document) splitPointPlane(ctx context.Context, budget *proofbound.WorkBudget,
	target, tool *Body) ([]*Body, bool, error) {
	record, ok := facetedrecord.PointPlaneSource(&target.n)
	if !ok {
		return nil, false, nil
	}
	plane, ok := facetedrecord.PointPlaneSheet(&tool.n, record.Base.PointSection.Apex)
	if !ok {
		return nil, false, nil
	}
	if err := facetedrecord.ValidatePointPlane(record.Base, plane); err != nil {
		return nil, true, err
	}
	if center, proved, err := facetedrecord.PointPlaneCapCentroid(ctx, record.Base, plane); err != nil {
		return nil, true, err
	} else {
		plane.Centroid, plane.HasCentroid = center, proved
	}
	if record.Lower != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, record.Lower, plane) {
		case -1:
			// The new plane is strictly beyond the existing lower cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's lower plane", ErrUnsupported)
		}
	}
	if record.Upper != nil {
		switch facetedrecord.PointPlaneOrder(record.Base, plane, record.Upper) {
		case -1:
			// The new plane is strictly before the existing upper cap.
		case 1:
			return nil, true, fmt.Errorf("%w: the split sheet misses this point-loft slab", ErrDegenerate)
		default:
			return nil, true, fmt.Errorf("%w: the split plane meets or crosses the slab's upper plane", ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return nil, true, err
	}
	ref := d.nextProducerID()
	inner := record
	inner.Upper = plane
	outer := record
	outer.Lower = plane
	first, err := bodyResult(facetedrecord.BuildPointPlaneSlab(ctx,
		envOf(documentNode(d), ref), inner))
	if err != nil {
		return nil, true, fmt.Errorf("point-loft inner slab: %w", err)
	}
	second, err := bodyResult(facetedrecord.BuildPointPlaneSlab(ctx,
		envOf(documentNode(d), ref+1), outer))
	if err != nil {
		return nil, true, fmt.Errorf("point-loft outer slab: %w", err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	pieces := []*Body{first, second}
	d.commitMany(pieces, target, tool)
	return pieces, true, nil
}
