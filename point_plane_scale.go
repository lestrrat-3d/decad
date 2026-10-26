package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ScaledAt returns the positive uniform image of an authenticated point-loft
// Split slab, exact line/circular solid prism, line-only stack or stack-backed
// BRep, solid Revolve (complete or partial), or audited faceted Boolean result about anchor,
// then retires b. A nil context or a nil b is ErrDegenerate.
// Source roles and certified boundary and occupied-volume bounds follow the
// image. A faceted Boolean result keeps the retained Plane, Cylinder, Sphere
// and Torus surfaces of its faces, scaled with the body; a retained Cone or
// denoted face reads Faceted. A refused or canceled call leaves the document
// unchanged.
func ScaledAt(ctx context.Context, b *Body, anchor r3.Vec, factor units.Value) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationScaledAt, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a scale", ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf("%w: the body belongs to no document", ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	ctx, err := d.evalContext(ctx)
	if err != nil {
		return nil, err
	}
	if factor.Kind() != units.Dimensionless {
		return nil, fmt.Errorf("%w: the scale factor must be dimensionless", ErrUnitKind)
	}
	s := factor.Base()
	if proofbound.IsNonFinite(s) || !proofbound.FiniteVec(anchor) {
		return nil, fmt.Errorf("%w: scale factor and anchor must be finite", ErrNotFinite)
	}
	if s <= 0 {
		return nil, fmt.Errorf("%w: the scale factor must be positive", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	fp, ok := b.n.Payload.(facetedPayload)
	if !ok {
		var err error
		switch source := b.n.Payload.(type) {
		case prismPayload:
			fp, err = prismScaleSource(ctx, b, source, s)
		case revolvePayload:
			fp, err = revolveScaleSource(ctx, b, source, s)
		case stackedPrismPayload:
			fp, err = stackedScaleSource(ctx, b, source)
		case brepPayload:
			fp, err = brepScaleSource(ctx, b, source)
		default:
			return nil, fmt.Errorf("%w: this body has no certified similarity source", ErrUnsupported)
		}
		if err != nil {
			return nil, err
		}
	}
	next, repair, err := facetedrecord.ScaleImageWithSummary(ctx, &b.n, fp, anchor, s)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	scaled, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), next))
	if err != nil {
		return nil, err
	}
	scaled.n.OperationWarnings = append(scaled.n.OperationWarnings, repair.Diagnostics()...)
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, nil, OperationScaledAt,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputConsumed}}, scaled, b); err != nil {
		return nil, err
	}
	return scaled, nil
}
