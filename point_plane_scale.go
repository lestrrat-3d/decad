package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ScaledAt returns the command that scales b uniformly about anchor. Do
// returns the positive uniform image of an authenticated point-loft
// Split slab, exact line/circular solid prism, line-only stack or stack-backed
// BRep, solid Revolve (complete or partial), audited faceted Boolean result,
// or other faceted solid whose held mesh passes its shell and crossing audit
// about anchor, then retires b. A nil context or a nil b is ErrDegenerate.
// Source roles and certified boundary and occupied-volume bounds follow the
// image. A faceted Boolean result keeps the retained Plane, Cylinder, Sphere
// and Torus surfaces of its faces, scaled with the body; a retained Cone or
// denoted face reads Faceted. Another faceted solid follows the same rule:
// plane faces and retained Cylinder, Sphere and Torus faces keep their scaled
// surfaces, and retained Cone, denoted and unretained curved faces read
// Faceted. A refused or canceled call leaves the document unchanged.
func ScaledAt(b *Body, anchor r3.Vec, factor units.Value) *ScaledAtCommand {
	c := &ScaledAtCommand{b: b, anchor: anchor, factor: factor}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// ScaledAtCommand scales a body uniformly about a point. Build it with
// [ScaledAt].
type ScaledAtCommand struct {
	state  opcommand.State // consumed flag and first recorded fault
	b      *Body           // stored as given
	anchor r3.Vec          // the fixed point, gated at Do
	factor units.Value     // the factor, gated at Do
}

// Do scales the body and retires it, or returns the error of the first gate
// that refuses. A nil ctx is ErrDegenerate, and a second call returns
// ErrConsumed.
func (c *ScaledAtCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationScaledAt, c.run)
}

func (c *ScaledAtCommand) run(ctx context.Context) (*Body, error) {
	return scaledAt(ctx, c.b, c.anchor, c.factor)
}

// scaledAt is ScaledAt's implementation. ctx is not nil, and the errors it
// returns carry no operation tag (agent-docs/command-api-design.md §11).
func scaledAt(ctx context.Context, b *Body, anchor r3.Vec, factor units.Value) (*Body, error) {
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
	ref := d.producerAt(ctx)
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
