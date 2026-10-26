package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/units"
)

// ThickenOption configures which side of a sheet receives material.
type ThickenOption interface{ thickenOption() }

type thickenSideOption struct{ side ThickenSide }

func (thickenSideOption) thickenOption() {}

// ThickenSide names the side of the source sheet that receives material.
type ThickenSide int

const (
	// ThickenPositive grows along the sheet's positive normal and is the default.
	ThickenPositive ThickenSide = iota
	// ThickenNegative grows opposite the sheet's positive normal.
	ThickenNegative
	// ThickenCentered grows half the thickness on each side.
	ThickenCentered
)

// WithThickenSide selects the side that receives the material.
func WithThickenSide(side ThickenSide) ThickenOption { return thickenSideOption{side: side} }

// Thicken builds a solid from the admitted sheet b and retires b.
// A nil context or a nil b is ErrDegenerate.
// It extrudes a recorded planar patch through the signed thickness interval,
// builds a certified annular wall around a profile-fed prism sheet, spins a
// certified meridian annulus through a profile-fed revolve sheet's own
// interval, or sweeps a ribbon's own assembled section through the ribbon's
// interval and spins a chain shell's own through the shell's. A one-span
// sweep sheet uses its prism or revolve reduction and restores sweep roles.
// A hole-free multi-span sweep sheet re-sweeps one generated annulus along its
// path, after a radial gate on each arc span.
// A one-arc chain sweep uses its chain Revolve reduction; a cardinal composite
// chain sweep assembles one section and sweeps both spans.
// Other sheet families are staged under agent-docs/surface-design.md §16.8.
func Thicken(ctx context.Context, b *Body, thickness units.Value,
	opts ...ThickenOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationThicken, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a thicken`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	side := ThickenPositive
	for _, raw := range opts {
		o, ok := raw.(thickenSideOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the thicken option is not a decad thicken option (%T)`, ErrDegenerate, raw)
		}
		if o.side < ThickenPositive || o.side > ThickenCentered {
			return nil, fmt.Errorf(`%w: unknown thicken side %d`, ErrDegenerate, o.side)
		}
		side = o.side
	}
	tmm, tDelta, err := extent.MagnitudeInBounded(thickness, units.Length, units.Millimeter, "the thicken thickness")
	if err != nil {
		return nil, err
	}
	if tmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-thickness sheet encloses no solid`, ErrDegenerate)
	}
	if b.Kind() != BodySheet {
		return nil, fmt.Errorf(`%w: Thicken requires a sheet body`, ErrUnsupported)
	}
	node, err := thickenbuild.Thicken(ctx, envOf(b.n.Doc, b.n.Doc.NextProducer), &b.n, thickenbuild.Side(side), tmm, tDelta)
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	result = bodyHandle(node)
	if err := d.commitOperationIn(ctx, nil, OperationThicken,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputConsumed}}, result, b); err != nil {
		return nil, err
	}
	return result, nil
}
