package decad

import (
	"context"
	"fmt"

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

// Thicken builds a solid from an admitted sheet and retires that sheet.
// The first arm extrudes a recorded planar patch through the signed thickness
// interval. Other sheet families are staged under docs/surface-design.md §16.
func (b *Body) Thicken(ctx context.Context, thickness units.Value, opts ...ThickenOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a thicken`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
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
	tmm, tDelta, err := magnitudeInBounded(thickness, units.Length, units.Millimeter, "the thicken thickness")
	if err != nil {
		return nil, err
	}
	if tmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-thickness sheet encloses no solid`, ErrDegenerate)
	}
	pp, ok := b.payload.(patchPayload)
	if !ok || b.Kind() != BodySheet {
		return nil, fmt.Errorf(`%w: this evaluator thickens a recorded planar patch only`, ErrUnsupported)
	}
	level := measuredScalar(tmm, tDelta)
	var z0, z1 boundedScalar
	switch side {
	case ThickenPositive:
		z1 = level
	case ThickenNegative:
		z0 = boundedNeg(level)
	case ThickenCentered:
		half := boundedQuotient(level.value, level.bound, 2, 0)
		z0, z1 = boundedNeg(half), half
	}
	if admitAbove(boundedSub(z1, z0), 0) != survAdmit {
		return nil, fmt.Errorf(`%w: the thicken interval has no proven positive height`, ErrUnsupported)
	}
	prism := pp.prism()
	prism.z0, prism.z0Delta = z0.value, z0.bound
	prism.z1, prism.z1Delta = z1.value, z1.bound
	ref := d.nextProducerID()
	result, err := evalPrismContext(ctx, d, ref, prism, newFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(result, b)
	return result, nil
}
