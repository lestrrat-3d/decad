package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/units"
)

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

// Thicken builds a solid from the admitted sheet b and retires b.
// A nil context given to Do or a nil b is ErrDegenerate.
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
func Thicken(b *Body, thickness units.Value) *ThickenCommand {
	return &ThickenCommand{b: b, thickness: thickness}
}

// ThickenCommand builds a solid from a sheet. Build it with [Thicken].
type ThickenCommand struct {
	state     opcommand.State             // consumed flag and first recorded fault
	b         *Body                       // stored as given
	thickness units.Value                 // the thickness, gated at Do
	side      opcommand.Slot[ThickenSide] // set by Side
}

// Side selects the side that receives the material. Without it the side is
// [ThickenPositive]. A value that is no side is ErrDegenerate, returned by Do.
// A second call replaces the first.
func (c *ThickenCommand) Side(side ThickenSide) *ThickenCommand {
	var err error
	if side < ThickenPositive || side > ThickenCentered {
		err = fmt.Errorf(`%w: unknown thicken side %d`, ErrDegenerate, side)
	}
	c.side.Set(side, err)
	return c
}

// Do builds the solid and retires the sheet, or returns the error of the
// first gate that refuses, leaving the sheet live. A nil ctx is ErrDegenerate,
// and a second call returns ErrConsumed.
func (c *ThickenCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationThicken, c.run)
}

func (c *ThickenCommand) run(ctx context.Context) (*Body, error) {
	side, sideErr := c.side.Get()
	return thicken(ctx, c.b, c.thickness, side, sideErr)
}

// thicken is Thicken's implementation. ctx is not nil, and the errors it
// returns carry no operation tag (agent-docs/command-api-design.md §11).
// sideErr is the fault of the Side call.
func thicken(ctx context.Context, b *Body, thickness units.Value, side ThickenSide, sideErr error) (*Body, error) {
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
	if sideErr != nil {
		return nil, subjectError(OperationThicken, "Side", sideErr)
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
	node, err := thickenbuild.Thicken(ctx, envOf(b.n.Doc, documentHandle(b.n.Doc).producerID(ctx)), &b.n, thickenbuild.Side(side), tmm, tDelta)
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	result := bodyHandle(node)
	if err := d.commitOperationIn(ctx, nil, OperationThicken,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputConsumed}}, result, b); err != nil {
		return nil, err
	}
	return result, nil
}
