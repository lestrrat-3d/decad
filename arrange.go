package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/arrangeplan"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ArrangeBed is the flat area [Arrange] fills. Origin is the world point of the
// bed's minimum corner and its surface height. Width runs along X from Origin
// and Depth along Y. Gap is the clear distance kept between any two bodies and
// between a body and the bed's edge. Width and Depth are positive lengths and
// Gap is a non-negative length.
type ArrangeBed struct {
	Origin       r3.Vec
	Width, Depth units.Value
	Gap          units.Value
	// Height, when set, lets bodies that do not fit the bed's surface stack in
	// layers above one another: each layer rests Gap above the tallest body of
	// the one below, and all of them stay within Height of the surface
	// (agent-docs/arrange-design.md §7). The zero Value is a single layer.
	Height units.Value
}

// Arrange returns the command that lays bodies side by side on bed. Do moves
// bodies by translation so they lie side by side on bed, and returns the moved
// bodies in the order given, retiring the inputs (agent-docs/arrange-design.md). Each body's bounding box, grown on every side
// by the box's own error bound, rests on the bed's surface (its minimum Z is
// Origin.Z) and keeps Gap from the others and from the bed's edge. Bodies are
// placed deepest first in shelves along X. Arrange turns nothing unless
// [ArrangeCommand.Turns] or [ArrangeCommand.Angle] is called, so a body keeps
// its orientation. Arrange copies bodies when it is called, so a later write to
// the caller's slice changes nothing.
//
// bodies MUST be live, distinct and belong to one document, and must not be
// empty or hold a nil first body. A violation is ErrDegenerate, ErrForeignBody
// or ErrRetiredBody. A bed with a non-positive size or a negative gap is
// ErrDegenerate, a wrong unit kind is ErrUnitKind, and bodies that do not all
// fit on the bed are ErrDegenerate; no body moves in any of these cases. A
// failed or canceled call leaves the document as it found it: every body live
// at its own place, and no history entry or producer number added. Every error
// carries [OperationTransform].
func Arrange(bodies []*Body, bed ArrangeBed) *ArrangeCommand {
	c := &ArrangeCommand{bodies: slices.Clone(bodies), bed: bed}
	switch {
	case len(bodies) == 0:
		c.state.Record("bodies", errArrangeEmpty)
	case bodies[0] == nil:
		c.state.Record("bodies", errArrangeNilFirst)
	}
	return c
}

var (
	errArrangeEmpty    = fmt.Errorf(`%w: an arrangement needs at least one body`, ErrDegenerate)
	errArrangeNilFirst = fmt.Errorf(`%w: the first body belongs to no document`, ErrDegenerate)
)

// ArrangeCommand lays bodies side by side on a bed. Build it with [Arrange].
type ArrangeCommand struct {
	state  opcommand.State         // consumed flag and first recorded fault
	bodies []*Body                 // copied at Arrange
	bed    ArrangeBed              // the bed, gated at Do
	turns  bool                    // set by Turns
	angle  opcommand.Slot[float64] // set by Angle, in radians, with the fault of its validation
}

// Turns lets [Arrange] turn a body a quarter turn about +Z to fit, which swaps
// its footprint's width and depth. A body that is deeper than wide is laid with
// its long side along X when that lets all of them fit, and no body turns
// otherwise. A turned body is rotated about the minimum corner of its bounding
// box and then moved, so it adds a second [OperationTransform] entry to the
// history. Calling Turns again changes nothing.
func (c *ArrangeCommand) Turns() *ArrangeCommand {
	c.turns = true
	return c
}

// Do arranges the bodies, or returns the error of the first gate that refuses
// with the document unchanged. A nil ctx is ErrDegenerate, and a second call
// returns ErrConsumed.
func (c *ArrangeCommand) Do(ctx context.Context) ([]*Body, error) {
	return runCommand(ctx, &c.state, OperationTransform, c.run)
}

func (c *ArrangeCommand) run(ctx context.Context) ([]*Body, error) {
	step, err := c.angle.Get()
	return arrange(ctx, c.bodies, c.bed, c.turns, arrangeAngle{step: step, err: err, subject: "Angle"})
}

// arrangeAngle is the folded form of an arrangement's Angle method. err is the
// fault of the step, refused where the option decode refused it, and subject
// names what the diagnostic points at (agent-docs/command-api-design.md §11).
type arrangeAngle struct {
	step    float64 // radians, zero when absent
	err     error
	subject string
}

// arrange is Arrange's implementation. ctx is not nil, and the errors it
// returns carry no operation tag, except one that names a method
// (agent-docs/command-api-design.md §11). Its snapshot and restore sit after the
// gates, so a call a gate refuses never takes a snapshot.
func arrange(ctx context.Context, bodies []*Body, bed ArrangeBed, turns bool, angle arrangeAngle) (result []*Body, retErr error) {
	if len(bodies) == 0 {
		return nil, errArrangeEmpty
	}
	if bodies[0] == nil || bodies[0].n.Doc == nil {
		return nil, errArrangeNilFirst
	}
	if angle.err != nil {
		return nil, subjectError(OperationTransform, angle.subject, angle.err)
	}
	angleStep := angle.step
	d := documentHandle(bodies[0].n.Doc)
	seen := make(map[*Body]struct{}, len(bodies))
	for _, b := range bodies {
		if err := d.requireLive(b); err != nil {
			return nil, err
		}
		if _, dup := seen[b]; dup {
			return nil, fmt.Errorf(`%w: a body appears twice in the arrangement`, ErrDegenerate)
		}
		seen[b] = struct{}{}
	}
	width, err := sectionrecord.MagnitudeIn(bed.Width, units.Length, units.Millimeter, "the bed width")
	if err != nil {
		return nil, err
	}
	depth, err := sectionrecord.MagnitudeIn(bed.Depth, units.Length, units.Millimeter, "the bed depth")
	if err != nil {
		return nil, err
	}
	gap, err := sectionrecord.MagnitudeIn(bed.Gap, units.Length, units.Millimeter, "the bed gap")
	if err != nil {
		return nil, err
	}
	if width == 0 || depth == 0 {
		return nil, fmt.Errorf(`%w: the bed needs a positive width and depth`, ErrDegenerate)
	}
	layered := bed.Height != (units.Value{})
	var limit float64
	if layered {
		if limit, err = sectionrecord.MagnitudeIn(bed.Height, units.Length, units.Millimeter, "the bed height"); err != nil {
			return nil, err
		}
		if limit == 0 {
			return nil, fmt.Errorf(`%w: the bed height must be positive`, ErrDegenerate)
		}
	}
	snap := d.snapshot(ctx)
	defer func() {
		if retErr != nil {
			d.restore(snap)
		}
	}()
	items := make([]arrangeplan.Item, len(bodies))
	heights := make([]float64, len(bodies))
	margins := make([]float64, len(bodies))
	angles := make([]float64, len(bodies))
	for i, b := range bodies {
		if angleStep > 0 {
			var w, dp float64
			if angles[i], w, dp, margins[i], err = footprintTurned(ctx, b, angleStep); err != nil {
				return nil, err
			}
			items[i] = arrangeplan.Item{Width: w, Depth: dp}
			heights[i] = zExtent(b, margins[i])
			continue
		}
		box, err := b.Bounds()
		if err != nil {
			return nil, err
		}
		margin, err := box.Bound.In(units.Millimeter)
		if err != nil {
			return nil, fmt.Errorf(`%w: the bounds of a body are not representable: %s`, ErrNotFinite, err)
		}
		margins[i] = margin
		items[i] = arrangeplan.Item{
			Width: box.Max.X - box.Min.X + 2*margin,
			Depth: box.Max.Y - box.Min.Y + 2*margin,
		}
		heights[i] = box.Max.Z - box.Min.Z + margin
	}
	var positions [][2]float64
	var turned []bool
	var ok bool
	zBase := make([]float64, len(bodies))
	if layered {
		var layout arrangeplan.Layout
		layout, ok = arrangeplan.PackLayers(items, heights, arrangeplan.Bed{Width: width, Depth: depth, Gap: gap}, limit, turns)
		positions, turned, zBase = layout.Positions, layout.Turned, layout.ZBase
	} else if turns {
		positions, turned, ok = arrangeplan.PackTurning(items, arrangeplan.Bed{Width: width, Depth: depth, Gap: gap})
	} else {
		positions, ok = arrangeplan.Pack(items, arrangeplan.Bed{Width: width, Depth: depth, Gap: gap})
	}
	if !ok {
		if layered {
			return nil, fmt.Errorf(`%w: the %d bodies do not fit in layers on a %g by %g mm bed %g mm high with a %g mm gap`,
				ErrDegenerate, len(bodies), width, depth, limit, gap)
		}
		return nil, fmt.Errorf(`%w: the %d bodies do not fit on a %g by %g mm bed with a %g mm gap`,
			ErrDegenerate, len(bodies), width, depth, gap)
	}
	moved := make([]*Body, len(bodies))
	for i, b := range bodies {
		if angles[i] != 0 {
			var err error
			if b, err = turnAbout(ctx, b, angles[i]); err != nil {
				return nil, err
			}
			if margins[i], err = boundMillimetres(b); err != nil {
				return nil, err
			}
		}
		if turned != nil && turned[i] {
			var err error
			if b, err = turnQuarter(ctx, b); err != nil {
				return nil, err
			}
			// The turned body is the one to place; the input is retired.
			margins[i], err = boundMillimetres(b)
			if err != nil {
				return nil, err
			}
		}
		box, err := b.Bounds()
		if err != nil {
			return nil, err
		}
		shift := r3.Vec{
			X: bed.Origin.X + positions[i][0] + margins[i] - box.Min.X,
			Y: bed.Origin.Y + positions[i][1] + margins[i] - box.Min.Y,
			Z: bed.Origin.Z + zBase[i] - box.Min.Z,
		}
		t, err := r3.Translation(shift)
		if err != nil {
			return nil, fmt.Errorf(`%w: the move of a body is not finite: %s`, ErrNotFinite, err)
		}
		if moved[i], err = transform(ctx, b, t, OperationTransform); err != nil {
			return nil, err
		}
	}
	return moved, nil
}

// turnQuarter turns b a quarter turn counter-clockwise about +Z through the
// minimum corner of its bounding box.
func turnQuarter(ctx context.Context, b *Body) (*Body, error) {
	box, err := b.Bounds()
	if err != nil {
		return nil, err
	}
	t, err := r3.RotationAround(box.Min, r3.Vec{Z: 1}, units.Degrees(90))
	if err != nil {
		return nil, fmt.Errorf(`%w: the turn of a body is not finite: %s`, ErrNotFinite, err)
	}
	return transform(ctx, b, t, OperationTransform)
}

// boundMillimetres is the bound of b's bounding box in millimetres.
func boundMillimetres(b *Body) (float64, error) {
	box, err := b.Bounds()
	if err != nil {
		return 0, err
	}
	margin, err := box.Bound.In(units.Millimeter)
	if err != nil {
		return 0, fmt.Errorf(`%w: the bounds of a body are not representable: %s`, ErrNotFinite, err)
	}
	return margin, nil
}

// zExtent is b's extent along Z grown by the bound of its box, the height a layer
// of bodies must clear, or zero for a body whose bounds do not read.
func zExtent(b *Body, margin float64) float64 {
	box, err := b.Bounds()
	if err != nil {
		return 0
	}
	return box.Max.Z - box.Min.Z + margin
}
