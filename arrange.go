package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/arrangeplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// ArrangeOption configures [Arrange].
type ArrangeOption interface {
	option.Interface
	arrangeOption()
}

type arrangeOption struct{ option.Interface }

func (arrangeOption) arrangeOption() {}

type identArrangeTurns struct{}

// WithArrangeTurns lets [Arrange] turn a body a quarter turn about +Z to fit,
// which swaps its footprint's width and depth. A body that is deeper than wide
// is laid with its long side along X when that lets all of them fit, and no
// body turns otherwise. A turned body is rotated about the minimum corner of
// its bounding box and then moved, so it adds a second [OperationTransform]
// entry to the history.
func WithArrangeTurns() ArrangeOption {
	return arrangeOption{option.New(identArrangeTurns{}, true)}
}

// ArrangeBed is the flat area [Arrange] fills. Origin is the world point of the
// bed's minimum corner and its surface height. Width runs along X from Origin
// and Depth along Y. Gap is the clear distance kept between any two bodies and
// between a body and the bed's edge. Width and Depth are positive lengths and
// Gap is a non-negative length.
type ArrangeBed struct {
	Origin       r3.Vec
	Width, Depth units.Value
	Gap          units.Value
}

// Arrange moves bodies by translation so they lie side by side on bed, and
// returns the moved bodies in the order given, retiring the inputs
// (agent-docs/arrange-design.md). Each body's bounding box, grown on every side
// by the box's own error bound, rests on the bed's surface (its minimum Z is
// Origin.Z) and keeps Gap from the others and from the bed's edge. Bodies are
// placed deepest first in shelves along X. Arrange turns nothing unless
// [WithArrangeTurns] is given, so a body keeps its orientation.
//
// bodies MUST be live, distinct and belong to one document, and must not be
// empty or hold a nil. A violation is ErrDegenerate, ErrForeignBody or
// ErrRetiredBody. A bed with a non-positive size or a negative gap is
// ErrDegenerate, a wrong unit kind is ErrUnitKind, and bodies that do not all
// fit on the bed are ErrDegenerate; no body moves in any of these cases. A
// canceled context stops the moves and leaves the bodies already moved at
// their new places and the rest where they were.
func Arrange(ctx context.Context, bodies []*Body, bed ArrangeBed, opts ...ArrangeOption) (result []*Body, retErr error) {
	defer func() { retErr = operationError(OperationTransform, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an arrangement`, ErrDegenerate)
	}
	if len(bodies) == 0 {
		return nil, fmt.Errorf(`%w: an arrangement needs at least one body`, ErrDegenerate)
	}
	if bodies[0] == nil || bodies[0].n.Doc == nil {
		return nil, fmt.Errorf(`%w: the first body belongs to no document`, ErrDegenerate)
	}
	turns := false
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := o.Ident().(identArrangeTurns); ok {
			turns = true
		}
	}
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
	items := make([]arrangeplan.Item, len(bodies))
	margins := make([]float64, len(bodies))
	for i, b := range bodies {
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
	}
	var positions [][2]float64
	var turned []bool
	var ok bool
	if turns {
		positions, turned, ok = arrangeplan.PackTurning(items, arrangeplan.Bed{Width: width, Depth: depth, Gap: gap})
	} else {
		positions, ok = arrangeplan.Pack(items, arrangeplan.Bed{Width: width, Depth: depth, Gap: gap})
	}
	if !ok {
		return nil, fmt.Errorf(`%w: the %d bodies do not fit on a %g by %g mm bed with a %g mm gap`,
			ErrDegenerate, len(bodies), width, depth, gap)
	}
	moved := make([]*Body, len(bodies))
	for i, b := range bodies {
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
			Z: bed.Origin.Z - box.Min.Z,
		}
		t, err := r3.Translation(shift)
		if err != nil {
			return nil, fmt.Errorf(`%w: the move of a body is not finite: %s`, ErrNotFinite, err)
		}
		if moved[i], err = Transform(ctx, b, t); err != nil {
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
	return Transform(ctx, b, t)
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
