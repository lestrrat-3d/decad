package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// maxArrangeAngles is how many turns [ArrangeCommand.Angle] tries for each body, a
// quarter turn cut into steps, so a step finer than a quarter degree is refused.
const maxArrangeAngles = 360

// Angle lets [Arrange] turn each body about +Z to the angle that
// gives its bounding box the smallest footprint, so a long part lying across the
// bed's diagonal is laid nearly square to it (agent-docs/arrange-design.md §6).
// Angles are the multiples of step from zero up to a quarter turn, because a
// turn by a further quarter swaps a footprint's width and depth and keeps its
// area. The smallest footprint area wins, and the smallest angle among equals, so
// a part already square to the axes does not turn.
//
// The footprint of each trial is read from a copy of the body turned by that
// angle and retired again, so the bounds are the turned body's own, grown by the
// bound of its box. A chosen turn is a [Transform] about the body's bounding box
// centre before the move, a second [OperationTransform] entry. It combines with
// [ArrangeCommand.Turns], whose quarter turns then act on the turned footprints.
// Each trial angle also records one [OperationCopy] entry that the probe's
// retirement leaves without a Remove entry (agent-docs/arrange-design.md §6).
//
// step is a positive angle no finer than a quarter degree: a wrong unit kind is
// ErrUnitKind, a negative step ErrNegativeMagnitude, and a zero or too fine step
// ErrDegenerate. Do returns that fault naming Angle as its subject. A body that
// cannot be copied turned carries [Copy]'s refusal. A second call replaces the
// first.
func (c *ArrangeCommand) Angle(step units.Value) *ArrangeCommand {
	c.angle.Set(arrangeAngleStep(step))
	return c
}

// arrangeAngleStep validates step and returns it in radians.
func arrangeAngleStep(step units.Value) (float64, error) {
	if step.Kind() != units.Angle {
		return 0, fmt.Errorf(`%w: the arrangement angle step must be an angle`, ErrUnitKind)
	}
	rad := step.Base()
	switch {
	case math.IsNaN(rad) || math.IsInf(rad, 0):
		return 0, fmt.Errorf(`%w: the arrangement angle step is not finite`, ErrNotFinite)
	case rad < 0:
		return 0, fmt.Errorf(`%w: the arrangement angle step is negative`, ErrNegativeMagnitude)
	case rad == 0 || math.Pi/2/rad > maxArrangeAngles:
		return 0, fmt.Errorf(`%w: the arrangement angle step must be at least a quarter degree`, ErrDegenerate)
	}
	return rad, nil
}

// footprintTurned is the angle in [0, π/2) among the multiples of step that gives b
// the smallest bounding-box footprint, with that footprint and the box's bound,
// all in millimetres. Each trial is a retired copy of b.
func footprintTurned(ctx context.Context, b *Body, step float64) (angle, width, depth, margin float64, err error) {
	box, err := b.Bounds()
	if err != nil {
		return 0, 0, 0, 0, err
	}
	centre := box.Min.Add(box.Max).Scale(0.5)
	best := math.Inf(1)
	for k := 0; float64(k)*step < math.Pi/2-1e-12; k++ {
		if err := ctx.Err(); err != nil {
			return 0, 0, 0, 0, err
		}
		theta := float64(k) * step
		var w, d, m float64
		if k == 0 {
			if m, err = boundMillimetres(b); err != nil {
				return 0, 0, 0, 0, err
			}
			w, d = box.Max.X-box.Min.X+2*m, box.Max.Y-box.Min.Y+2*m
		} else {
			motion, cerr := rotationMotion(centre, r3.Vec{Z: 1}, units.Radians(theta))
			if cerr != nil {
				return 0, 0, 0, 0, cerr
			}
			c, cerr := copyBody(ctx, b, motion)
			if cerr != nil {
				return 0, 0, 0, 0, cerr
			}
			cb, berr := c.Bounds()
			if m, err = boundMillimetres(c); err == nil {
				err = berr
			}
			removeTools([]*Body{c})
			if err != nil {
				return 0, 0, 0, 0, err
			}
			w, d = cb.Max.X-cb.Min.X+2*m, cb.Max.Y-cb.Min.Y+2*m
		}
		if w*d < best {
			best, angle, width, depth, margin = w*d, theta, w, d, m
		}
	}
	return angle, width, depth, margin, nil
}

// turnAbout turns b by angle radians counter-clockwise about +Z through its
// bounding box centre.
func turnAbout(ctx context.Context, b *Body, angle float64) (*Body, error) {
	box, err := b.Bounds()
	if err != nil {
		return nil, err
	}
	t, err := r3.RotationAround(box.Min.Add(box.Max).Scale(0.5), r3.Vec{Z: 1}, units.Radians(angle))
	if err != nil {
		return nil, fmt.Errorf(`%w: the turn of a body is not finite: %s`, ErrNotFinite, err)
	}
	return transform(ctx, b, t, OperationTransform)
}
