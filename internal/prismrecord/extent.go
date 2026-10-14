package prismrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file answers the extent questions asked OF a finished prism: how far
// the solid reaches along a direction, and the axis-aligned box that contains
// it.
//
// Every answer is a bounded interval, not a float: the coefficients of the
// direction in the payload's own frame round, the section and axial
// displacements move the boundary the interval is read from, and a boundary
// extreme riding a walked endpoint or a computed arc radius carries that
// walk's own bound. A direction whose extreme cannot be bracketed refuses
// rather than publishing the held value. See docs/evaluator-design.md §5.

// ExtentAlong is the through-all stop's reading of the prism (root stops.go): the
// extent interval along an arbitrary world direction g — the lifted linear
// functional point·g = origin·g + u·(U'·g) + v·(V'·g) + z·(N'·g), primes the
// placed directions, extremized over the region boundary and the sweep —
// beside the proven displacement ExtentBoundedAlong states for its two ends.
// The stop charges that displacement to the level it resolves and decides its
// own in-path test outside it, so a boundary extreme held by a bracket
// (docs/spline-design.md §6.2) still answers rather than refusing.
//
// The section displacement is NOT one of the terms this reading carries — it
// moves a coordinate IN the plane, and the interval is stated over the
// recorded section — so a prism holding one refuses instead
// (docs/prism-boolean-design.md §12). Bounds reads
// ExtentBoundedAlong directly and composes both terms into its own outward
// bound.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	if pp.SectionDelta != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, decaderr.ErrUnsupported)
	}
	return pp.ExtentBoundedAlong(context.Background(), g, freeform.NewFreeformWork(), nil)
}

// ExtentAlongContext is ExtentAlongWork with a fresh free-form counter.
func (pp Payload) ExtentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.ExtentAlongWork(ctx, g, freeform.NewFreeformWork())
}

// ExtentAlongWork is ExtentBoundedAlong's refusing wrapper, mirroring
// revolvePayload.ExtentAlongWork word for word: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement.
// root clearance.go's payloadExtent is that consumer — its separating-plane
// short-circuit compares two bodies' intervals and simply loses the
// short-circuit where it cannot get an exact one — so a direction whose extreme
// only a bracket holds refuses here rather than publish a held coordinate as the
// one it denotes. Which candidate holds it does not matter and the refusal never
// names a kind: a free-form span's enclosure, a computed arc radius and a walked
// endpoint the record does not state all reach this wrapper the same way,
// through one nonzero bound. A through-all stop does not read through this wrapper:
// it consumes the bounded reading and charges the displacement to its own level
// (root stops.go, docs/spline-design.md §6.4).
func (pp Payload) ExtentAlongWork(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.ExtentBoundedAlong(ctx, g, work, nil)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: this prism's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, decaderr.ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// ExtentBoundedAlong passes the payload's placement coefficients to
// prismextent.ExtentAlong. Its bound covers the section candidates, frame
// and placement rounding, and the final endpoint sums. A non-nil walks
// reuses the profile's resolved segments across all three box axes.
func (pp Payload) ExtentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	base := pp.Xform.Apply(pp.Frame.Origin()).Dot(g)
	gu := pp.Dir(1, 0, 0).Dot(g)
	gv := pp.Dir(0, 1, 0).Dot(g)
	gz := pp.Dir(0, 0, 1).Dot(g)
	return prismextent.ExtentAlong(ctx, prismextent.ExtentInput{
		Profile: pp.Profile, Frame: pp.Frame, Transform: pp.Xform,
		Z0: pp.Z0, Z1: pp.Z1, Base: base, GU: gu, GV: gv, GZ: gz,
		Direction: g,
	}, work, walks, prismextent.ProfileBoundaryExtremes)
}

// PlacementCoeffAllow adapts the held prism frame for prismextent.
func PlacementCoeffAllow(pp Payload, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	return prismextent.PlacementCoeffAllow(pp.Xform, pp.Frame, g, base, gu, gv, gz, coordUpper, zUpper)
}

// Bounds adapts the three-axis reading to a measurement.Box. A cached
// walk set is reused across axes; nil resolves each axis as before.
func Bounds(ctx context.Context, pp Payload, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (measurement.Box, error) {
	low, high, bound, err := prismextent.Bounds(ctx, work, walks, pp.SectionDelta, pp.AxialDelta(), pp.ExtentBoundedAlong)
	if err != nil {
		return measurement.Box{}, err
	}
	return measurement.Box{
		Min:       low,
		Max:       high,
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}
