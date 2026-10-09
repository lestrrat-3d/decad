package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"

	"github.com/lestrrat-3d/decad/internal/survey2d"

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

// extentAlong is the through-all stop's reading of the prism (stops.go): the
// extent interval along an arbitrary world direction g — the lifted linear
// functional point·g = origin·g + u·(U'·g) + v·(V'·g) + z·(N'·g), primes the
// placed directions, extremized over the region boundary and the sweep —
// beside the proven displacement extentBoundedAlong states for its two ends.
// The stop charges that displacement to the level it resolves and decides its
// own in-path test outside it, so a boundary extreme held by a bracket
// (docs/spline-design.md §6.2) still answers rather than refusing.
//
// The section displacement is NOT one of the terms this reading carries — it
// moves a coordinate IN the plane, and the interval is stated over the
// recorded section — so a prism holding one refuses instead
// (docs/prism-boolean-design.md §12). prismBoundsContext reads
// extentBoundedAlong directly and composes both terms into its own outward
// bound.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	if pp.sectionDelta != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, ErrUnsupported)
	}
	return pp.extentBoundedAlong(context.Background(), g, freeform.NewFreeformWork(), nil)
}

func (pp prismPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.extentAlongWork(ctx, g, freeform.NewFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, mirroring
// revolvePayload.extentAlongWork word for word: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement.
// clearance.go's payloadExtent is that consumer — its separating-plane
// short-circuit compares two bodies' intervals and simply loses the
// short-circuit where it cannot get an exact one — so a direction whose extreme
// only a bracket holds refuses here rather than publish a held coordinate as the
// one it denotes. Which candidate holds it does not matter and the refusal never
// names a kind: a free-form span's enclosure, a computed arc radius and a walked
// endpoint the record does not state all reach this wrapper the same way,
// through one nonzero bound. A through-all stop does not read through this wrapper:
// it consumes the bounded reading and charges the displacement to its own level
// (stops.go, docs/spline-design.md §6.4).
func (pp prismPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.extentBoundedAlong(ctx, g, work, nil)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: this prism's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong passes the payload's placement coefficients to
// prismextent.ExtentAlong. Its bound covers the section candidates, frame
// and placement rounding, and the final endpoint sums. A non-nil walks
// reuses the profile's resolved segments across all three box axes.
func (pp prismPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	return prismextent.ExtentAlong(ctx, prismextent.ExtentInput{
		Profile: pp.profile, Frame: pp.frame, Transform: pp.xform,
		Z0: pp.z0, Z1: pp.z1, Base: base, GU: gu, GV: gv, GZ: gz,
		Direction: g,
	}, work, walks, boundaryExtremesBoundedContext)
}

// prismPlacementCoeffAllow adapts the held prism frame for prismextent.
func prismPlacementCoeffAllow(pp prismPayload, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	return prismextent.PlacementCoeffAllow(pp.xform, pp.frame, g, base, gu, gv, gz, coordUpper, zUpper)
}

func prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	return prismextent.DecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper)
}

// prismBoundsContext adapts the internal three-axis reading to Box. A cached
// walk set is reused across axes; nil resolves each axis as before.
func prismBoundsContext(ctx context.Context, pp prismPayload, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (Box, error) {
	low, high, bound, err := prismextent.Bounds(ctx, work, walks, pp.sectionDelta, pp.axialDelta(), pp.extentBoundedAlong)
	if err != nil {
		return Box{}, err
	}
	return Box{
		Min:       low,
		Max:       high,
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// boundaryExtremesBoundedContext adapts recorded profile walks for prismextent.
func boundaryExtremesBoundedContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	if err := freeform.RequireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}
	if walks != nil && !walks.Matches(profile) {
		return 0, 0, 0, momentinput.ErrResolvedWalksMismatch
	}
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	counts := make([]int, len(loops))
	for li, loop := range loops {
		counts[li] = len(loop.Segments)
	}
	return prismextent.BoundaryExtremesBoundedContext(ctx, gu, gv, work, counts, func(li, si int) (survey2d.SegmentWalk, error) {
		return momentinput.ResolveOrRead(loops[li].Segments[si], work, walks, li, si)
	})
}
