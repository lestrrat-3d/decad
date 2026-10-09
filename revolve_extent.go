package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file answers the extent questions asked OF a finished revolve: how far
// the solid reaches along a direction, and the axis-aligned box containing it.
//
// A revolve's extreme along a direction is a sweep extreme, not a boundary
// vertex: the meridian's own extremes are swept through the angular interval,
// and revolveangle.ExtremeBounds brackets where that sweep turns. Every answer is a
// bounded interval charging the frame's rounding, the angular interval's own
// bound, and the meridian bound the walk carries. See
// docs/evaluator-design.md §6.

// extentAlong is the through-all stop's reading of the revolved solid
// (stops.go): its extent interval along an arbitrary world direction g, beside
// the proven displacement extentBoundedAlong states for its two ends. The
// swept radial factor's range over the angular interval turns the extreme into
// a linear functional over the recorded boundary in (z, ρ) — ρ ≥ 0 over the
// region, so the solid's extreme along g is the extreme of wg·z + m·ρ with m at
// its own extreme, and a linear functional's extreme sits on the boundary. An
// end a sweep extreme or a computed arc radius holds only to a bracket
// publishes that bracket's width, and the stop charges it to the level it
// resolves (docs/evaluator-design.md §5/§6).
func (rp revolvePayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	return rp.extentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

func (rp revolvePayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return rp.extentAlongWork(ctx, g, freeform.NewFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, the same shape
// prismPayload.extentAlongWork already takes: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement —
// clearance.go's separating-plane short-circuit, which falls back rather than
// fails. A direction whose extreme is held to a bracket rather than proven
// exactly — by the boundary scan, or by the sweep extreme a partial revolution
// reaches through math.Sin/Cos — refuses here rather than publish a held
// coordinate as the one it denotes. A through-all stop instead consumes the
// bounded reading and charges the displacement to its own level (stops.go).
func (rp revolvePayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, error) {
	lo, hi, bound, err := rp.extentBoundedAlong(ctx, g, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: the revolved solid's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong adapts the payload to revolveaxis.ExtentAlong. The
// internal reader composes the boundary, sweep, frame and placement bounds
// for both interval ends before any consumer uses them.
func (rp revolvePayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	return rp.extentBoundedAlongProfile(ctx, g, work, nil)
}

func (rp revolvePayload) extentBoundedAlongProfile(
	ctx context.Context, g r3.Vec, work *freeform.FreeformWork, profile *revolveaxis.ExtentProfile,
) (float64, float64, float64, error) {
	return revolveaxis.ExtentAlong(ctx, rp.extentInput(), g, work, profile, boundaryExtremesBoundedContext)
}

func (rp revolvePayload) extentInput() revolveaxis.ExtentInput {
	b := rp.basis()
	return revolveaxis.ExtentInput{
		Profile: rp.profile, Axis: rp.ax.numeric(), SectionDelta: rp.sectionDelta,
		Phi0: rp.phi0, Phi1: rp.phi1, Full: rp.full, Denotation: rp.den,
		Transform: rp.xform, Basis: b, Lift: rp.lift(),
	}
}

// revolveBoundsContext adapts the shared three-axis extent reading to Box.
// Each axis's bound includes its full displacement before the box takes the
// largest, so the box and through-all stops use the same extent proof.
func revolveBoundsContext(ctx context.Context, rp revolvePayload, work *freeform.FreeformWork) (Box, error) {
	low, high, bound, err := revolveaxis.Bounds(ctx, rp.extentInput(), work, boundaryExtremesBoundedContext)
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
