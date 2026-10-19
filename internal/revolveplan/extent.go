package revolveplan

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismextent"
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

// ExtentAlong is the through-all stop's reading of the revolved solid
// (stops.go): its extent interval along an arbitrary world direction g, beside
// the proven displacement ExtentBoundedAlong states for its two ends. The
// swept radial factor's range over the angular interval turns the extreme into
// a linear functional over the recorded boundary in (z, ρ) — ρ ≥ 0 over the
// region, so the solid's extreme along g is the extreme of wg·z + m·ρ with m at
// its own extreme, and a linear functional's extreme sits on the boundary. An
// end a sweep extreme or a computed arc radius holds only to a bracket
// publishes that bracket's width, and the stop charges it to the level it
// resolves (docs/evaluator-design.md §5/§6).
func (rp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	return rp.ExtentBoundedAlong(context.Background(), g, freeform.NewFreeformWork())
}

// ExtentAlongContext is extentAlongWork under a fresh free-form work counter.
func (rp Payload) ExtentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return rp.extentAlongWork(ctx, g, freeform.NewFreeformWork())
}

// extentAlongWork is ExtentBoundedAlong's refusing wrapper, the same shape
// prismrecord.Payload.ExtentAlongWork already takes: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement —
// pairkernel's separating-plane short-circuit, which falls back rather than
// fails. A direction whose extreme is held to a bracket rather than proven
// exactly — by the boundary scan, or by the sweep extreme a partial revolution
// reaches through math.Sin/Cos — refuses here rather than publish a held
// coordinate as the one it denotes. A through-all stop instead consumes the
// bounded reading and charges the displacement to its own level (stops.go).
func (rp Payload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, error) {
	lo, hi, bound, err := rp.ExtentBoundedAlong(ctx, g, work)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: the revolved solid's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, decaderr.ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// ExtentBoundedAlong adapts the payload to revolveaxis.ExtentAlong. The
// internal reader composes the boundary, sweep, frame and placement bounds
// for both interval ends before any consumer uses them.
func (rp Payload) ExtentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	return rp.extentBoundedAlongProfile(ctx, g, work, nil)
}

func (rp Payload) extentBoundedAlongProfile(
	ctx context.Context, g r3.Vec, work *freeform.FreeformWork, profile *revolveaxis.ExtentProfile,
) (float64, float64, float64, error) {
	return revolveaxis.ExtentAlong(ctx, rp.extentInput(), g, work, profile, prismextent.ProfileBoundaryExtremes)
}

func (rp Payload) extentInput() revolveaxis.ExtentInput {
	b := rp.Basis()
	return revolveaxis.ExtentInput{
		Profile: rp.Profile, Axis: rp.Ax, SectionDelta: rp.SectionDelta,
		Phi0: rp.Phi0, Phi1: rp.Phi1, Full: rp.Full, Denotation: rp.Den,
		Transform: rp.Xform, Basis: b, Lift: rp.Lift(),
	}
}

// Bounds adapts the shared three-axis extent reading to measurement.Box.
// Each axis's bound includes its full displacement before the box takes the
// largest, so the box and through-all stops use the same extent proof.
func Bounds(ctx context.Context, rp Payload, work *freeform.FreeformWork) (measurement.Box, error) {
	low, high, bound, err := revolveaxis.Bounds(ctx, rp.extentInput(), work, prismextent.ProfileBoundaryExtremes)
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
