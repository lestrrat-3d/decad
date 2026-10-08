package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"

	"github.com/lestrrat-3d/decad/internal/survey2d"

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

// extentBoundedAlong is the reading itself: the interval AND the proven
// half-width its two ends carry. Both mechanisms that can move an end are
// charged here, because this is the ONE reading every consumer takes — the box
// and the through-all stop alike — and a term charged in only one of them lets
// the same coordinate publish as bounded on one path and exact on the other.
//
// The first mechanism is everything axisExtremeContext returns a bound for: the
// boundary-extreme scan's own — a circular candidate sits at a math.Hypot radius
// and can miss the apex the sweep actually reaches — the rounding that scan's
// own gu·u + gv·v arithmetic commits at the section's coordinate magnitude, and
// that reading's own anchor-shift arithmetic (its doc comment states all three).
// The second is the swept radial coefficient's, sweepBoundAlong below. A section
// whose every candidate is exactly representable, read through a direction whose
// own two products and their sum are exact, swept through an extreme this
// direction's own amplitude holds exactly and shifted by an anchor its own
// products and subtraction hold exactly, carries a zero bound — an all-straight
// or recorded-circle meridian under a full revolution about an axis-aligned
// frame reads exactly.
//
// The two mechanisms displace the SAME end and they COMPOSE, so this reading
// sums them; taking the larger would be sound only if they could not both move
// the end the same way, and nothing makes them exclusive. They are displacements
// of different quantities — the scan bounds how far the computed extreme of
// wg·z + m·ρ sits from the true extreme at the HELD coefficient m, while
// sweepBoundAlong bounds how far that held extreme sits from the extreme at the
// TRUE coefficient — so the triangle inequality is the only composition
// available, and the sum is what the end's total displacement obeys.
//
// The sum is per END. The scan's own bound belongs to the end that scan
// produced, and the sweep term belongs to the end whose coefficient it charges
// (mlo for the low end, mhi for the high end), so the two pairs are composed
// separately and the reading publishes the larger total — it states ONE
// half-width covering both ends, and the larger of two per-end totals covers
// each end's own displacement.
//
// A THIRD mechanism displaces both ends the same way and so composes outward
// with that per-end maximum rather than folding into it:
// revolvePayload.frameRoundAllow, base/wg/c0/c1's own displacement from the
// axis frame's proven direction/anchor uncertainty (axisInPlane's
// dUBound/dVBound/aUBound/aVBound — the fields axisMoments already folds into
// the region's moments) and from the placement's own rounding
// (proofbound.ExactIsometryDotRound, internal/proofbound/bounds.go). It is zero for an axis-aligned frame
// under an identity placement, which is what keeps an ordinary, unplaced
// revolve's box Exact as before.
//
// A FOURTH is this reading's own recombination of those terms into a published
// endpoint — the base + lo and base + hi below, charged exactly by
// proofbound.ExactSumRound. It covers what none of the other three does: a placement whose
// coefficients are every one of them exactly right, whose sum nonetheless
// rounds. It is zero wherever that addition is exactly representable, so an
// unplaced revolve's box keeps its zero bound.
func (rp revolvePayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeform.FreeformWork) (float64, float64, float64, error) {
	return rp.extentBoundedAlongProfile(ctx, g, work, nil)
}

// revolveExtentProfile shares plane-local walks and their coordinate envelope
// across the three axis reads of one bounds calculation. Only analytic walks
// enter this cache: free-form walks charge a per-read work budget.
type revolveExtentProfile struct {
	walks      *momentinput.ProfileWalks
	coordUpper float64
}

func (rp revolvePayload) extentBoundedAlongProfile(
	ctx context.Context, g r3.Vec, work *freeform.FreeformWork, profile *revolveExtentProfile,
) (float64, float64, float64, error) {
	b := rp.basis()
	base := rp.xform.Apply(b.A3).Dot(g)
	wg := rp.xform.ApplyDir(b.W).Dot(g)
	c0 := rp.xform.ApplyDir(b.E0).Dot(g)
	c1 := rp.xform.ApplyDir(b.E1).Dot(g)
	mlo, mhi := revolveangle.Extremes(c0, c1, rp.phi0, rp.phi1, rp.full)
	hi, hiBound, err := axisExtremeContext(ctx, rp, wg, mhi, true, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	lo, loBound, err := axisExtremeContext(ctx, rp, wg, mlo, false, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	sweepLo, sweepHi, err := rp.sweepBoundAlong(c0, c1, mlo, mhi, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	bound := revolveaxis.ExtentBoundarySweepBound(loBound, hiBound, sweepLo, sweepHi)
	frameAllow, err := rp.frameRoundAllow(g, b, base, wg, c0, c1, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	loEnd, hiEnd, bound := revolveaxis.FinishExtent(rp.ax.numeric(), rp.sectionDelta, base, lo, hi, bound, frameAllow)
	return loEnd, hiEnd, bound, nil
}

// frameRoundAllow resolves the profile envelope before the internal axis and
// placement proof reads the numeric frame and sweep coefficients.
func (rp revolvePayload) frameRoundAllow(
	g r3.Vec, b revolvemesh.RevolveBasis, base, wg, c0, c1 float64, work *freeform.FreeformWork, profile *revolveExtentProfile,
) (float64, error) {
	coordUpper := 0.0
	if profile != nil {
		coordUpper = profile.coordUpper
	} else {
		var err error
		coordUpper, err = profileCoordinateUpper(rp.profile, work, nil)
		if err != nil {
			return 0, err
		}
	}
	return revolveaxis.FrameRoundAllow(rp.ax.numeric(), rp.sectionDelta, rp.xform, g, b, base, wg, c0, c1, coordUpper), nil
}

// sweepBoundAlong resolves the profile envelope before the internal proof
// bounds the two sweep extrema against this axis.
func (rp revolvePayload) sweepBoundAlong(
	c0, c1, mlo, mhi float64, work *freeform.FreeformWork, profile *revolveExtentProfile,
) (float64, float64, error) {
	coordUpper := 0.0
	if profile != nil {
		coordUpper = profile.coordUpper
	} else {
		var err error
		coordUpper, err = profileCoordinateUpper(rp.profile, work, nil)
		if err != nil {
			return 0, 0, err
		}
	}
	return revolveaxis.SweepBoundAlong(rp.ax.numeric(), rp.sectionDelta, c0, c1,
		rp.phi0, rp.phi1, rp.den, mlo, mhi, rp.full, coordUpper)
}

// revolveBoundsContext computes the axis-aligned bounds of the placed
// revolved solid — the same directional-extreme analysis the prism uses, in
// cylindrical coordinates (docs/evaluator-design.md §6). Bounds is Exact only
// where every axis's extent reads with a zero bound; every other reading (a
// partial sweep, an amplitude no float64 holds exactly, a boundary extreme a
// computed arc radius carries, the boundary scan's own gu·u + gv·v arithmetic
// under a non-trivial direction, the axis frame/placement's own rounding, or the
// reading's own summation of those terms into a published endpoint)
// is Approximate with the PROVEN bound its own arithmetic derives.
//
// The box states no error term of its own. Every mechanism that can move an
// end — the sweep extreme's, the boundary scan's candidate positions and its own
// arithmetic (proofbound.PlaneDotDecompositionRoundAllow), the axis frame and
// placement's own rounding (frameRoundAllow), and the endpoint summation's
// (proofbound.ExactSumRound) — belongs to the extent reading
// itself (extentBoundedAlong), which every consumer takes, so the box simply
// maxes the three axes' half-widths. Charging one of them here instead would
// leave the same coordinate bounded on this path and exact on the
// through-all stop path.
func revolveBoundsContext(ctx context.Context, rp revolvePayload, work *freeform.FreeformWork) (Box, error) {
	var profile *revolveExtentProfile
	analytic, err := analyticRevolveProfile(ctx, rp.profile)
	if err != nil {
		return Box{}, err
	}
	if analytic {
		profile, err = resolveAnalyticRevolveExtentProfile(ctx, rp.profile, work)
		if err != nil {
			return Box{}, err
		}
	}
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	bound := 0.0
	for i, g := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, extentBound, err := rp.extentBoundedAlongProfile(ctx, g, work, profile)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		if extentBound > bound {
			bound = extentBound
		}
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// analyticRevolveProfile leaves free-form and unknown segment kinds on the
// original per-read path, including their work charges and refusal order.
func analyticRevolveProfile(ctx context.Context, profile ProfileRecord) (bool, error) {
	for _, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for _, segment := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			switch segment.(type) {
			case LineSeg, ArcSeg, CircleSeg:
			default:
				return false, nil
			}
		}
	}
	return true, ctx.Err()
}

// resolveAnalyticRevolveExtentProfile resolves each analytic walk once and
// polls cancellation before each segment. Analytic walks charge no free-form
// work, so this local view needs no replay charge when the three axes read it.
func resolveAnalyticRevolveExtentProfile(
	ctx context.Context, profile ProfileRecord, work *freeform.FreeformWork,
) (*revolveExtentProfile, error) {
	walks := &momentinput.ProfileWalks{
		Profile: profile,
		Outer:   make([]survey2d.SegmentWalk, len(profile.Outer.Segments)),
		Holes:   make([][]survey2d.SegmentWalk, len(profile.Holes)),
	}
	coordUpper := 0.0
	resolve := func(segments []CurveSegment, result []survey2d.SegmentWalk) error {
		for i, segment := range segments {
			if err := ctx.Err(); err != nil {
				return err
			}
			walk, err := boundarywalk.WalkOf(segment, work)
			if err != nil {
				return err
			}
			if err := boundarywalk.RequireAnalyticWalk(walk, "a placed cap frame"); err != nil {
				return err
			}
			result[i] = walk
			coordUpper = math.Max(coordUpper, walk.CoordUpper)
		}
		return nil
	}
	if err := resolve(profile.Outer.Segments, walks.Outer); err != nil {
		return nil, err
	}
	for i, hole := range profile.Holes {
		walks.Holes[i] = make([]survey2d.SegmentWalk, len(hole.Segments))
		if err := resolve(hole.Segments, walks.Holes[i]); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return &revolveExtentProfile{walks: walks, coordUpper: coordUpper}, nil
}

// axisExtremeContext keeps the profile scan and its work budget at the root.
// revolveaxis composes the scan, dot-product and axis-anchor bounds.
func axisExtremeContext(
	ctx context.Context, rp revolvePayload, wg, k float64, wantMax bool,
	work *freeform.FreeformWork, profile *revolveExtentProfile,
) (float64, float64, error) {
	gu, gv := rp.ax.planeDirection(wg, k)
	var walks *momentinput.ProfileWalks
	if profile != nil {
		walks = profile.walks
	}
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, rp.profile, gu, gv, work, walks)
	if err != nil {
		return 0, 0, err
	}
	coordUpper := 0.0
	if profile != nil {
		coordUpper = profile.coordUpper
	} else {
		coordUpper, err = profileCoordinateEnvelope(rp.profile, work, nil)
		if err != nil {
			return 0, 0, err
		}
	}
	extreme, allowance := revolveaxis.ExtremeFromScan(rp.ax.numeric(), gu, gv, lo, hi, bound, coordUpper, wantMax)
	return extreme, allowance, nil
}
