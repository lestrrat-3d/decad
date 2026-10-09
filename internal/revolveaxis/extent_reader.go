package revolveaxis

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/r3"
)

// ExtentInput is the resolved revolve data needed for a directional extent.
// The root evaluator supplies its placement and already resolved basis.
type ExtentInput struct {
	Profile      momentinput.Profile
	Axis         Frame
	SectionDelta float64
	Phi0, Phi1   float64
	Full         bool
	Denotation   revolveangle.Sweep
	Transform    r3.Transform
	Basis        revolvemesh.RevolveBasis
	Lift         revolvemesh.RevolveLift
}

// ExtentScan reads the profile boundary's directional extrema and their
// proven displacement. It retains the evaluator's profile scan and work budget.
type ExtentScan func(context.Context, momentinput.Profile, float64, float64,
	*freeform.FreeformWork, *momentinput.ProfileWalks) (float64, float64, float64, error)

// Bounds reads the three world axes through the same analytic profile walks.
// Free-form profiles retain per-read resolution and work charges.
func Bounds(ctx context.Context, in ExtentInput, work *freeform.FreeformWork,
	scan ExtentScan,
) (r3.Vec, r3.Vec, float64, error) {
	var profile *ExtentProfile
	analytic, err := AnalyticProfile(ctx, in.Profile)
	if err != nil {
		return r3.Vec{}, r3.Vec{}, 0, err
	}
	if analytic {
		profile, err = ResolveAnalyticExtentProfile(ctx, in.Profile, work)
		if err != nil {
			return r3.Vec{}, r3.Vec{}, 0, err
		}
	}
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	bound := 0.0
	for i, g := range axes {
		if err := ctx.Err(); err != nil {
			return r3.Vec{}, r3.Vec{}, 0, err
		}
		lo, hi, extentBound, err := ExtentAlong(ctx, in, g, work, profile, scan)
		if err != nil {
			return r3.Vec{}, r3.Vec{}, 0, err
		}
		minC[i] = lo
		maxC[i] = hi
		if extentBound > bound {
			bound = extentBound
		}
	}
	return r3.NewVec(minC[0], minC[1], minC[2]),
		r3.NewVec(maxC[0], maxC[1], maxC[2]), bound, nil
}

// ExtentAlong composes the section, sweep, frame and placement displacements
// for both ends of a revolve's interval along g. The boundary scan and swept
// radial coefficient can displace the same end, so they are summed per end
// before the larger total is taken. Frame, section and final-sum allowances
// displace both ends and compose outside that maximum. Thus bounds and
// through-all stops read the same complete interval, and exactly resolved
// axis-aligned profiles retain a zero bound.
func ExtentAlong(ctx context.Context, in ExtentInput, g r3.Vec, work *freeform.FreeformWork,
	profile *ExtentProfile, scan ExtentScan,
) (float64, float64, float64, error) {
	b := in.Basis
	base := in.Transform.Apply(b.A3).Dot(g)
	wg := in.Transform.ApplyDir(b.W).Dot(g)
	c0 := in.Transform.ApplyDir(b.E0).Dot(g)
	c1 := in.Transform.ApplyDir(b.E1).Dot(g)
	mlo, mhi := revolveangle.Extremes(c0, c1, in.Phi0, in.Phi1, in.Full)
	hi, hiBound, err := axisExtreme(ctx, in, wg, mhi, true, work, profile, scan)
	if err != nil {
		return 0, 0, 0, err
	}
	lo, loBound, err := axisExtreme(ctx, in, wg, mlo, false, work, profile, scan)
	if err != nil {
		return 0, 0, 0, err
	}
	coordUpper, err := extentCoordUpper(in.Profile, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	sweepLo, sweepHi, err := SweepBoundAlong(in.Axis, in.SectionDelta, c0, c1,
		in.Phi0, in.Phi1, in.Denotation, mlo, mhi, in.Full, coordUpper)
	if err != nil {
		return 0, 0, 0, err
	}
	bound := ExtentBoundarySweepBound(loBound, hiBound, sweepLo, sweepHi)
	// The frame and sweep readings each used CoordinateUpper on the uncached
	// path. Preserve that second read and its work charge.
	coordUpper, err = extentCoordUpper(in.Profile, work, profile)
	if err != nil {
		return 0, 0, 0, err
	}
	frameAllow := FrameRoundAllow(in.Axis, in.SectionDelta, in.Transform, g, b,
		in.Lift.BasisRound(b), base, wg, c0, c1, coordUpper)
	loEnd, hiEnd, bound := FinishExtent(in.Axis, in.SectionDelta, base, lo, hi, bound, frameAllow)
	return loEnd, hiEnd, bound, nil
}

func extentCoordUpper(profile momentinput.Profile, work *freeform.FreeformWork,
	cached *ExtentProfile,
) (float64, error) {
	if cached != nil {
		return cached.CoordUpper, nil
	}
	return momentinput.CoordinateUpper(profile, work, nil)
}

func axisExtreme(ctx context.Context, in ExtentInput, wg, k float64, wantMax bool,
	work *freeform.FreeformWork, profile *ExtentProfile, scan ExtentScan,
) (float64, float64, error) {
	gu, gv := in.Axis.PlaneDirection(wg, k)
	var walks *momentinput.ProfileWalks
	if profile != nil {
		walks = profile.Walks
	}
	lo, hi, bound, err := scan(ctx, in.Profile, gu, gv, work, walks)
	if err != nil {
		return 0, 0, err
	}
	coordUpper := 0.0
	if profile != nil {
		coordUpper = profile.CoordUpper
	} else {
		coordUpper, err = momentinput.CoordinateEnvelope(in.Profile, work, nil)
		if err != nil {
			return 0, 0, err
		}
	}
	extreme, allowance := ExtremeFromScan(in.Axis, gu, gv, lo, hi, bound, coordUpper, wantMax)
	return extreme, allowance, nil
}
