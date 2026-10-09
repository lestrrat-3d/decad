package prismextent

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ExtentInput holds the resolved prism data and direction coefficients for
// one reading. The evaluator supplies coefficients from its payload's frame.
type ExtentInput struct {
	Profile    momentinput.Profile
	Frame      r3.Frame
	Transform  r3.Transform
	Z0, Z1     float64
	Base       float64
	GU, GV, GZ float64
	Direction  r3.Vec
}

// ExtentScan reads the section boundary's directional extrema and bound.
type ExtentScan func(context.Context, momentinput.Profile, float64, float64,
	*freeform.FreeformWork, *momentinput.ProfileWalks) (float64, float64, float64, error)

// ExtentReader reads one world direction from the same prism payload.
type ExtentReader func(context.Context, r3.Vec, *freeform.FreeformWork,
	*momentinput.ProfileWalks) (float64, float64, float64, error)

// Bounds reads all three world axes, then composes the largest directional
// bound with the section and axial displacements. A sole nonzero term is
// copied directly; summing it with zeros would add an unnecessary ulp.
func Bounds(ctx context.Context, work *freeform.FreeformWork, walks *momentinput.ProfileWalks,
	sectionDelta, axialDelta float64, extent ExtentReader,
) (r3.Vec, r3.Vec, float64, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var low, high [3]float64
	extremeBound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return r3.Vec{}, r3.Vec{}, 0, err
		}
		lo, hi, bound, err := extent(ctx, axis, work, walks)
		if err != nil {
			return r3.Vec{}, r3.Vec{}, 0, err
		}
		low[i], high[i] = lo, hi
		extremeBound = math.Max(extremeBound, bound)
	}
	terms := make([]float64, 0, 3)
	if sectionDelta != 0 {
		terms = append(terms, sectionDelta)
	}
	if extremeBound != 0 {
		terms = append(terms, extremeBound)
	}
	if axialDelta != 0 {
		terms = append(terms, axialDelta)
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = proofbound.AbsSumUpper(terms...)
	}
	return r3.NewVec(low[0], low[1], low[2]), r3.NewVec(high[0], high[1], high[2]), bound, nil
}

// ExtentAlong composes the section scan, frame and placement, and final
// endpoint summation into the bound for one directional interval. The scan
// charges each extreme's candidate enclosure, including a computed arc radius
// or free-form span bracket. The placement term charges the direction
// coefficients independently. Finally, each endpoint's own sum is compared
// against its exact arithmetic value, which catches rounding from a pure
// translation even when every coefficient is exact. All three terms are zero
// for exactly resolved endpoints in an unplaced, axis-aligned frame.
func ExtentAlong(ctx context.Context, in ExtentInput, work *freeform.FreeformWork,
	walks *momentinput.ProfileWalks, scan ExtentScan,
) (float64, float64, float64, error) {
	lo, hi, bound, err := scan(ctx, in.Profile, in.GU, in.GV, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zlo := math.Min(in.Z0*in.GZ, in.Z1*in.GZ)
	zhi := math.Max(in.Z0*in.GZ, in.Z1*in.GZ)
	coordUpper, err := momentinput.CoordinateEnvelope(in.Profile, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(in.Z0), math.Abs(in.Z1))
	placeAllow := PlacementCoeffAllow(in.Transform, in.Frame, in.Direction,
		in.Base, in.GU, in.GV, in.GZ, coordUpper, zUpper)
	// The endpoints round separately because they use different section and
	// axial extrema. The section and placement bounds cover both ends.
	loEnd, hiEnd := in.Base+lo+zlo, in.Base+hi+zhi
	sumAllow := math.Max(
		proofbound.ExactSumRound(loEnd, in.Base, lo, zlo),
		proofbound.ExactSumRound(hiEnd, in.Base, hi, zhi),
	)
	bound = proofbound.AbsSumUpper(bound, placeAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
}
