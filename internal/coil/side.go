package coil

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// ProfileSide is Table CS rows CS5 and CS6 (§5.1): it reads the profile's
// radial and axial extremes about the resolved axis, decides the side
// through revolveaxis.ResolveSide, and requires the near-axis extreme proven
// strictly positive. With one turn or more it requires the axial extent's
// upper bound below the pitch (CP5). Both are reject-only readings off the
// profile's own record; neither admits a profile the proof does not cover.
func ProfileSide(ctx context.Context, profile momentinput.Profile, line revolveaxis.Line2, pitch, turns float64, work *freeform.FreeformWork) (int, error) {
	nU, nV := -line.DV, line.DU
	rlo, rhi, rBound, err := prismextent.ProfileBoundaryExtremes(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return 0, err
	}
	zlo, zhi, zBound, err := prismextent.ProfileBoundaryExtremes(ctx, profile, line.DU, line.DV, work, nil)
	if err != nil {
		return 0, err
	}
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, nil)
	if err != nil {
		return 0, err
	}
	resolved, err := revolveaxis.ResolveSide(line, revolveaxis.SideExtremes{
		RLo: rlo, RHi: rhi, RBound: rBound,
		ZLo: zlo, ZHi: zhi, ZBound: zBound,
		CoordUpper: coordUpper,
	})
	if err != nil {
		return 0, err
	}
	switch proofbound.AdmitAbove(resolved.Near, 0) {
	case proofbound.SurvReject:
		return 0, fmt.Errorf(`%w: the coil profile touches its axis; a point on the axis sweeps to a segment of it and pinches the wall`, decaderr.ErrDegenerate)
	case proofbound.SurvStraddle:
		return 0, fmt.Errorf(`%w: the coil profile's distance from its axis is known only to ±%v mm, which does not prove it off the axis`, decaderr.ErrUnsupported, resolved.Near.Bound)
	}
	if turns >= 1 && resolved.AxialExtentUpper >= pitch {
		return 0, fmt.Errorf(`%w: the coil profile reaches %v mm along its axis, at or past the %v mm pitch, so its %v turns would meet; a profile narrower than one pitch is required at one turn or more`,
			decaderr.ErrUnsupported, resolved.AxialExtentUpper, pitch, turns)
	}
	return int(resolved.Side), nil
}
