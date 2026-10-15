package revolveplan

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
)

// ResolveAxisSide scans the record, asks revolveaxis to decide its side, then
// audits each walk for axis contact and charges any snapped endpoint.
func ResolveAxisSide(
	ctx context.Context, profile momentinput.Profile, line revolveaxis.Line2, work *freeform.FreeformWork,
) (revolveaxis.Frame, float64, error) {
	nU, nV := -line.DV, line.DU
	rlo, rhi, rBound, err := prismextent.ProfileBoundaryExtremes(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return revolveaxis.Frame{}, 0, err
	}
	zlo, zhi, zBound, err := prismextent.ProfileBoundaryExtremes(ctx, profile, line.DU, line.DV, work, nil)
	if err != nil {
		return revolveaxis.Frame{}, 0, err
	}

	// The side gate charges the scan's dot-product rounding separately from
	// the positional bounds returned with the extremes.
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, nil)
	if err != nil {
		return revolveaxis.Frame{}, 0, err
	}
	resolved, err := revolveaxis.ResolveSide(line, revolveaxis.SideExtremes{
		RLo: rlo, RHi: rhi, RBound: rBound,
		ZLo: zlo, ZHi: zhi, ZBound: zBound,
		CoordUpper: coordUpper,
	})
	if err != nil {
		return revolveaxis.Frame{}, 0, err
	}
	ax := resolved.Frame
	ax.RadialAdmitAllow = resolved.RadialAdmitAllow
	ax.RadialProof = resolved.RadialProof
	ax.AxialExtentUpper = resolved.AxialExtentUpper
	snap, err := revolveaxis.AuditProfileAxisContact(ax, profile, work)
	if err != nil {
		return revolveaxis.Frame{}, 0, err
	}
	ax.Snap = snap
	return ax, resolved.Side, nil
}
