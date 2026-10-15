package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the tapered Extrude's entry (docs/draft-design.md §7): the
// taper's angle gate (SD2), the SD12 surface-result gate and extent
// resolution, then the one or two slabs internal/draftrecord builds over
// topology nodes (draftrecord.Build, draftrecord.BuildTwoSided).

// draftAngle is SD2 (docs/draft-design.md Table SD) on a nonzero taper that
// has passed SD1: the angle in radians beside its conversion rounding, or
// ErrDegenerate for an angle at or past a right angle to the sweep. The test
// reads the stated angle in degrees as well as in radians, because a stated
// 90 degrees converts to a radian float just below π/2.
func draftAngle(taper units.Value) (float64, float64, error) {
	alpha, alphaDelta, err := extent.DisplacementIn(taper, units.Angle, units.Radian, "the taper")
	if err != nil {
		return 0, 0, err
	}
	if deg, err := taper.In(units.Degree); err != nil || math.Abs(deg) >= 90 || math.Abs(alpha) >= math.Pi/2 {
		return 0, 0, fmt.Errorf(`%w: a taper of %s leans the walls at or past a right angle to the sweep, so they sweep no solid; a draft angle lies strictly between -90 and 90 degrees`, ErrDegenerate, taper)
	}
	return alpha, alphaDelta, nil
}

// extrudeDraft is Extrude's path for a nonzero taper alpha (radians, within
// alphaDelta of the stated angle, past SD1 and SD2): the SD12 surface-result
// gate, extent resolution, then one or two draft slabs. A one-sided stop
// resolves its far level and axial bound before the single-slab build.
// ctx is checked between those steps and polled inside the slab builds; a
// canceled ctx returns ctx.Err() before the commit. Every gate runs before the
// document changes.
func (d *Document) extrudeDraft(ctx context.Context, profile profileRecord, frame r3.Frame, e Extent, alpha, alphaDelta float64, surfaceResult bool) (*Body, error) {
	switch e.(type) {
	case Distance, ThroughAll, ToFace, Symmetric, TwoSided:
	default:
		return nil, fmt.Errorf(`%w: a tapered extrude does not support extent %T`, ErrUnsupported, e)
	}
	if surfaceResult {
		return nil, fmt.Errorf(`%w: this evaluator builds a tapered extrude as a solid only; omit WithSurfaceResult or the taper (draft SD12)`, ErrUnsupported)
	}
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	base := draftPayload{
		Profile:    profile,
		Frame:      frame,
		Z0:         sweep.z0,
		Z1:         sweep.z1,
		Z0Delta:    sweep.z0Delta,
		Z1Delta:    sweep.z1Delta,
		NearStart:  sweep.z0 == 0,
		Taper:      alpha,
		TaperDelta: alphaDelta,
		Xform:      r3.Identity(),
	}
	var body *Body
	if sweep.z0 < 0 && sweep.z1 > 0 {
		negative, positive := base, base
		negative.Z1, negative.Z1Delta, negative.NearStart = 0, 0, false
		positive.Z0, positive.Z0Delta, positive.NearStart = 0, 0, true
		body, err = bodyResult(draftrecord.BuildTwoSided(ctx, envOf(documentNode(d), ref), twoSidedDraftPayload{
			Negative: negative, Positive: positive,
		}))
	} else {
		body, err = bodyResult(draftrecord.Build(ctx, envOf(documentNode(d), ref), base))
	}
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}
