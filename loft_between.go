package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// LoftEnd is one end of a LoftBetween: a recorded profile or a single point.
// Build one with ProfileEnd or PointEnd; the zero value names no end.
type LoftEnd struct {
	sketch  *sketch.Sketch
	profile *sketch.Profile
	apex    r3.Vec
	point   bool
}

// ProfileEnd is a LoftBetween end that is a closed profile of a sketch.
func ProfileEnd(s *sketch.Sketch, p *sketch.Profile) LoftEnd {
	return LoftEnd{sketch: s, profile: p}
}

// PointEnd is a LoftBetween end that is one world point, in millimetres.
func PointEnd(apex r3.Vec) LoftEnd { return LoftEnd{apex: apex, point: true} }

// LoftBetween builds a solid between two ends, each a profile or a point
// (agent-docs/loft-design.md §3.3). Two profiles are Loft's pair and take its
// options; a profile and a point are LoftFromPoint's cone over that profile,
// whichever end the point is, so the solid, its certificate and its limits are
// LoftFromPoint's: one outer loop and no holes, the point off the profile's
// plane, a faceted payload with no loft fillet or chamfer, and the profile's
// cap named capEnd as LoftFromPoint names it. WithLoftAlignment and
// WithLoftSplits name a pairing a point end does not have, so either is
// [ErrDegenerate] on a call with a point end, and WithSurfaceResult is
// [ErrUnsupported]. Two points, or an end built by neither constructor, are
// [ErrDegenerate]. Loft and LoftFromPoint keep their own contracts.
//
// A failed call leaves the document untouched.
func LoftBetween(ctx context.Context, d *Document, from, to LoftEnd, opts ...LoftOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a loft`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	for _, end := range []LoftEnd{from, to} {
		if !end.point && (end.sketch == nil || end.profile == nil) {
			return nil, fmt.Errorf(`%w: a loft end must be built by ProfileEnd with a sketch and a profile, or by PointEnd`, ErrDegenerate)
		}
	}
	switch {
	case from.point && to.point:
		return nil, fmt.Errorf(`%w: a loft between two points has no section`, ErrDegenerate)
	case !from.point && !to.point:
		return Loft(ctx, d, from.sketch, from.profile, to.sketch, to.profile, opts...)
	}
	cfg, err := decodeLoftOptions(opts)
	if err != nil {
		return nil, err
	}
	if cfg.Alignment != nil || len(cfg.Splits[0]) != 0 || len(cfg.Splits[1]) != 0 {
		return nil, fmt.Errorf(`%w: a point end has no segments to align or split`, ErrDegenerate)
	}
	if cfg.SurfaceResult {
		return nil, fmt.Errorf(`%w: a point loft has no surface result`, ErrUnsupported)
	}
	end, apex := from, to
	if from.point {
		end, apex = to, from
	}
	return LoftFromPoint(ctx, d, apex.apex, end.sketch, end.profile)
}
