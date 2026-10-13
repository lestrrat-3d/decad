package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Section returns the material profile where this solid meets plane. The
// returned Sketch uses plane and its profiles can enter another feature. The
// query is read-only, including when the body has been retired.
//
// One class is an exact straight prism cut strictly between its caps by a
// plane whose frame preserves the prism's recorded section coordinates.
// Other classes are a whole-circle prism cut by a cardinal plane with an exact
// chord, a rectangular prism cut by an interior cardinal vertical plane, and
// a bounded oblique cut of an exact rectangular or whole-circle prism,
// including a certified crossing of one circle-prism cap. The
// bounded result enters only straight solid Extrude with an independent Distance.
// Other body or plane classes return [ErrUnsupported]. A nil body or context,
// or an invalid plane, returns [ErrDegenerate]. A sheet or unsound solid returns
// [ErrNotSolid]. See docs/section-design.md.
func (b *Body) Section(ctx context.Context, plane r3.Frame) (*sketch.Sketch, []*sketch.Profile, error) {
	if b == nil || ctx == nil || !plane.IsValid() {
		return nil, nil, fmt.Errorf(`%w: a section needs a body, context and valid plane`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	if b.Kind() != BodySolid || !b.IsSolid() {
		return nil, nil, fmt.Errorf(`%w: a section needs a sound solid`, ErrNotSolid)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
		return nil, nil, fmt.Errorf(`%w: section needs an exact straight prism`, ErrUnsupported)
	}
	profile := pp.Profile
	if !pp.SectionPlaneInside(plane) {
		var admitted bool
		profile, admitted = cardinalCircleSectionRecord(pp, plane)
		if !admitted {
			profile, admitted = cardinalRectangleSectionRecord(pp, plane)
		}
		if !admitted {
			polygon, delta, clipped := obliqueRectangleClip(pp, plane)
			if !clipped {
				polygon, delta, clipped = obliqueCircleClip(pp, plane)
			}
			if !clipped {
				polygon, delta, clipped = obliqueCircleCapClip(ctx, pp, plane)
			}
			if clipped {
				s, result, err := reconstructObliqueSection(ctx, polygon, plane, delta)
				if err != nil {
					return nil, nil, err
				}
				return s, []*sketch.Profile{result}, nil
			}
			return nil, nil, fmt.Errorf(`%w: section plane does not cross an admitted exact prism section`, ErrUnsupported)
		}
	}
	work := freeform.NewFreeformWork()
	if _, err := momentinput.ChargeReconstruction(profile, work); err != nil {
		return nil, nil, err
	}
	s, result, err := momentinput.ReconstructProfileOnFrame(ctx, profile, plane)
	if err != nil {
		return nil, nil, err
	}
	return s, []*sketch.Profile{result}, nil
}
