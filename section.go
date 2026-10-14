package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
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
// chord, an authenticated sphere cut by a cardinal plane with an exact circle
// or bounded polygon or by a central oblique plane, an authenticated right cone
// cut by a cardinal plane with an exact circle,
// a direct circular-hole Loft cut by an exact world-XY interior plane,
// a rectangular prism cut by an interior cardinal vertical plane, and
// a bounded oblique cut of an exact rectangular or whole-circle prism,
// including a certified crossing of one circle-prism cap. The
// exact slab profiles of a stack-backed BRep can be cut strictly inside one
// slab by a plane preserving their recorded coordinates. The
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
	if pl, ok := b.payload.(loftPayload); ok {
		profile, admitted := loftCircleSectionRecord(pl, plane)
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an exact parallel circular-hole loft`, ErrUnsupported)
		}
		return reconstructExactSection(ctx, profile, plane)
	}
	if rp, ok := b.payload.(revolvePayload); ok {
		profile, polygon, delta, admitted := cardinalSphereSection(b, plane)
		if !admitted {
			faces := b.Faces()
			if len(faces) == 2 && len(b.Edges()) == 1 {
				cone, planar := 0, 0
				for _, face := range faces {
					switch face.n.Surface.Kind() {
					case surfacegeom.KindCone:
						cone++
					case surfacegeom.KindPlane:
						planar++
					}
				}
				if cone == 1 && planar == 1 {
					profile, admitted = cardinalConeSectionRecord(rp, plane)
				}
			}
		}
		if !admitted {
			if sphere, authenticated := sourceSphereAtPose(b, r3.Identity()); authenticated {
				polygon, delta, admitted = sectionclip.ObliqueSpherePolygon(sphere.center, sphere.radius, plane)
			}
		}
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an authenticated sphere or cone and certified plane cut`,
				ErrUnsupported)
		}
		if polygon != nil {
			s, result, err := sectioncert.ReconstructBoundedSection(ctx, polygon, plane, delta)
			if err != nil {
				return nil, nil, err
			}
			return s, []*sketch.Profile{result}, nil
		}
		return reconstructExactSection(ctx, profile, plane)
	}
	if _, ok := b.payload.(brepPayload); ok {
		operand, admitted := stackedUnionOperandOf(b)
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an authenticated BRep slab`, ErrUnsupported)
		}
		return sectionBrepStack(ctx, operand, plane)
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
				s, result, err := sectioncert.ReconstructBoundedSection(ctx, polygon, plane, delta)
				if err != nil {
					return nil, nil, err
				}
				return s, []*sketch.Profile{result}, nil
			}
			return nil, nil, fmt.Errorf(`%w: section plane does not cross an admitted exact prism section`, ErrUnsupported)
		}
	}
	return reconstructExactSection(ctx, profile, plane)
}

// sectionBrepStack reads only the certified slab profiles of an A1 Union.
// The same exact frame and strict interior-level check used for a prism
// applies to each slab separately. An interface level names two different
// sections, so neither slab admits it.
func sectionBrepStack(ctx context.Context, operand stackedUnionOperand, plane r3.Frame) (
	*sketch.Sketch, []*sketch.Profile, error,
) {
	if operand.proxy.SectionDelta != 0 {
		return nil, nil, fmt.Errorf(`%w: BRep section has a displaced source profile`, ErrUnsupported)
	}
	for _, slab := range operand.slabs {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		candidate := operand.proxy
		candidate.Z0, candidate.Z1 = slab.Z0, slab.Z1
		if !candidate.SectionPlaneInside(plane) {
			continue
		}
		if slab.Z0Delta != 0 || slab.Z1Delta != 0 || len(slab.Regions) != 1 {
			return nil, nil, fmt.Errorf(`%w: BRep section needs one exact slab region`, ErrUnsupported)
		}
		return reconstructExactSection(ctx, slab.Regions[0], plane)
	}
	return nil, nil, fmt.Errorf(`%w: section plane lies outside BRep slab interiors`, ErrUnsupported)
}

func reconstructExactSection(ctx context.Context, profile profileRecord, plane r3.Frame) (
	*sketch.Sketch, []*sketch.Profile, error,
) {
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
