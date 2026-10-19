package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/stackedunion"
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
// cut by a cardinal plane with an exact circle or bounded polygon,
// a direct matching-profile, full line-boundary or varying circular-boundary Loft
// cut by an exact world-XY interior plane,
// an original rectangular or homothetic LoftSections cut by an exact world-XY interior plane,
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
	if pl, ok := b.n.Payload.(loftPayload); ok {
		profile, admitted := sectionclip.LoftCircleSectionRecord(pl, plane)
		if !admitted {
			profile, admitted = sectionclip.LoftMatchingSectionRecord(pl, plane)
		}
		if !admitted {
			profile, admitted = sectionclip.LoftVaryingCircularOuterSectionRecord(pl, plane)
		}
		if !admitted {
			profile, admitted = sectionclip.LoftRuledLineSectionRecord(pl, plane)
		}
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an exact parallel loft`, ErrUnsupported)
		}
		return sectionclip.ReconstructExactSection(ctx, profile, plane)
	}
	if rp, ok := b.n.Payload.(revolvePayload); ok {
		sphere, authenticated := sourceSphereAtPose(b, r3.Identity())
		var (
			profile  profileRecord
			polygon  []planePoint
			delta    float64
			admitted bool
		)
		if authenticated {
			profile, polygon, delta, admitted = sectionclip.CardinalSphereSection(sphere.center, sphere.radius, plane)
		}
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
					profile, polygon, delta, admitted = sectionclip.CardinalConeSection(rp, plane)
				}
			}
		}
		if !admitted {
			if authenticated {
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
		return sectionclip.ReconstructExactSection(ctx, profile, plane)
	}
	if _, ok := b.n.Payload.(brepPayload); ok {
		operand, admitted := stackedunion.OperandFromPayload(b.n.Payload)
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an authenticated BRep slab`, ErrUnsupported)
		}
		return sectionclip.SectionBrepStack(ctx, operand.Proxy, operand.Slabs, plane)
	}
	if fp, ok := b.n.Payload.(facetedPayload); ok {
		profile, admitted := sectionclip.SmoothRectangularSectionRecord(fp, plane)
		if !admitted {
			profile, admitted = sectionclip.SmoothHomotheticSectionRecord(fp, plane)
		}
		if !admitted {
			return nil, nil, fmt.Errorf(`%w: section needs an original certified smooth loft`, ErrUnsupported)
		}
		return sectionclip.ReconstructExactSection(ctx, profile, plane)
	}
	pp, ok := b.n.Payload.(prismPayload)
	if !ok || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 {
		return nil, nil, fmt.Errorf(`%w: section needs an exact straight prism`, ErrUnsupported)
	}
	profile := pp.Profile
	if !pp.SectionPlaneInside(plane) {
		var admitted bool
		profile, admitted = sectionclip.CardinalCircleSectionRecord(pp, plane)
		if !admitted {
			profile, admitted = sectionclip.CardinalRectangleSectionRecord(pp, plane)
		}
		if !admitted {
			polygon, delta, clipped := sectionclip.ObliqueRectangleClip(pp, plane)
			if !clipped {
				polygon, delta, clipped = sectionclip.ObliqueCircleClip(pp, plane)
			}
			if !clipped {
				polygon, delta, clipped = sectionclip.ObliqueCircleCapClip(ctx, pp, plane)
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
	return sectionclip.ReconstructExactSection(ctx, profile, plane)
}

// Section is the sketch geometry a section-fed operation consumes: a closed
// [*sketch.Profile] or an open [*sketch.Chain]. An operation declared as
// func Op[S Section](..., sec S, ...) infers S from the argument, so callers
// pass either kind without naming the type argument. The operation decides
// what each kind builds and which options it accepts; a kind or option the
// operation cannot honour returns ErrUnsupported, and a nil section returns
// ErrDegenerate.
type Section interface {
	*sketch.Profile | *sketch.Chain
}
