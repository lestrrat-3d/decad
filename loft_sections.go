package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// LoftSection names one recorded section of a smooth LoftSections feature.
type LoftSection struct {
	Sketch  *sketch.Sketch
	Profile *sketch.Profile
}

// LoftSections builds one quadratic solid through exactly three Sketch
// profiles. The admitted profiles are exact positive homothetic whole-line
// loops star-shaped about their common origin on ascending, equally spaced
// world-XY planes; every other shape
// returns ErrUnsupported (docs/loft-sections-design.md). The middle section
// is interpolated exactly. Each smooth wall reports NURBSSurface, and the
// certified held mesh carries an occupied-volume proof into VerifyAll.
func (d *Document) LoftSections(ctx context.Context, sections ...LoftSection) (*Body, error) {
	if ctx == nil || d == nil || len(sections) != 3 {
		return nil, fmt.Errorf("%w: LoftSections needs a context, document and exactly three sections", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var records [3]profileRecord
	var planes [3]planeRecord
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := recordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		if _, err := falsifyRecordedArea(profile, sketchArea, freeform.NewFreeformWork()); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	built, err := smoothloft.BuildThree(ctx, records, planes)
	if err != nil {
		return nil, err
	}
	frameStart, err := r3.NewFrame(planes[0].Origin, planes[0].U, planes[0].V)
	if err != nil {
		return nil, fmt.Errorf("%w: the first loft section has no frame: %s", ErrDegenerate, err)
	}
	frameEnd, err := r3.NewFrame(planes[2].Origin, planes[2].U, planes[2].V)
	if err != nil {
		return nil, fmt.Errorf("%w: the last loft section has no frame: %s", ErrDegenerate, err)
	}
	ref := d.nextProducerID()
	groups := make([]facetGroup, built.EdgeCount+2)
	for i := range built.EdgeCount {
		groups[i] = facetGroup{origins: []FeatureRef{{producer: ref,
			Role: fmt.Sprintf("side(0,%d)", i)}}, surface: NURBSSurface{}}
	}
	groups[built.EdgeCount] = facetGroup{origins: []FeatureRef{{producer: ref, Role: roleCapStart}},
		surface: Plane{Frame: frameStart}, planar: true, reversed: true}
	groups[built.EdgeCount+1] = facetGroup{origins: []FeatureRef{{producer: ref, Role: roleCapEnd}},
		surface: Plane{Frame: frameEnd}, planar: true}
	vertexBound := make([]float64, len(built.Vertices))
	for i := range vertexBound {
		vertexBound[i] = built.MeshBound
	}
	body, err := buildFacetedBody(ctx, d, ref, facetedPayload{
		verts: built.Vertices, tris: built.Triangles, src: built.Sources,
		groups: groups, vertexBound: vertexBound, meshBound: built.MeshBound,
		volSymDiff: built.VolSymDiff, areaSlack: built.AreaSlack,
		dPair: built.Diameter, xform: r3.Identity(),
	})
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}
