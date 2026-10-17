package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// LoftSection names one recorded section of a smooth LoftSections feature.
type LoftSection struct {
	Sketch  *sketch.Sketch
	Profile *sketch.Profile
}

// LoftSections builds one solid through three or more Sketch profiles.
// The three-section quadratic route admits positive homothetic whole-line loops
// star-shaped about their common origin. A varying four-arc circular section
// can build as a ruled cone; exactly matching curved records build as a
// straight prism. Two varying fitted flanks with unchanged circular arcs
// use the source interpolants' quadratic height wall. Four rounded rectangles
// with a common radius and height use a cubic-width wall. Three to thirty-two
// centered rectangles may vary width and height independently. All routes require
// ascending, equally spaced world-XY planes and interpolate every section exactly
// (docs/loft-sections-design.md). Other shapes refuse.
func (d *Document) LoftSections(ctx context.Context, sections ...LoftSection) (*Body, error) {
	if ctx == nil || d == nil || len(sections) < 3 {
		return nil, fmt.Errorf("%w: LoftSections needs a context, document and at least three sections", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(sections) == 4 {
		return d.buildFourRoundedSections(ctx, sections)
	}
	if len(sections) >= 5 {
		return d.buildManyRectangularSections(ctx, sections)
	}
	var records [3]profileRecord
	var planes [3]planeRecord
	var work [3]*freeform.FreeformWork
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := sectioncert.RecordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		work[i] = freeform.NewFreeformWork()
		if _, err := profile.FalsifyRecordedArea(sketchArea, work[i]); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	var built smoothloft.Build
	var err error
	if smoothloft.HasCurvedSection(records[0]) {
		built, err = smoothloft.BuildThreeCircles(ctx, records, planes)
		if errors.Is(err, ErrUnsupported) {
			body, constantErr := d.buildConstantCurvedSections(ctx, records, planes, work[0])
			if constantErr == nil {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
				d.commit(body)
				return body, nil
			}
			if !errors.Is(constantErr, ErrUnsupported) {
				return nil, constantErr
			}
			built, err = smoothloft.BuildThreeFits(ctx, records, planes, work)
		}
	} else {
		var rectangular bool
		built, rectangular, err = smoothloft.TryBuildThreeRectangles(ctx, records, planes)
		if !rectangular {
			built, err = smoothloft.BuildThree(ctx, records, planes)
		}
	}
	if err != nil {
		return nil, err
	}
	return d.buildSmoothSectionBody(ctx, built, planes[0], planes[2])
}

func (d *Document) buildManyRectangularSections(ctx context.Context, sections []LoftSection) (*Body, error) {
	if len(sections) > 32 {
		return nil, fmt.Errorf("%w: rectangular LoftSections accepts at most 32 sections", ErrUnsupported)
	}
	records := make([]profileRecord, len(sections))
	planes := make([]planeRecord, len(sections))
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := sectioncert.RecordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		if _, err := profile.FalsifyRecordedArea(sketchArea, freeform.NewFreeformWork()); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	built, err := smoothloft.BuildManyRectangles(ctx, records, planes)
	if err != nil {
		return nil, err
	}
	return d.buildSmoothSectionBody(ctx, built, planes[0], planes[len(planes)-1])
}

func (d *Document) buildFourRoundedSections(ctx context.Context, sections []LoftSection) (*Body, error) {
	var records [4]profileRecord
	var planes [4]planeRecord
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := sectioncert.RecordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		if _, err := profile.FalsifyRecordedArea(sketchArea, freeform.NewFreeformWork()); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	built, rectangular, err := smoothloft.TryBuildFourRectangles(ctx, records, planes)
	if !rectangular {
		built, err = smoothloft.BuildFourRounded(ctx, records, planes)
	}
	if err != nil {
		return nil, err
	}
	return d.buildSmoothSectionBody(ctx, built, planes[0], planes[3])
}

func (d *Document) buildSmoothSectionBody(ctx context.Context, built smoothloft.Build,
	start, end planeRecord) (*Body, error) {
	frameStart, err := r3.NewFrame(start.Origin, start.U, start.V)
	if err != nil {
		return nil, fmt.Errorf("%w: the first loft section has no frame: %s", ErrDegenerate, err)
	}
	frameEnd, err := r3.NewFrame(end.Origin, end.U, end.V)
	if err != nil {
		return nil, fmt.Errorf("%w: the last loft section has no frame: %s", ErrDegenerate, err)
	}
	ref := d.nextProducerID()
	groups := make([]facetGroup, built.EdgeCount+2)
	for i := range built.EdgeCount {
		groups[i] = facetGroup{Origins: []provenance.Ref{{Producer: ref,
			Role: fmt.Sprintf("side(0,%d)", i)}}, Surface: surfacegeom.NURBSSurface{}}
		if i < len(built.ArcRadii) && built.ArcRadii[i] > 0 {
			groups[i].Surface = surfacegeom.Cylinder{Origin: start.Origin, Axis: r3.Vec{Z: 1},
				Radius: units.Millimeters(built.ArcRadii[i])}
		}
	}
	groups[built.EdgeCount] = facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapStart}},
		Surface: surfacegeom.Plane{Frame: frameStart}, Planar: true, Reversed: true}
	groups[built.EdgeCount+1] = facetGroup{Origins: []provenance.Ref{{Producer: ref, Role: provenance.RoleCapEnd}},
		Surface: surfacegeom.Plane{Frame: frameEnd}, Planar: true}
	vertexBound := make([]float64, len(built.Vertices))
	for i := range vertexBound {
		vertexBound[i] = built.MeshBound
	}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), facetedPayload{
		SmoothRectSection:       built.RectangularSection,
		SmoothHomotheticSection: built.HomotheticSection,
		Verts:                   built.Vertices, Tris: built.Triangles, Src: built.Sources,
		Groups: groups, VertexBound: vertexBound, MeshBound: built.MeshBound,
		VolSymDiff: built.VolSymDiff, AreaSlack: built.AreaSlack,
		DPair: built.Diameter, Xform: r3.Identity(),
	}))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// buildConstantCurvedSections uses the analytic prism kernel only after
// exact recorded identity proves every stated section is the same region.
func (d *Document) buildConstantCurvedSections(ctx context.Context, profiles [3]profileRecord,
	planes [3]planeRecord, work *freeform.FreeformWork) (*Body, error) {
	frame, height, err := smoothloft.ConstantCurvedFrameHeight(ctx, profiles, planes)
	if err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: profiles[0], Frame: frame, Z0: 0, Z1: height, Xform: r3.Identity(),
	}, work))
}
