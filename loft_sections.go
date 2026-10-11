package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/smoothloft"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// LoftSection names one recorded section of a smooth LoftSections feature.
type LoftSection struct {
	Sketch  *sketch.Sketch
	Profile *sketch.Profile
}

// LoftSections builds one solid through exactly three Sketch
// profiles. The quadratic route admits positive homothetic whole-line loops
// star-shaped about their common origin. An exactly matching curved record
// on all three planes builds as a straight prism. Both routes require
// ascending, equally spaced world-XY planes and interpolate the middle
// section exactly (docs/loft-sections-design.md). Other shapes refuse.
func (d *Document) LoftSections(ctx context.Context, sections ...LoftSection) (*Body, error) {
	if ctx == nil || d == nil || len(sections) != 3 {
		return nil, fmt.Errorf("%w: LoftSections needs a context, document and exactly three sections", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var records [3]profileRecord
	var planes [3]planeRecord
	var work [3]*freeform.FreeformWork
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := recordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		work[i] = freeform.NewFreeformWork()
		if _, err := falsifyRecordedArea(profile, sketchArea, work[i]); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	if hasCurvedSection(records[0]) {
		body, err := d.buildConstantCurvedSections(ctx, records, planes, work[0])
		if err != nil {
			return nil, err
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		d.commit(body)
		return body, nil
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

func hasCurvedSection(profile profileRecord) bool {
	for _, segment := range profile.Outer.Segments {
		if _, ok := segment.(lineSeg); !ok {
			return true
		}
	}
	return false
}

// buildConstantCurvedSections uses the analytic prism kernel only after
// exact recorded identity proves every stated section is the same region.
func (d *Document) buildConstantCurvedSections(ctx context.Context, profiles [3]profileRecord,
	planes [3]planeRecord, work *freeform.FreeformWork) (*Body, error) {
	if len(profiles[0].Holes) != 0 || len(profiles[0].Outer.Segments) < 3 ||
		len(profiles[0].Outer.Segments) > 64 {
		return nil, fmt.Errorf("%w: a curved three-section loft needs one outer loop of 3 to 64 segments", ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	for i := 1; i < 3; i++ {
		same, err := momentinput.ExactProfileEqual(budget, profiles[0], profiles[i])
		if err != nil {
			return nil, err
		}
		if !same {
			return nil, fmt.Errorf("%w: curved three-section loft profiles must have identical records", ErrUnsupported)
		}
	}
	for i := range planes {
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return nil, fmt.Errorf("%w: curved three-section loft planes need common world-XY axes and origin", ErrUnsupported)
		}
	}
	z0 := new(big.Rat).SetFloat64(planes[0].Origin.Z)
	z1 := new(big.Rat).SetFloat64(planes[1].Origin.Z)
	z2 := new(big.Rat).SetFloat64(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 ||
		new(big.Rat).Mul(big.NewRat(2, 1), z1).Cmp(new(big.Rat).Add(z0, z2)) != 0 {
		return nil, fmt.Errorf("%w: curved three-section loft planes need ascending, equally spaced levels", ErrUnsupported)
	}
	height, exact := new(big.Rat).Sub(z2, z0).Float64()
	if !exact || math.IsInf(height, 0) || height <= 0 {
		return nil, fmt.Errorf("%w: curved three-section loft height must be held exactly", ErrUnsupported)
	}
	frame, err := r3.NewFrame(planes[0].Origin, planes[0].U, planes[0].V)
	if err != nil {
		return nil, fmt.Errorf("%w: curved three-section loft has no first frame: %s", ErrDegenerate, err)
	}
	ref := d.nextProducerID()
	return evalPrismContext(ctx, d, ref, prismPayload{
		profile: profiles[0], frame: frame, z0: 0, z1: height, xform: r3.Identity(),
	}, work)
}
