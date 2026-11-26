package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/opcommand"
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

// CircularLoftCorrespondence gives one rational Bernstein map for each
// cardinal quarter, starting at +X and proceeding counterclockwise. Each
// map uses two to six strictly increasing numerators over its positive
// denominator, starting at zero and ending at the denominator.
type CircularLoftCorrespondence struct {
	Numerators   [4][]int64
	Denominators [4]int64
	// SmoothCircularBands keeps the first circular band linear and uses
	// smoothstep for subsequent circular bands.
	SmoothCircularBands bool
	// EmergingFlat maps the three ascending, collinear target lines from the
	// final circle's +X vertex. Every fitted flat must remain on that circle's
	// tangent line; the fitted spline maps around all four circle quarters.
	// The tangent plane's world X coordinate must be exactly representable.
	EmergingFlat bool
}

// LoftSections builds one solid through three or more Sketch profiles.
// The three-section quadratic route admits positive homothetic whole-line loops
// star-shaped about their common origin. A varying four-arc circular section
// can build as a ruled cone; exactly matching curved records build as a
// straight prism. Two varying fitted flanks with unchanged circular arcs
// use the source interpolants' quadratic height wall. Four rounded rectangles
// with a common radius and height use a cubic-width wall. Three to thirty-two
// centered rectangles may vary width and height independently. All of those routes
// require ascending, equally spaced world-XY planes and interpolate every section
// exactly. Three to 1024 sections of one fitted edge and whole lines may turn
// about the plane origin on strictly ascending world-XY planes (agent-docs/loft-sections-design.md).
// A four-quarter-arc circle may also transition to four-edge sections of one
// fitted edge and three lines, with a certified rational wall.
// Other shapes refuse.
func LoftSections(d *Document, sections ...LoftSection) *LoftSectionsCommand {
	c := &LoftSectionsCommand{d: d, sections: slices.Clone(sections)}
	if d == nil {
		c.state.Record("d", errLoftSectionsCount)
	}
	if len(sections) < 3 {
		c.state.Record("sections", errLoftSectionsCount)
	}
	return c
}

// errLoftSectionsCount is the refusal of a missing document or fewer than
// three sections, which LoftSections reports right after the nil ctx check.
var errLoftSectionsCount = fmt.Errorf("%w: LoftSections needs a document and at least three sections", ErrDegenerate)

// LoftSectionsCommand builds one solid through three or more profiles. Build it
// with [LoftSections].
type LoftSectionsCommand struct {
	state      opcommand.State                           // consumed flag and first recorded fault
	d          *Document                                 // stored as given
	sections   []LoftSection                             // copied at LoftSections
	correspond opcommand.Slot[circularCorrespondenceMap] // set by CircularCorrespondence
}

// circularCorrespondenceMap is the validated form of a CircularLoftCorrespondence:
// one rational map per cardinal quarter, and the two band flags.
type circularCorrespondenceMap struct {
	maps                          [][]*big.Rat
	smoothCircularBands, emerging bool
}

// CircularCorrespondence builds a circular-to-spline loft using explicit
// cardinal-quarter parameter maps. The maps and the source sections must
// satisfy the exact admission and continuity proofs in
// agent-docs/loft-sections-design.md §14. A map that does not give two to six
// strictly increasing controls from zero to the denominator is
// [ErrUnsupported], returned by Do. A second call replaces the first.
func (c *LoftSectionsCommand) CircularCorrespondence(correspondence CircularLoftCorrespondence) *LoftSectionsCommand {
	maps, err := circularMaps(correspondence)
	c.correspond.Set(circularCorrespondenceMap{
		maps: maps, smoothCircularBands: correspondence.SmoothCircularBands, emerging: correspondence.EmergingFlat,
	}, err)
	return c
}

// circularMaps validates correspondence and converts each quarter to its
// rational control list. It reads nothing but the value.
func circularMaps(correspondence CircularLoftCorrespondence) ([][]*big.Rat, error) {
	maps := make([][]*big.Rat, 4)
	for quarter, controls := range correspondence.Numerators {
		denominator := correspondence.Denominators[quarter]
		if denominator <= 0 || len(controls) < 2 || len(controls) > 6 || controls[0] != 0 ||
			controls[len(controls)-1] != denominator {
			return nil, fmt.Errorf("%w: circular correspondence map %d needs two to six controls from zero to one", ErrUnsupported, quarter)
		}
		maps[quarter] = make([]*big.Rat, len(controls))
		for i, value := range controls {
			if i > 0 && value <= controls[i-1] {
				return nil, fmt.Errorf("%w: circular correspondence map %d must strictly increase", ErrUnsupported, quarter)
			}
			maps[quarter][i] = big.NewRat(value, denominator)
		}
	}
	return maps, nil
}

// Do builds the loft, registers the new body and returns it, or returns the
// error of the first gate that refuses, leaving the document unchanged. A nil
// ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *LoftSectionsCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationLoftSections, c.run)
}

func (c *LoftSectionsCommand) run(ctx context.Context) (*Body, error) {
	var corr loftCorrespondence
	if c.correspond.IsSet() {
		corr.set = true
		corr.m, corr.err = c.correspond.Get()
	}
	return loftSections(ctx, c.d, corr, c.sections)
}

// loftCorrespondence is what LoftSections's implementation reads of the
// CircularCorrespondence method: whether it was called, its maps and the error
// validating them.
type loftCorrespondence struct {
	set bool
	m   circularCorrespondenceMap
	err error
}

// loftSections is LoftSections's implementation. ctx is not nil, and the
// errors it returns carry no operation tag, except the refusal of a stored
// method value (agent-docs/command-api-design.md §11).
func loftSections(ctx context.Context, d *Document, corr loftCorrespondence, sections []LoftSection) (*Body, error) {
	if d == nil || len(sections) < 3 {
		return nil, errLoftSectionsCount
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if corr.err != nil {
		return nil, subjectError(OperationLoftSections, "CircularCorrespondence", corr.err)
	}
	maps, smoothCircularBands, emergingFlat := corr.m.maps, corr.m.smoothCircularBands, corr.m.emerging
	sketches := make([]*sketch.Sketch, len(sections))
	for i, section := range sections {
		sketches[i] = section.Sketch
	}
	if err := d.requireSameContainer(nil, sketches...); err != nil {
		return nil, err
	}
	if sketches[0] != nil {
		ctx = d.resultIn(ctx, d.ContainerOfSketch(sketches[0]))
	}
	if circularFitCandidate(sections) {
		return d.buildCircularFitSectionsMode(ctx, sections, maps, smoothCircularBands, emergingFlat)
	}
	if maps != nil {
		return nil, fmt.Errorf("%w: explicit circular correspondence needs circular and spline sections", ErrUnsupported)
	}
	if len(sections) > 3 && twistedFitCandidate(sections[0]) {
		return d.buildTwistedFitSections(ctx, sections)
	}
	if len(sections) == 4 {
		return d.buildFourRoundedSections(ctx, sections)
	}
	if len(sections) >= 5 {
		return d.buildManyRectangularSections(ctx, sections)
	}
	body, err := d.buildThreeSections(ctx, sections)
	if err != nil && errors.Is(err, ErrUnsupported) && twistedFitCandidate(sections[0]) {
		return d.buildTwistedFitSections(ctx, sections)
	}
	return body, err
}

// circularFitCandidate chooses the §14 route by source entity kinds; the
// builder authenticates every record before accepting it.
func circularFitCandidate(sections []LoftSection) bool {
	if len(sections) < 3 || sections[0].Profile == nil || len(sections[0].Profile.Holes) != 0 ||
		len(sections[0].Profile.Outer) != 4 {
		return false
	}
	for _, edge := range sections[0].Profile.Outer {
		if _, ok := edge.Entity.(*sketch.Arc); !ok {
			return false
		}
	}
	return slices.ContainsFunc(sections[1:], circularTargetCandidate)
}

func circularTargetCandidate(section LoftSection) bool {
	if section.Profile == nil || len(section.Profile.Holes) != 0 || len(section.Profile.Outer) != 4 {
		return false
	}
	fits := 0
	for _, edge := range section.Profile.Outer {
		switch edge.Entity.(type) {
		case *sketch.FitSpline, *sketch.Spline:
			fits++
		case *sketch.Line:
		default:
			return false
		}
	}
	return fits == 1
}

// buildCircularFitSections records the source sketches and builds §14's
// rational transition through the internal smooth-loft kernel.
func (d *Document) buildCircularFitSections(ctx context.Context, sections []LoftSection, maps [][]*big.Rat, smoothCircularBands bool) (*Body, error) {
	return d.buildCircularFitSectionsMode(ctx, sections, maps, smoothCircularBands, false)
}

func (d *Document) buildCircularFitSectionsMode(ctx context.Context, sections []LoftSection, maps [][]*big.Rat, smoothCircularBands, emergingFlat bool) (*Body, error) {
	if len(sections) > smoothloft.MaxTwistedSections {
		return nil, fmt.Errorf("%w: circular fitted LoftSections accepts at most %d sections",
			ErrUnsupported, smoothloft.MaxTwistedSections)
	}
	records := make([]profileRecord, len(sections))
	planes := make([]planeRecord, len(sections))
	work := make([]*freeform.FreeformWork, len(sections))
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := sectioncert.RecordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		work[i] = freeform.NewFreeformWork()
		if _, err := profile.FalsifyLoftRecordedArea(sketchArea, work[i]); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	built, err := smoothloft.BuildCircularFitsWithOptions(ctx, records, planes, work, maps, smoothCircularBands, emergingFlat)
	if err != nil {
		return nil, err
	}
	return d.buildSmoothSectionBody(ctx, built, planes[0], planes[len(planes)-1])
}

// buildThreeSections owns the routes of §§1–8, which take exactly three sections.
func (d *Document) buildThreeSections(ctx context.Context, sections []LoftSection) (*Body, error) {
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
				if err := d.commitOperationIn(ctx, nil, OperationLoftSections, nil, body); err != nil {
					return nil, err
				}
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

// twistedFitCandidate reports whether the first section is one fit spline
// closed by whole lines, the class of the twisted route (§13). It reads only
// the profile's entity kinds; the builder authenticates the records.
func twistedFitCandidate(section LoftSection) bool {
	if section.Profile == nil || len(section.Profile.Holes) != 0 ||
		len(section.Profile.Outer) < 3 || len(section.Profile.Outer) > 9 {
		return false
	}
	fits := 0
	for _, edge := range section.Profile.Outer {
		switch edge.Entity.(type) {
		case *sketch.FitSpline:
			fits++
		case *sketch.Line:
		default:
			return false
		}
	}
	return fits == 1
}

// buildTwistedFitSections builds §13's loft through sections that turn about
// the plane origin and carry one fitted edge.
func (d *Document) buildTwistedFitSections(ctx context.Context, sections []LoftSection) (*Body, error) {
	if len(sections) > smoothloft.MaxTwistedSections {
		return nil, fmt.Errorf("%w: twisted LoftSections accepts at most %d sections",
			ErrUnsupported, smoothloft.MaxTwistedSections)
	}
	records := make([]profileRecord, len(sections))
	planes := make([]planeRecord, len(sections))
	work := make([]*freeform.FreeformWork, len(sections))
	for i, section := range sections {
		if section.Sketch == nil || section.Profile == nil {
			return nil, fmt.Errorf("%w: LoftSections section %d needs a sketch and profile", ErrDegenerate, i)
		}
		profile, plane, sketchArea, err := sectioncert.RecordProfile(section.Sketch, section.Profile)
		if err != nil {
			return nil, err
		}
		work[i] = freeform.NewFreeformWork()
		if _, err := profile.FalsifyLoftRecordedArea(sketchArea, work[i]); err != nil {
			return nil, err
		}
		records[i], planes[i] = profile, plane
	}
	built, err := smoothloft.BuildTwistedFits(ctx, records, planes, work)
	if err != nil {
		return nil, err
	}
	return d.buildSmoothSectionBody(ctx, built, planes[0], planes[len(planes)-1])
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
	ref := d.producerAt(ctx)
	groups := make([]facetGroup, built.EdgeCount+2)
	for i := range built.EdgeCount {
		role := fmt.Sprintf("side(0,%d)", i)
		if i < len(built.SideRoles) && built.SideRoles[i] != "" {
			role = built.SideRoles[i]
		}
		groups[i] = facetGroup{Origins: []provenance.Ref{{Producer: ref,
			Role: role}}, Surface: surfacegeom.NURBSSurface{}}
		if built.EmergingPlaneX != nil && i >= 4 {
			frame, err := r3.NewFrame(r3.Vec{X: *built.EmergingPlaneX}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
			if err != nil {
				return nil, err
			}
			groups[i].Surface, groups[i].Planar = surfacegeom.Plane{Frame: frame}, true
		}
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
		CircularFitPrimitive:    built.CircularFitPrimitive,
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
	if err := d.commitOperationIn(ctx, nil, OperationLoftSections, nil, body); err != nil {
		return nil, err
	}
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
	ref := d.producerAt(ctx)
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: profiles[0], Frame: frame, Z0: 0, Z1: height, Xform: r3.Identity(),
	}, work))
}
