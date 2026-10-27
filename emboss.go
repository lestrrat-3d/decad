package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/embossplan"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/ribplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// EmbossMode says whether a relief stands out of a face or is cut into it.
type EmbossMode int

const (
	// EmbossRaised adds the relief on the outward side of the face.
	EmbossRaised EmbossMode = iota
	// EmbossEngraved cuts the relief into the body from the face.
	EmbossEngraved
)

// String names the mode.
func (m EmbossMode) String() string {
	switch m {
	case EmbossRaised:
		return "EmbossRaised"
	case EmbossEngraved:
		return "EmbossEngraved"
	default:
		return fmt.Sprintf("EmbossMode(%d)", int(m))
	}
}

// EmbossOption configures [Emboss].
type EmbossOption interface {
	option.Interface
	embossOption()
}

type embossOption struct{ option.Interface }

func (embossOption) embossOption() {}

type identEmbossDraft struct{}
type identEmbossRimFillet struct{}

// WithEmbossRimFillet rounds the relief's rim with a fillet of the given radius
// (agent-docs/emboss-design.md §7). The rim of a raised relief is the edges of
// its top face, and the rim of an engraving is the edges where its wall meets the
// face it is cut into. The radius is a positive length. The relief is built and
// filleted on a copy of b first, so a fillet the evaluator refuses leaves b
// live and the document unchanged; the copy leaves history entries.
func WithEmbossRimFillet(radius units.Value) EmbossOption {
	return embossOption{option.New(identEmbossRimFillet{}, radius)}
}

// WithEmbossDraft leans the relief's walls by a draft angle
// (agent-docs/emboss-design.md §6). A positive angle makes a raised relief
// narrower toward its top and an engraving narrower toward its floor, the way a
// moulded or cast part leaves its mould. A negative angle widens them. The angle
// is [WithTaper]'s: a draft at or past a right angle is ErrDegenerate, and the
// zero value is a straight wall.
func WithEmbossDraft(angle units.Value) EmbossOption {
	return embossOption{option.New(identEmbossDraft{}, angle)}
}

// Emboss raises or engraves the profiles on a planar or cylindrical face of b and
// returns the result, retiring b.
//
// face selects the face and must resolve to exactly one face. s is a sketch whose
// plane is the plane of that face, or on a cylindrical face a plane tangent to the
// cylinder (agent-docs/emboss-design.md §8), and profiles are closed
// profiles of s, such as the outlines of letters or a logo; a profile with
// holes keeps them. The relief is height tall: a raised relief stands height
// out of the face, and an engraved one is cut height into the body. The tool
// extends one extra millimetre to the other side of the face so that it
// overlaps the body instead of touching it (agent-docs/emboss-design.md). The
// result has the exactness Union or Cut gives for the same bodies.
//
// b MUST be a live solid. A nil context, body, sketch or selector, an empty
// or nil profile list, a non-positive height, a sketch plane that is more than
// 1e-6 mm off the face plane or not parallel to it, and a relief that
// certifiably misses b or changes no volume are ErrDegenerate. A face that is
// not an analytic plane or cylinder, nor a convex planar face of a faceted solid with exact
// vertices, is ErrUnsupported. Profiles must not touch each other
// in a shared plane. A boolean the evaluator cannot decide is the boolean's
// own error. Every failure leaves the document unchanged and b live.
func Emboss(ctx context.Context, b *Body, s *sketch.Sketch, profiles []*sketch.Profile, face FaceSelector,
	mode EmbossMode, height units.Value, opts ...EmbossOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an emboss`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: an emboss needs a solid body`, ErrNotSolid)
	}
	if mode != EmbossRaised && mode != EmbossEngraved {
		return nil, fmt.Errorf(`%w: unknown emboss mode %d`, ErrDegenerate, int(mode))
	}
	if s == nil || len(profiles) == 0 {
		return nil, fmt.Errorf(`%w: an emboss needs a sketch and at least one profile`, ErrDegenerate)
	}
	if face == nil {
		return nil, errNilSelector
	}
	var taper []ExtrudeOption
	var rim *units.Value
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		switch o.Ident().(type) {
		case identEmbossDraft:
			angle, _ := option.Get[units.Value](o)
			taper = []ExtrudeOption{WithTaper(angle)}
		case identEmbossRimFillet:
			r, _ := option.Get[units.Value](o)
			rim = &r
		}
	}
	rimRadius := 0.0
	if rim != nil {
		var err error
		if rimRadius, err = sectionrecord.MagnitudeIn(*rim, units.Length, units.Millimeter, "the relief rim fillet"); err != nil {
			return nil, err
		}
		if rimRadius == 0 {
			return nil, fmt.Errorf(`%w: a relief rim fillet needs a positive radius`, ErrDegenerate)
		}
	}
	h, err := sectionrecord.MagnitudeIn(height, units.Length, units.Millimeter, "the relief height")
	if err != nil {
		return nil, err
	}
	frame, err := s.Plane().Frame()
	if err != nil {
		return nil, fmt.Errorf(`%w: the sketch plane has no frame: %s`, ErrDegenerate, err)
	}
	site, err := resolveEmbossSite(ctx, b, face, frame)
	if err != nil {
		return nil, err
	}
	plane, outward := site.plane, VecMeasurement{Value: site.outward}
	extra := 0.0
	if site.cylinder != nil {
		if rim != nil {
			return nil, fmt.Errorf(`%w: a relief on a cylindrical face takes no rim fillet`, ErrUnsupported)
		}
		if extra, err = embossCurvedReach(ctx, d, s, profiles, frame, site, mode); err != nil {
			return nil, err
		}
	}
	sweep, err := embossplan.PlanCurved(frame, plane.Frame, outward.Value, h, mode == EmbossRaised, extra)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.requireSameContainer([]*Body{b}, s); err != nil {
		return nil, err
	}
	extent := TwoSided{
		One: DistanceSide{D: units.Millimeters(sweep.Along)},
		Two: DistanceSide{D: units.Millimeters(sweep.Against)},
	}
	run := func(target *Body) (*Body, error) {
		return embossOnto(ctx, d, target, s, profiles, mode, extent, taper)
	}
	if rim == nil {
		return run(b)
	}
	// The relief and its rim are tried on a copy first, so a fillet the evaluator
	// refuses leaves b live and the document unchanged.
	probe, err := Copy(ctx, b)
	if err != nil {
		return nil, err
	}
	defer removeTools([]*Body{probe})
	probed, err := run(probe)
	if err != nil {
		return nil, err
	}
	filleted, perr := filletEmbossRim(ctx, probed, plane, outward.Value, h, mode, rimRadius)
	removeTools([]*Body{probed, filleted})
	if perr != nil {
		return nil, perr
	}
	grown, err := run(b)
	if err != nil {
		return nil, err
	}
	return filletEmbossRim(ctx, grown, plane, outward.Value, h, mode, rimRadius)
}

// embossOnto builds the relief tools of the profiles and unions them into
// target, or cuts them from it, in one atomic commit that retires target.
func embossOnto(ctx context.Context, d *Document, target *Body, s *sketch.Sketch, profiles []*sketch.Profile,
	mode EmbossMode, extent TwoSided, taper []ExtrudeOption) (*Body, error) {
	var tools []*Body
	defer func() { removeTools(tools) }()
	toolCtx, capture := privateConstructionContext(ctx)
	for _, p := range profiles {
		if p == nil {
			return nil, fmt.Errorf(`%w: a nil profile names no relief`, ErrDegenerate)
		}
		tool, err := Extrude(toolCtx, d, s, p, extent, taper...)
		if err != nil {
			return nil, err
		}
		tools = append(tools, tool)
		disjoint, err := boundsDisjoint(target, tool)
		if err != nil {
			return nil, err
		}
		if disjoint {
			return nil, fmt.Errorf(`%w: a profile lies outside the body, so its relief would not reach it`, ErrDegenerate)
		}
	}
	op := meshbool.OpUnion
	if mode == EmbossEngraved {
		op = meshbool.OpCut
	}
	return combineTools(ctx, d, OperationEmboss, target, op, tools, false, func(result *Body) error {
		return requireReliefChangesVolume(target, result, mode)
	}, capture)
}

// requireReliefChangesVolume refuses a result whose volume certifiably did not
// move the way the mode says: a raised relief that adds nothing or an
// engraving that removes nothing.
func requireReliefChangesVolume(before, after *Body, mode EmbossMode) error {
	vb, err := before.Volume()
	if err != nil {
		return err
	}
	va, err := after.Volume()
	if err != nil {
		return err
	}
	b0, bb, a0, ab := vb.Value.Base(), vb.Bound.Base(), va.Value.Base(), va.Bound.Base()
	if mode == EmbossRaised && ribplan.AddsNothing(b0, bb, a0, ab) {
		return fmt.Errorf(`%w: the raised relief lies inside the body and adds no material`, ErrDegenerate)
	}
	if mode == EmbossEngraved && embossplan.RemovesNothing(b0, bb, a0, ab) {
		return fmt.Errorf(`%w: the engraving lies outside the material and removes none`, ErrDegenerate)
	}
	return nil
}

// embossSite is the plane an emboss lies on, its outward normal, and for a
// cylindrical face the cylinder.
type embossSite struct {
	plane   Plane
	outward r3.Vec
	// cylinder, when set, says the face is a cylinder and plane is tangent to it.
	cylinder *embossCylinder
}

// embossCylinder is the cylindrical face an emboss lies on.
type embossCylinder struct {
	radius  float64
	concave bool
	// axis is the cylinder's unit axis and lateral the unit direction in the
	// tangent plane at right angles to it. line is the signed position of the
	// line the plane touches the cylinder along, on lateral.
	axis, lateral r3.Vec
	line          float64
}

// resolveEmbossSite resolves face to a plane: a planar face is its own plane, and a
// cylindrical face gives the plane of the sketch frame when that is tangent to the
// cylinder.
func resolveEmbossSite(ctx context.Context, b *Body, face FaceSelector, sketchFrame r3.Frame) (embossSite, error) {
	if faces, err := face.SelectFaces(b); err == nil && len(faces) == 1 {
		if cyl, ok := faces[0].Surface().(Cylinder); ok {
			return cylinderEmbossSite(faces[0], cyl, sketchFrame)
		}
		if _, ok := faces[0].Surface().(Faceted); ok {
			site, err := facetedHoleSite(ctx, b, faces[0])
			if err != nil {
				return embossSite{}, err
			}
			return embossSite{plane: Plane{Frame: site.frame}, outward: site.outward}, nil
		}
	}
	plane, normal, err := resolvePlanarFace(b, face, "Emboss")
	if err != nil {
		return embossSite{}, err
	}
	return embossSite{plane: plane, outward: normal.Value}, nil
}

// cylinderEmbossSite checks that the sketch plane is tangent to cyl and reads the
// face's outward normal where it touches.
func cylinderEmbossSite(face *Face, cyl Cylinder, sketchFrame r3.Frame) (embossSite, error) {
	radius, err := sectionrecord.MagnitudeIn(cyl.Radius, units.Length, units.Millimeter, "the cylinder radius")
	if err != nil {
		return embossSite{}, err
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok {
		return embossSite{}, fmt.Errorf(`%w: the cylinder has no axis direction`, ErrDegenerate)
	}
	n := sketchFrame.N()
	if math.Abs(n.Dot(axis)) > embossplan.ParallelTol {
		return embossSite{}, fmt.Errorf(`%w: the sketch plane is not parallel to the cylinder's axis, so it is not tangent to it`, ErrDegenerate)
	}
	rel := sketchFrame.Origin().Sub(cyl.Origin)
	dist := rel.Dot(n)
	if off := math.Abs(math.Abs(dist) - radius); off > embossplan.PlaneTol {
		return embossSite{}, fmt.Errorf(`%w: the sketch plane lies %g mm from the cylinder's surface, so it is not tangent to it`, ErrDegenerate, off)
	}
	toward := n
	if dist < 0 {
		toward = n.Scale(-1)
	}
	// The line of contact: on the axis' side of the plane, radius along the normal's pointing back.
	axial := cyl.Origin.Add(axis.Scale(rel.Dot(axis)))
	touch := axial.Add(toward.Scale(radius))
	normal, err := face.NormalAt(touch)
	if err != nil {
		return embossSite{}, err
	}
	outward, ok := normal.Value.Normalize()
	if !ok {
		return embossSite{}, fmt.Errorf(`%w: the cylindrical face has no normal where the sketch plane touches it`, ErrDegenerate)
	}
	lateral, ok := n.Cross(axis).Normalize()
	if !ok {
		return embossSite{}, fmt.Errorf(`%w: the cylinder's axis and the sketch normal are parallel`, ErrDegenerate)
	}
	plane := Plane{Frame: sketchFrame}
	return embossSite{plane: plane, outward: outward, cylinder: &embossCylinder{
		radius: radius, concave: outward.Dot(toward) < 0, axis: axis, lateral: lateral,
		line: touch.Sub(sketchFrame.Origin()).Dot(lateral),
	}}, nil
}

// embossCurvedReach returns how much farther the relief's tool must reach past the
// tangent plane on its open side so it overlaps the cylinder across the whole width
// of the profiles: the sagitta R - sqrt(R² - w²) at the widest distance w of a
// profile from the line of contact, for a raised relief on a rod or an engraving in a
// bore, where the wall moves away from the tool's start. A relief that is wider than
// the cylinder is ErrDegenerate.
func embossCurvedReach(ctx context.Context, d *Document, s *sketch.Sketch, profiles []*sketch.Profile,
	frame r3.Frame, site embossSite, mode EmbossMode) (float64, error) {
	c := site.cylinder
	// A relief reaches the wall that falls away from the tangent plane (a convex wall) on its
	// inward side when raised, and the wall that rises toward it (a concave wall) on its
	// outward side when engraved.
	if (mode != EmbossRaised || c.concave) && (mode != EmbossEngraved || !c.concave) {
		return 0, nil
	}
	toolCtx, _ := privateConstructionContext(ctx)
	width := 0.0
	for _, p := range profiles {
		if p == nil {
			return 0, fmt.Errorf(`%w: a nil profile names no relief`, ErrDegenerate)
		}
		probe, err := Extrude(toolCtx, d, s, p, Distance{D: units.Millimeters(1), Dir: Along})
		if err != nil {
			return 0, err
		}
		box, err := probe.Bounds()
		removeTools([]*Body{probe})
		if err != nil {
			return 0, err
		}
		for _, x := range []float64{box.Min.X, box.Max.X} {
			for _, y := range []float64{box.Min.Y, box.Max.Y} {
				for _, z := range []float64{box.Min.Z, box.Max.Z} {
					at := r3.Vec{X: x, Y: y, Z: z}.Sub(frame.Origin()).Dot(c.lateral)
					width = math.Max(width, math.Abs(at-c.line))
				}
			}
		}
	}
	if width >= c.radius {
		return 0, fmt.Errorf(`%w: a relief %g mm from the line of contact is as wide as the %g mm radius cylinder it lies on`,
			ErrDegenerate, width, c.radius)
	}
	return c.radius - math.Sqrt(c.radius*c.radius-width*width), nil
}
