package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/holeplan"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/threadplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// HoleDepth is how deep a hole runs. Its variants are ThroughHole and
// BlindHole.
type HoleDepth interface{ holeDepth() }

// ThroughHole runs the hole out of the far side of the body.
type ThroughHole struct{}

// BlindHole stops the hole Depth below the face, measured along the hole axis.
// Depth is a positive length.
type BlindHole struct{ Depth units.Value }

func (ThroughHole) holeDepth() {}
func (BlindHole) holeDepth()   {}

// HoleHead is the widening at the mouth of a hole. Its variants are
// Counterbore and Countersink.
type HoleHead interface{ holeHead() }

// Counterbore cuts a flat-bottomed cylindrical recess Depth deep and Diameter
// wide at the mouth. Diameter is a length wider than the hole, and Depth is a
// length shorter than the hole.
type Counterbore struct {
	Diameter units.Value
	Depth    units.Value
}

// Countersink cuts a conical recess Diameter wide at the face. Angle is the
// included angle of the cone, strictly between 0 and 180 degrees; 90 degrees
// is the common metric flat-head screw. Diameter is a length wider than the
// hole, and the cone's depth is set by Diameter, Angle and the hole diameter.
type Countersink struct {
	Diameter units.Value
	Angle    units.Value
}

func (Counterbore) holeHead() {}
func (Countersink) holeHead() {}

// HoleShape is the cross-section of a hole that is not round. Its variants are
// RectangularHole, SlotHole, PolygonHole and OutlineHole.
type HoleShape interface{ holeShape() }

// RectangularHole is a hole whose bore is a Width by Height rectangle centred
// on the hole axis, with Width along the face frame's U axis before Angle
// turns it counter-clockwise about the axis. Width and Height are positive
// lengths and Angle is an angle.
type RectangularHole struct {
	Width, Height units.Value
	Angle         units.Value
}

func (RectangularHole) holeShape() {}

// SlotHole is a hole whose bore is a stadium: a strip Width wide with a half
// circle on each end, Length long from end to end. Length is a positive length
// greater than Width. Its long axis lies along the face frame's U axis before
// Angle turns it counter-clockwise about the hole axis. The bore is centred on
// the hole axis.
type SlotHole struct {
	Length, Width units.Value
	Angle         units.Value
}

func (SlotHole) holeShape() {}

// PolygonHole is a hole whose bore is a regular polygon of Sides sides, from 3
// to 64, AcrossFlats wide from one flat to the opposite one (a hexagon for a
// nut, say), centred on the hole axis. With a zero Angle a flat faces the face
// frame's U axis; Angle turns the polygon counter-clockwise about the axis.
// AcrossFlats is a positive length and Angle is an angle.
type PolygonHole struct {
	Sides       int
	AcrossFlats units.Value
	Angle       units.Value
}

func (PolygonHole) holeShape() {}

// OutlineHole is a hole whose bore is an arbitrary simple polygon: Corners are its corners about the
// hole axis in the face frame's U and V, in millimetres, in order round the polygon either way, with
// Angle turning them counter-clockwise about the axis (the zero Value is no turn). It takes from 3 to 256 corners that enclose
// an area and whose sides do not cross or touch but at a shared corner, otherwise ErrDegenerate. Boss
// does not take it.
type OutlineHole struct {
	Corners []Point2
	Angle   units.Value
}

func (OutlineHole) holeShape() {}

// HoleThread is a thread tapped into a round hole. The hole's Diameter is then
// the thread's minor (tap drill) diameter. Pitch is the axial advance per turn,
// a positive length. The thread is right-handed unless LeftHand is set. Length
// is the length to thread, measured from where the bore proper starts (the face
// of a plain hole, the floor of a counterbore or the bottom of a countersink): the
// zero Value threads the bore's whole length, running out of a through hole's far
// end, and a positive length no longer than the bore stops the thread there with
// the coil's flat end (agent-docs/hole-design.md §8).
type HoleThread struct {
	Pitch    units.Value
	LeftHand bool
	Length   units.Value
}

// HoleSpec describes one hole.
//
// Face selects the face the hole starts in and must resolve to exactly one
// face, an analytic plane or cylinder. At is a world point in millimetres that
// lies on that face's plane, or on the cylinder; the hole axis runs through it,
// into the body, square to the plane or along the cylinder's radius. Diameter is a length. Depth is
// required. Head is optional, and nil makes a plain hole; on a cylindrical face it cuts
// from the plane tangent there (agent-docs/hole-design.md §11).
//
// Shape is optional. A nil Shape makes a round hole of Diameter. A non-nil
// Shape replaces the round bore: Diameter MUST then be the zero units.Value and
// Head MUST be nil, each otherwise ErrDegenerate.
//
// Thread is optional. A non-nil Thread taps the round bore in the same commit
// (agent-docs/hole-design.md §8). It needs a nil Shape, ErrDegenerate otherwise.
// On a cylindrical face the thread starts at the tangent plane at At, and the
// cut is the evaluator's own ErrUnsupported where it cannot hold the pair
// (agent-docs/hole-design.md §12).
//
// Draft is optional and leans the round bore's wall (agent-docs/hole-design.md
// §9): a positive angle makes the bore narrower toward its bottom, a negative one
// wider, and the zero Value is a straight bore. Diameter is the bore's diameter
// at the face either way. It needs a nil Shape and a nil Thread, each otherwise
// ErrDegenerate, and a planar face, ErrUnsupported on a cylinder. An angle whose
// magnitude is at or past a right angle is ErrDegenerate, and one that closes the
// bore before its bottom is the drafted extrusion's own refusal.
type HoleSpec struct {
	Face     FaceSelector
	At       r3.Vec
	Diameter units.Value
	Depth    HoleDepth
	Head     HoleHead
	Shape    HoleShape
	Thread   *HoleThread
	Draft    units.Value
}

// holeTolerance is the chord tolerance, in millimetres, Hole reads At's
// containment with.
const holeTolerance = 0.05

// Hole cuts a round hole into b and returns the result, retiring b.
//
// The hole starts at At on the face spec.Face selects, runs along the face's
// inward normal for spec.Depth, and widens at the mouth by spec.Head. The
// result is the same body that Extrude, Revolve and Cut would give for the
// same tool shapes, so its measurements are as exact as those booleans make
// them; a round hole is Approximate because pi is never exact.
//
// b MUST be a live solid; a nil context, body or selector, a missing depth,
// a non-positive diameter or depth, a head not wider than the hole, a
// counterbore not shallower than the hole and a countersink deeper than the
// hole are ErrDegenerate. A face that is not an analytic plane is
// ErrUnsupported. A selector that does not resolve to one face is the
// selector's own error. An At more than 1e-6 mm off the face plane, or one
// that Body.LocatePoint certifies lies outside the body along the hole axis,
// is ErrDegenerate; an undecided location does not refuse. A boolean the
// evaluator cannot decide is the boolean's own error. Every failure leaves
// the document unchanged and b live.
func Hole(ctx context.Context, b *Body, spec HoleSpec) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationHole, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a hole`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Hole needs a solid body`, ErrNotSolid)
	}
	if spec.Face == nil {
		return nil, errNilSelector
	}
	in, err := holeInput(b, spec)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	plan, err := holeplan.Compute(in)
	if err != nil {
		return nil, err
	}
	if err := holeAxisInsideBody(ctx, b, in, plan); err != nil {
		return nil, err
	}
	var tap *holeTap
	if spec.Thread != nil {
		if spec.Shape != nil {
			return nil, fmt.Errorf(`%w: a thread needs a round hole, not a shaped one`, ErrDegenerate)
		}
		_, through := spec.Depth.(ThroughHole)
		if tap, err = planHoleThread(spec.Thread, through, in, plan); err != nil {
			return nil, err
		}
	}
	toolCtx, capture := privateConstructionContext(ctx)
	tools, err := buildHoleTools(toolCtx, d, plan)
	if err != nil {
		return nil, err
	}
	defer func() { removeTools(tools) }()
	if tap != nil {
		coil, err := buildThreadCoil(toolCtx, d, tap.frame, tap.plan, tap.pitch, tap.leftHand)
		if err != nil {
			return nil, err
		}
		// The coil is cut after the bore and its head, as Thread cuts a bore that exists.
		tools = append(tools, coil)
	}
	// A hole on a cylindrical face (in.Margin is set only for one) meets the body on
	// the mesh path, where a second cut of the bore's head would find a target that
	// already holds the first cut's coarse mesh. The tools are united first, so one
	// cut serves.
	return combineTools(ctx, d, OperationHole, b, meshbool.OpCut, tools, in.Margin != 0, nil, capture)
}

// holeInput resolves spec into the plain-data request holeplan reads.
func holeInput(b *Body, spec HoleSpec) (holeplan.Input, error) {
	var in holeplan.Input
	site, err := resolveHoleSite(b, spec)
	if err != nil {
		return in, err
	}
	box, err := b.Bounds()
	if err != nil {
		return in, err
	}
	in.Frame = site.frame
	in.Outward = site.outward
	in.At = spec.At
	if spec.Draft != (units.Value{}) {
		if in.Draft, err = readDraft(spec.Draft, "the hole draft"); err != nil {
			return in, err
		}
		switch {
		case spec.Shape != nil:
			return in, fmt.Errorf(`%w: a shaped hole takes no draft`, ErrDegenerate)
		case spec.Thread != nil:
			return in, fmt.Errorf(`%w: a threaded hole is straight and takes no draft`, ErrDegenerate)
		case site.cylinder != nil:
			return in, fmt.Errorf(`%w: a draft in a hole on a cylindrical face is not built`, ErrUnsupported)
		}
	}
	if spec.Shape != nil {
		if err := holeShapeInput(&in, spec); err != nil {
			return in, err
		}
	} else if in.Diameter, err = sectionrecord.MagnitudeIn(spec.Diameter, units.Length, units.Millimeter, "the hole diameter"); err != nil {
		return in, err
	}
	bound, err := box.Bound.In(units.Millimeter)
	if err != nil {
		return in, fmt.Errorf(`%w: the body bounds are not representable: %s`, ErrNotFinite, err)
	}
	in.Reach = holeplan.Reach(box.Min, box.Max, site.frame.Origin(), site.outward, bound)
	if err := holeDepthInput(&in, spec.Depth); err != nil {
		return in, err
	}
	if err := holeHeadInput(&in, spec.Head); err != nil {
		return in, err
	}
	if site.cylinder != nil {
		if in.Margin, err = site.cylinder.margin(in); err != nil {
			return in, err
		}
	}
	return in, nil
}

// holeSite is where a hole starts: the frame whose plane is tangent to the
// face at the hole axis, the face's outward normal there, and for a cylinder
// the cylinder.
type holeSite struct {
	frame    r3.Frame
	outward  r3.Vec
	cylinder *curvedHole
}

// curvedHole is the cylindrical face a hole starts in.
type curvedHole struct {
	radius  float64
	concave bool
}

// margin returns how far the tools start outside the tangent plane, from the
// largest radius the bore's section reaches.
func (c *curvedHole) margin(in holeplan.Input) (float64, error) {
	reach := in.Diameter / 2
	if in.Head.Kind != holeplan.HeadNone {
		// A head is wider than the bore and the tool that cuts it must clear the wall as
		// far out as it reaches.
		reach = math.Max(reach, in.Head.Diameter/2)
	}
	switch {
	case in.Rect != nil:
		reach = math.Hypot(in.Rect.Width, in.Rect.Height) / 2
	case in.Poly != nil:
		reach = in.Poly.Circumradius()
	case in.Outline != nil:
		reach = in.Outline.Reach()
	case in.Slot != nil:
		reach = in.Slot.Length / 2
	}
	return holeplan.CurvedMargin(c.radius, reach, c.concave)
}

// resolveHoleSite resolves spec.Face to exactly one analytic plane or cylinder.
// On a plane the site is the plane. On a cylinder the hole axis runs along the
// radial line through spec.At, which MUST lie on the cylinder, and the site is
// the plane tangent to the cylinder there.
func resolveHoleSite(b *Body, spec HoleSpec) (holeSite, error) {
	faces, err := spec.Face.SelectFaces(b)
	if err != nil {
		return holeSite{}, err
	}
	if len(faces) != 1 {
		return holeSite{}, fmt.Errorf(`%w: Hole needs exactly one face, the selector matched %d`, ErrDegenerate, len(faces))
	}
	switch surface := faces[0].Surface().(type) {
	case Plane:
		normal, err := faces[0].NormalAt(surface.Frame.Origin())
		if err != nil {
			return holeSite{}, err
		}
		return holeSite{frame: surface.Frame, outward: normal.Value}, nil
	case Cylinder:
		return cylinderHoleSite(faces[0], surface, spec.At)
	default:
		return holeSite{}, fmt.Errorf(`%w: Hole needs an analytic planar or cylindrical face, this one is %T`,
			ErrUnsupported, surface)
	}
}

// cylinderHoleSite builds the site of a hole whose centre is the point centre on the cylinder of face.
func cylinderHoleSite(face *Face, cyl Cylinder, centre r3.Vec) (holeSite, error) {
	radius, err := sectionrecord.MagnitudeIn(cyl.Radius, units.Length, units.Millimeter, "the cylinder radius")
	if err != nil {
		return holeSite{}, err
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok {
		return holeSite{}, fmt.Errorf(`%w: the cylinder has no axis direction`, ErrDegenerate)
	}
	rel := centre.Sub(cyl.Origin)
	radial, ok := rel.Sub(axis.Scale(rel.Dot(axis))).Normalize()
	if !ok {
		return holeSite{}, fmt.Errorf(`%w: the hole centre lies on the cylinder's axis`, ErrDegenerate)
	}
	if off := rel.Sub(axis.Scale(rel.Dot(axis))).Len() - radius; off > holeplan.OnPlaneTol || off < -holeplan.OnPlaneTol {
		return holeSite{}, fmt.Errorf(`%w: the hole centre lies %g mm off the cylindrical face`, ErrDegenerate, off)
	}
	normal, err := face.NormalAt(centre)
	if err != nil {
		return holeSite{}, err
	}
	outward := radial
	if normal.Value.Dot(radial) < 0 {
		outward = radial.Scale(-1)
	}
	frame, err := r3.NewFrame(centre, axis, outward.Cross(axis))
	if err != nil {
		return holeSite{}, fmt.Errorf(`%w: the hole frame is degenerate: %s`, ErrDegenerate, err)
	}
	return holeSite{frame: frame, outward: outward,
		cylinder: &curvedHole{radius: radius, concave: outward.Dot(radial) < 0}}, nil
}

// holeShapeInput reads a non-round bore into in.
func holeShapeInput(in *holeplan.Input, spec HoleSpec) error {
	if spec.Diameter != (units.Value{}) {
		return fmt.Errorf(`%w: a shaped hole takes its size from the shape, so Diameter must be zero`, ErrDegenerate)
	}
	if spec.Head != nil {
		return fmt.Errorf(`%w: a shaped hole takes no counterbore or countersink`, ErrDegenerate)
	}
	switch shape := spec.Shape.(type) {
	case RectangularHole:
		width, err := sectionrecord.MagnitudeIn(shape.Width, units.Length, units.Millimeter, "the rectangular hole width")
		if err != nil {
			return err
		}
		height, err := sectionrecord.MagnitudeIn(shape.Height, units.Length, units.Millimeter, "the rectangular hole height")
		if err != nil {
			return err
		}
		angle, err := sectionrecord.MagnitudeIn(shape.Angle, units.Angle, units.Radian, "the rectangular hole angle")
		if err != nil {
			return err
		}
		in.Rect = &holeplan.Rect{Width: width, Height: height, Angle: angle}
	case PolygonHole:
		flats, err := sectionrecord.MagnitudeIn(shape.AcrossFlats, units.Length, units.Millimeter, "the polygonal hole width across flats")
		if err != nil {
			return err
		}
		angle, err := sectionrecord.MagnitudeIn(shape.Angle, units.Angle, units.Radian, "the polygonal hole angle")
		if err != nil {
			return err
		}
		in.Poly = &holeplan.Poly{Sides: shape.Sides, AcrossFlats: flats, Angle: angle}
	case OutlineHole:
		angle := 0.0 // the zero Value is no turn
		if shape.Angle != (units.Value{}) {
			var err error
			if angle, err = sectionrecord.MagnitudeIn(shape.Angle, units.Angle, units.Radian, "the outline hole angle"); err != nil {
				return err
			}
		}
		corners := make([][2]float64, len(shape.Corners))
		for i, c := range shape.Corners {
			corners[i] = [2]float64{c.U, c.V}
		}
		in.Outline = &holeplan.Outline{Corners: corners, Angle: angle}
	case SlotHole:
		length, err := sectionrecord.MagnitudeIn(shape.Length, units.Length, units.Millimeter, "the slot length")
		if err != nil {
			return err
		}
		width, err := sectionrecord.MagnitudeIn(shape.Width, units.Length, units.Millimeter, "the slot width")
		if err != nil {
			return err
		}
		angle, err := sectionrecord.MagnitudeIn(shape.Angle, units.Angle, units.Radian, "the slot angle")
		if err != nil {
			return err
		}
		in.Slot = &holeplan.Slot{Length: length, Width: width, Angle: angle}
	default:
		return fmt.Errorf(`%w: unknown hole shape %T`, ErrDegenerate, spec.Shape)
	}
	return nil
}

func holeDepthInput(in *holeplan.Input, depth HoleDepth) error {
	switch v := depth.(type) {
	case ThroughHole:
		in.Through = true
	case BlindHole:
		m, err := sectionrecord.MagnitudeIn(v.Depth, units.Length, units.Millimeter, "the hole depth")
		if err != nil {
			return err
		}
		in.Depth = m
	default:
		return fmt.Errorf(`%w: a hole needs a depth, ThroughHole or BlindHole`, ErrDegenerate)
	}
	return nil
}

func holeHeadInput(in *holeplan.Input, head HoleHead) error {
	switch v := head.(type) {
	case nil:
		return nil
	case Counterbore:
		dia, err := sectionrecord.MagnitudeIn(v.Diameter, units.Length, units.Millimeter, "the counterbore diameter")
		if err != nil {
			return err
		}
		depth, err := sectionrecord.MagnitudeIn(v.Depth, units.Length, units.Millimeter, "the counterbore depth")
		if err != nil {
			return err
		}
		in.Head = holeplan.Head{Kind: holeplan.HeadCounterbore, Diameter: dia, Depth: depth}
	case Countersink:
		dia, err := sectionrecord.MagnitudeIn(v.Diameter, units.Length, units.Millimeter, "the countersink diameter")
		if err != nil {
			return err
		}
		angle, err := sectionrecord.MagnitudeIn(v.Angle, units.Angle, units.Radian, "the countersink angle")
		if err != nil {
			return err
		}
		in.Head = holeplan.Head{Kind: holeplan.HeadCountersink, Diameter: dia, Angle: angle}
	default:
		return fmt.Errorf(`%w: unknown hole head %T`, ErrDegenerate, head)
	}
	return nil
}

// holeAxisInsideBody refuses a hole whose axis, a quarter of the bore in, lies
// certifiably outside b. It can only refuse: an undecided or unreadable
// location lets the hole proceed (CLAUDE.md "reject-only").
func holeAxisInsideBody(ctx context.Context, b *Body, in holeplan.Input, plan holeplan.Plan) error {
	probe := in.At.Sub(in.Outward.Scale(plan.Bore.Length / 4))
	where, err := b.LocatePoint(ctx, probe, units.Millimeters(holeTolerance))
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return ctxErr
		}
		return nil
	}
	if where == PointOutside {
		return fmt.Errorf(`%w: the hole axis at %v lies outside the body`, ErrDegenerate, in.At)
	}
	return nil
}

// buildHoleTools builds the tool bodies of plan in d, the bore first and the
// head after it. On error it removes the tools already built.
func buildHoleTools(ctx context.Context, d *Document, plan holeplan.Plan) ([]*Body, error) {
	var tools []*Body
	fail := func(err error) ([]*Body, error) {
		removeTools(tools)
		return nil, err
	}
	if plan.Sink != nil && plan.Sink.IncludesBore {
		whole, err := holeSink(ctx, d, plan.Sink)
		if err != nil {
			return fail(err)
		}
		return []*Body{whole}, nil
	}
	bore, err := holeBore(ctx, d, plan)
	if err != nil {
		return fail(err)
	}
	tools = append(tools, bore)
	switch {
	case plan.Counterbore != nil:
		head, err := holeCylinder(ctx, d, plan, *plan.Counterbore, 0)
		if err != nil {
			return fail(err)
		}
		tools = append(tools, head)
	case plan.Sink != nil:
		head, err := holeSink(ctx, d, plan.Sink)
		if err != nil {
			return fail(err)
		}
		tools = append(tools, head)
	}
	return tools, nil
}

// holeBore builds the main drill: a prism over the rectangle or stadium when
// plan holds one, a cylinder otherwise.
func holeBore(ctx context.Context, d *Document, plan holeplan.Plan) (*Body, error) {
	if plan.Corners == nil && plan.SlotTool == nil {
		return holeCylinder(ctx, d, plan, plan.Bore, plan.BoreDraft)
	}
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(plan.Plane)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	fixed := func(p [2]float64) *sketch.Point {
		pt := s.CreatePoint(p[0], p[1])
		s.Fix(pt)
		return pt
	}
	if plan.Corners != nil {
		points := make([]*sketch.Point, len(plan.Corners))
		for i, c := range plan.Corners {
			points[i] = fixed(c)
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	} else {
		t := plan.SlotTool
		p0, p1, p2, p3 := fixed(t.P0), fixed(t.P1), fixed(t.P2), fixed(t.P3)
		start, end := fixed(t.Start), fixed(t.End)
		s.CreateLine(p0, p1)
		s.CreateArc(end, p1, p2)
		s.CreateLine(p2, p3)
		s.CreateArc(start, p3, p0)
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	dir := Along
	if plan.Against {
		dir = Against
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(plan.Bore.Tool()), Dir: dir})
}

// holeCylinder extrudes one circle on the face plane inward by c.Length.
func holeCylinder(ctx context.Context, d *Document, plan holeplan.Plan, c holeplan.Cylinder, draft float64) (*Body, error) {
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(plan.Plane)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	centre := s.CreatePoint(plan.U, plan.V)
	s.Fix(centre)
	// The tool starts Margin outside the face and a draft leans its wall from
	// there, so its circle is as much wider than the bore as the wall leans over
	// that margin: the wall is then the bore's radius at the face.
	s.CreateCircle(centre, c.Radius+c.Margin*math.Tan(draft))
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	dir := Along
	if plan.Against {
		dir = Against
	}
	var taper []ExtrudeOption
	if draft != 0 {
		taper = []ExtrudeOption{WithTaper(units.Radians(draft))}
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(c.Tool()), Dir: dir}, taper...)
}

// holeSink revolves the countersink cone profile about the hole axis.
func holeSink(ctx context.Context, d *Document, sink *holeplan.Sink) (*Body, error) {
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(sink.Plane)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	points := make([]*sketch.Point, len(sink.Profile))
	for i, p := range sink.Profile {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 1}}
	return Revolve(ctx, d, s, s.Profiles()[0], axis, FullRevolution{})
}

// holeTap is the thread of a tapped hole in the plain data buildThreadCoil reads.
type holeTap struct {
	frame    r3.Frame
	plan     threadplan.Plan
	pitch    float64
	leftHand bool
}

// planHoleThread plans thread on the round bore of plan. Its frame has its
// origin on the hole axis where the bore proper starts, its V axis into the body
// and its U axis in the face plane. through is whether the hole runs through the
// body, so that the groove may run out of its far end.
func planHoleThread(thread *HoleThread, through bool, in holeplan.Input, plan holeplan.Plan) (*holeTap, error) {
	pitch, err := sectionrecord.MagnitudeIn(thread.Pitch, units.Length, units.Millimeter, "the thread pitch")
	if err != nil {
		return nil, err
	}
	span := plan.Bore.Length - plan.HeadDepth
	partial := thread.Length != (units.Value{})
	if partial {
		threaded, err := sectionrecord.MagnitudeIn(thread.Length, units.Length, units.Millimeter, "the thread length")
		if err != nil {
			return nil, err
		}
		if threaded > span {
			return nil, fmt.Errorf(`%w: a thread length of %g mm is longer than the %g mm bore`, ErrDegenerate, threaded, span)
		}
		span = threaded
	} else if !through {
		// A blind bore's thread stops at its bottom rather than running past it.
		partial = true
	}
	threadPlan, err := threadplan.Compute(in.Diameter/2, pitch, span, false, partial)
	if err != nil {
		return nil, err
	}
	inward := in.Outward.Scale(-1)
	origin := in.Frame.ToWorldUV(plan.U, plan.V).Add(inward.Scale(plan.HeadDepth))
	frame, err := r3.NewFrame(origin, in.Frame.U(), inward)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thread frame is degenerate: %s`, ErrDegenerate, err)
	}
	return &holeTap{frame: frame, plan: threadPlan, pitch: pitch, leftHand: thread.LeftHand}, nil
}

// readDraft reads a signed draft angle in radians. A wrong unit kind is
// ErrUnitKind and a magnitude at or past a right angle is ErrDegenerate.
func readDraft(angle units.Value, what string) (float64, error) {
	rad, err := angle.In(units.Radian)
	if err != nil {
		return 0, fmt.Errorf(`%w: %s must be an angle: %s`, ErrUnitKind, what, err)
	}
	if !(math.Abs(rad) < math.Pi/2) {
		return 0, fmt.Errorf(`%w: %s must lie strictly within a quarter turn of straight`, ErrDegenerate, what)
	}
	return rad, nil
}
