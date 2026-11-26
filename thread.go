package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/threadplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ThreadForm is the groove's cross-section, passed to [ThreadCommand.Form]. The
// zero ThreadForm is the 60 degree ISO basic profile. [TrapezoidThreadForm] makes a
// symmetric trapezoidal groove, such as an Acme or trapezoidal thread, and
// [ButtressThreadForm] an asymmetric one (agent-docs/thread-design.md §6). The
// groove mouth opens a twentieth of the pitch past the face, as the ISO groove's
// does.
type ThreadForm struct {
	angle, depth, flat    units.Value
	flankLead, flankTrail units.Value
}

// TrapezoidThreadForm returns a symmetric trapezoidal groove. angle is the
// included angle between the two flanks, above zero and up to 120 degrees, depth
// is how far the groove reaches from the face to its root and flat the width of
// the root, both positive lengths.
func TrapezoidThreadForm(angle, depth, flat units.Value) ThreadForm {
	return ThreadForm{angle: angle, depth: depth, flat: flat}
}

// ButtressThreadForm returns an asymmetric groove, a buttress thread's. lead and
// trail are the lean of each flank from the radial direction, lead of the flank
// toward the start of the face (its least position along the axis) and trail of
// the one toward its far end. Each is an angle above zero and below 90 degrees.
// depth and flat are as in [TrapezoidThreadForm].
func ButtressThreadForm(lead, trail, depth, flat units.Value) ThreadForm {
	return ThreadForm{flankLead: lead, flankTrail: trail, depth: depth, flat: flat}
}

// threadSpec is the request a [ThreadCommand] hands to the implementation: the
// constructor arguments and the methods that were called.
type threadSpec struct {
	face      FaceSelector
	pitch     units.Value
	leftHand  bool
	length    units.Value
	lengthSet bool
	form      ThreadForm
	leadIn    units.Value
	leadAngle units.Value
	leadSet   bool
	angleSet  bool
	// the subjects name the method a fault of that value points at.
	lengthSubject, formSubject, leadInSubject, leadAngleSubject string
}

// Thread returns the command that cuts a 60 degree ISO metric thread of the given
// pitch into a cylindrical face of b and returns the threaded body, retiring b
// (agent-docs/thread-design.md). face must resolve to exactly one face. pitch is
// the axial advance per turn, a positive length. The thread is right-handed and
// runs the whole face unless [ThreadCommand.LeftHand] or [ThreadCommand.Length]
// is called; [ThreadCommand.Form] changes the groove and [ThreadCommand.LeadIn]
// chamfers the face's ends.
//
// A face whose outward normal points away from the axis is a rod: it is the
// thread's major diameter and a groove is cut into it. A face whose outward
// normal points toward the axis is a bore: it is the thread's minor (drilled)
// diameter and the groove is cut out of the wall around it. The groove follows
// the ISO basic profile, with an external root flat of an eighth of the pitch
// and an internal one of a quarter, and runs the whole length of the face, or
// the [ThreadCommand.Length] of it. A groove over the whole face runs out past
// both of its ends, so a larger face or a shoulder beside the threaded one is cut
// as well: thread a rod that ends at the thread, or give a length.
//
// The thread is a Coil of the groove profile cut from b, so the result is a
// mesh Boolean's: faceted, with the Cut's own bounds (agent-docs/helix-design.md
// §9). Its volume reading encloses the closed form in the thread design, and
// Verify reads it as the faceted verdict allows.
//
// b MUST be a live solid; a nil context, body or selector is ErrDegenerate. A
// selector that resolves to other than one face is ErrDegenerate, and a face
// that is not an analytic cylinder is ErrUnsupported. A pitch that is not a
// positive finite length is ErrDegenerate or ErrUnitKind, and one too coarse
// for the radius, so that the groove would reach the axis, is ErrDegenerate. A
// Coil or Cut the evaluator cannot decide is that call's own error. Every
// failure leaves b live and the document unchanged.
func Thread(b *Body, face FaceSelector, pitch units.Value) *ThreadCommand {
	c := &ThreadCommand{b: b, spec: threadSpec{face: copyFaceSelector(face), pitch: pitch}}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// ThreadCommand cuts a thread into a cylindrical face. Build it with [Thread].
type ThreadCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	spec  threadSpec      // the constructor arguments, copied, and the methods called
}

// LeftHand makes the thread left-handed. It is right-handed otherwise.
func (c *ThreadCommand) LeftHand() *ThreadCommand {
	c.spec.leftHand = true
	return c
}

// Length threads only length of the face, a positive length measured from the end
// of the face with the least position along the axis. A length no longer than the
// face stops the thread there, with the coil's flat end and no runout
// (agent-docs/thread-design.md §5); a longer one is ErrDegenerate, returned by Do
// naming Length as the subject. Without it the thread runs the whole face. A
// second call replaces the first.
func (c *ThreadCommand) Length(length units.Value) *ThreadCommand {
	c.spec.length, c.spec.lengthSet = length, true
	return c
}

// Form sets the groove's cross-section (agent-docs/thread-design.md §6). A form
// with a missing or out-of-range value is ErrDegenerate, ErrUnitKind or
// ErrNegativeMagnitude, returned by Do naming Form as the subject. A second call
// replaces the first.
func (c *ThreadCommand) Form(form ThreadForm) *ThreadCommand {
	c.spec.form = form
	return c
}

// LeadIn cuts a 45 degree chamfer of the given distance on each convex end of the
// threaded face before the thread, so a screw or a nut starts without a burr
// (agent-docs/thread-design.md §7). distance is a positive length. A face with no
// convex circular end is ErrDegenerate. A second call to LeadIn or
// [ThreadCommand.LeadInAngle] replaces the first.
func (c *ThreadCommand) LeadIn(distance units.Value) *ThreadCommand {
	c.spec.leadIn, c.spec.leadAngle, c.spec.leadSet, c.spec.angleSet = distance, units.Value{}, true, false
	return c
}

// LeadInAngle is [ThreadCommand.LeadIn] with the bevel at angle to the threaded
// face's end face, in place of 45 degrees: distance stays the bevel's distance
// along the end face and the bevel runs distance·tan(angle) along the threaded
// face. angle is above zero and below 90 degrees, ErrDegenerate otherwise,
// returned by Do naming LeadInAngle as the subject (agent-docs/thread-design.md
// §10). A second call to LeadIn or LeadInAngle replaces the first.
func (c *ThreadCommand) LeadInAngle(distance, angle units.Value) *ThreadCommand {
	c.spec.leadIn, c.spec.leadAngle, c.spec.leadSet, c.spec.angleSet = distance, angle, true, true
	return c
}

// Do cuts the thread and retires the body, or returns the error of the first gate
// that refuses, leaving the body live and the document unchanged. A nil ctx is
// ErrDegenerate, and a second call returns ErrConsumed.
func (c *ThreadCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationThread, c.run)
}

func (c *ThreadCommand) run(ctx context.Context) (*Body, error) {
	spec := c.spec
	spec.lengthSubject, spec.formSubject, spec.leadInSubject = "Length", "Form", "LeadIn"
	spec.leadAngleSubject = "LeadInAngle"
	return thread(ctx, c.b, spec)
}

// thread is the implementation of Thread. ctx is not nil, and the errors it
// returns carry no operation tag, except the refusal of a stored method value
// (agent-docs/command-api-design.md §11).
func thread(ctx context.Context, b *Body, spec threadSpec) (*Body, error) {
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Thread needs a solid body`, ErrNotSolid)
	}
	if spec.face == nil {
		return nil, errNilSelector
	}
	pitch, err := sectionrecord.MagnitudeIn(spec.pitch, units.Length, units.Millimeter, "the thread pitch")
	if err != nil {
		return nil, err
	}
	face, cyl, err := resolveCylindricalFace(b, spec.face)
	if err != nil {
		return nil, err
	}
	frame, length, external, radius, err := threadFrame(face, cyl)
	if err != nil {
		return nil, err
	}
	partial := spec.lengthSet
	if partial {
		threaded, err := sectionrecord.MagnitudeIn(spec.length, units.Length, units.Millimeter, "the thread length")
		if err != nil {
			return nil, subjectFault(OperationThread, spec.lengthSubject, err)
		}
		if threaded > length {
			return nil, subjectFault(OperationThread, spec.lengthSubject,
				fmt.Errorf(`%w: a thread length of %g mm is longer than the %g mm face`, ErrDegenerate, threaded, length))
		}
		length = threaded
	}
	form, err := threadForm(spec.form)
	if err != nil {
		return nil, subjectFault(OperationThread, spec.formSubject, err)
	}
	plan, err := threadplan.ComputeForm(radius, pitch, length, external, partial, form)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	cut := func(target *Body, consumed ...OperationInput) (*Body, error) {
		toolCtx, capture := privateConstructionContext(ctx)
		coil, err := buildThreadCoil(toolCtx, d, frame, plan, pitch, spec.leftHand, true)
		if err != nil {
			return nil, err
		}
		tools := []*Body{coil}
		defer removeTools(tools)
		return combineTools(ctx, d, OperationThread, target, meshbool.OpCut, tools, false, nil, capture, consumed...)
	}
	if !spec.leadSet {
		return cut(b)
	}
	leadIn, err := sectionrecord.MagnitudeIn(spec.leadIn, units.Length, units.Millimeter, "the thread lead-in")
	if err != nil {
		return nil, subjectFault(OperationThread, spec.leadInSubject, err)
	}
	if leadIn == 0 {
		return nil, subjectFault(OperationThread, spec.leadInSubject,
			fmt.Errorf(`%w: a thread lead-in needs a positive distance`, ErrDegenerate))
	}
	leadAngle := 0.0
	if spec.angleSet {
		leadAngle, err = sectionrecord.MagnitudeIn(spec.leadAngle, units.Angle, units.Radian, "the thread lead-in angle")
		if err != nil {
			return nil, subjectFault(OperationThread, spec.leadAngleSubject, err)
		}
		// A bevel along the end face or the threaded face is no chamfer.
		if leadAngle <= 0 || leadAngle >= math.Pi/2 {
			return nil, subjectFault(OperationThread, spec.leadAngleSubject,
				fmt.Errorf(`%w: a thread lead-in angle must lie above zero and below 90 degrees`, ErrDegenerate))
		}
	}
	ends := threadLeadInEdges(face)
	if len(ends) == 0 {
		return nil, fmt.Errorf(`%w: the threaded face has no convex circular end to chamfer for a lead-in`, ErrDegenerate)
	}
	bevel := func(target *Body) (*Body, error) {
		if leadAngle == 0 {
			out, err := chamfer(ctx, target, edgesQuery(ends), units.Millimeters(leadIn), chamferSettings{})
			return out, operationError(OperationChamfer, err)
		}
		out, err := chamferAngle(ctx, target, edgesQuery(ends), units.Millimeters(leadIn), Faces(Planar()), units.Radians(leadAngle))
		return out, operationError(OperationChamfer, err)
	}
	// Build on a copy, then retire b together with the chamfered copy only when
	// the cut succeeds. The frame and plan still come from b's unchamfered face.
	probe, err := copyAsIs(ctx, b)
	if err != nil {
		return nil, err
	}
	defer removeTools([]*Body{probe})
	chamfered, err := bevel(probe)
	if err != nil {
		return nil, err
	}
	defer removeTools([]*Body{chamfered})
	return cut(chamfered, OperationInput{Body: b, Role: operationRoleSource, Use: InputConsumed})
}

// threadLeadInEdges lists the convex circular edges of face, the rims at its
// ends where a lead-in chamfer bevels the thread's start.
func threadLeadInEdges(face *Face) []*Edge {
	var out []*Edge
	for _, e := range face.Edges() {
		if _, ok := e.Curve().(Circle3); ok && e.IsConvex() {
			out = append(out, e)
		}
	}
	return out
}

// threadForm reads a ThreadForm into the planner's millimetre form: the zero
// form is the ISO profile, and a partly set one is ErrDegenerate.
func threadForm(f ThreadForm) (threadplan.Form, error) {
	if f == (ThreadForm{}) {
		return threadplan.Form{}, nil
	}
	asymmetric := f.flankLead != (units.Value{}) || f.flankTrail != (units.Value{})
	if asymmetric {
		if f.angle != (units.Value{}) || f.flankLead == (units.Value{}) || f.flankTrail == (units.Value{}) {
			return threadplan.Form{}, fmt.Errorf(`%w: an asymmetric thread form sets both flanks and no angle`, ErrDegenerate)
		}
	}
	if f.depth == (units.Value{}) || f.flat == (units.Value{}) || (!asymmetric && f.angle == (units.Value{})) {
		return threadplan.Form{}, fmt.Errorf(`%w: a thread form needs an angle, a depth and a root flat, all three or none`, ErrDegenerate)
	}
	flank := func(v units.Value, what string, limit float64) (float64, error) {
		if v.Kind() != units.Angle {
			return 0, fmt.Errorf(`%w: the thread form %s must be an angle`, ErrUnitKind, what)
		}
		angle := v.Base()
		switch {
		case math.IsNaN(angle) || math.IsInf(angle, 0):
			return 0, fmt.Errorf(`%w: the thread form %s is not finite`, ErrNotFinite, what)
		case angle < 0:
			return 0, fmt.Errorf(`%w: the thread form %s is negative`, ErrNegativeMagnitude, what)
		case angle == 0 || angle > limit:
			return 0, fmt.Errorf(`%w: the thread form %s must be above zero and at most %g degrees`, ErrDegenerate, what, limit*180/math.Pi)
		}
		return angle, nil
	}
	depth, err := sectionrecord.MagnitudeIn(f.depth, units.Length, units.Millimeter, "the thread form depth")
	if err != nil {
		return threadplan.Form{}, err
	}
	flat, err := sectionrecord.MagnitudeIn(f.flat, units.Length, units.Millimeter, "the thread form root flat")
	if err != nil {
		return threadplan.Form{}, err
	}
	if asymmetric {
		// A flank leans from the radial direction, so it stays under a right angle.
		lead, err := flank(f.flankLead, "lead flank", math.Pi/2-1e-9)
		if err != nil {
			return threadplan.Form{}, err
		}
		trail, err := flank(f.flankTrail, "trail flank", math.Pi/2-1e-9)
		if err != nil {
			return threadplan.Form{}, err
		}
		return threadplan.Form{Slope: math.Tan(lead), BackSlope: math.Tan(trail), Depth: depth, Flat: flat}, nil
	}
	angle, err := flank(f.angle, "angle", 2*math.Pi/3)
	if err != nil {
		return threadplan.Form{}, err
	}
	return threadplan.Form{Slope: math.Tan(angle / 2), Depth: depth, Flat: flat}, nil
}

// resolveCylindricalFace resolves sel to exactly one face of b and returns it
// with its cylinder.
func resolveCylindricalFace(b *Body, sel FaceSelector) (*Face, Cylinder, error) {
	faces, err := sel.SelectFaces(b)
	if err != nil {
		return nil, Cylinder{}, err
	}
	if len(faces) != 1 {
		return nil, Cylinder{}, fmt.Errorf(`%w: Thread needs exactly one face, the selector matched %d`, ErrDegenerate, len(faces))
	}
	cyl, ok := faces[0].Surface().(Cylinder)
	if !ok {
		return nil, Cylinder{}, fmt.Errorf(`%w: Thread needs an analytic cylindrical face, this one is %T`,
			ErrUnsupported, faces[0].Surface())
	}
	return faces[0], cyl, nil
}

// threadFrame returns the frame of the axis plane that holds the groove
// profile, the face's length along its axis, whether the face is a rod, and
// the cylinder's radius in millimetres. The frame's origin lies on the axis at
// the face's start, its V axis runs along the axis and its U axis is radial.
func threadFrame(face *Face, cyl Cylinder) (frame r3.Frame, length float64, external bool, radius float64, err error) {
	radius, err = sectionrecord.MagnitudeIn(cyl.Radius, units.Length, units.Millimeter, "the cylinder radius")
	if err != nil {
		return r3.Frame{}, 0, false, 0, err
	}
	axis, ok := cyl.Axis.Normalize()
	if !ok {
		return r3.Frame{}, 0, false, 0, fmt.Errorf(`%w: the cylinder has no axis direction`, ErrDegenerate)
	}
	// A radial direction: the world axis least aligned with the cylinder's,
	// made perpendicular to it.
	seed := r3.Vec{X: 1}
	if math.Abs(axis.Y) < math.Abs(axis.X) && math.Abs(axis.Y) <= math.Abs(axis.Z) {
		seed = r3.Vec{Y: 1}
	} else if math.Abs(axis.Z) < math.Abs(axis.X) && math.Abs(axis.Z) < math.Abs(axis.Y) {
		seed = r3.Vec{Z: 1}
	}
	radial, ok := axis.Cross(seed).Normalize()
	if !ok {
		return r3.Frame{}, 0, false, 0, fmt.Errorf(`%w: the cylinder axis has no perpendicular`, ErrDegenerate)
	}
	normal, err := face.NormalAt(cyl.Origin.Add(radial.Scale(radius)))
	if err != nil {
		return r3.Frame{}, 0, false, 0, err
	}
	external = normal.Value.Dot(radial) > 0
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, e := range face.Edges() {
		for _, v := range [2]*Vertex{e.Start(), e.End()} {
			s := v.Position().Value.Sub(cyl.Origin).Dot(axis)
			lo, hi = math.Min(lo, s), math.Max(hi, s)
		}
	}
	if !(hi > lo) {
		return r3.Frame{}, 0, false, 0, fmt.Errorf(`%w: the cylindrical face has no length along its axis`, ErrUnsupported)
	}
	frame, err = r3.NewFrame(cyl.Origin.Add(axis.Scale(lo)), radial, axis)
	if err != nil {
		return r3.Frame{}, 0, false, 0, fmt.Errorf(`%w: the thread frame is degenerate: %s`, ErrDegenerate, err)
	}
	return frame, hi - lo, external, radius, nil
}

// buildThreadCoil sketches the groove profile of plan in the axis plane frame
// and coils it.
func buildThreadCoil(ctx context.Context, d *Document, frame r3.Frame, plan threadplan.Plan, pitch float64, leftHand, allowReduced bool) (*Body, error) {
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	var points [4]*sketch.Point
	for i, c := range plan.Profile {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 0, V: 1}}
	cfg := coilConfig{leftHand: leftHand, stationsPerTurn: int64(coilshell.StationsPerTurn), wholeLines: true}
	stations := threadplan.StationsPerTurn(plan, pitch, coilshell.StationsPerTurn)
	if allowReduced && stations < coilshell.StationsPerTurn {
		cfg.stationsPerTurn = stations
	}
	return coilBody(ctx, d, s, s.Profiles()[0], axis, units.Millimeters(pitch), units.Scalar(plan.Turns), cfg)
}
