package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/threadplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// ThreadSpec describes one ISO metric thread. Face selects the cylindrical face
// to thread and must resolve to exactly one face. Pitch is the axial advance
// per turn, a positive length.
//
// The thread is right-handed unless LeftHand is set. Length is the axial length
// to thread, measured from the end of the face with the least position along
// the axis: the zero Value threads the whole face, and a positive length no
// longer than the face stops the thread there, with the coil's flat end and no
// runout (agent-docs/thread-design.md §5).
type ThreadSpec struct {
	Face     FaceSelector
	Pitch    units.Value
	LeftHand bool
	Length   units.Value
	// Form is the groove's cross-section. The zero Form is the 60 degree ISO
	// basic profile; any other makes a symmetric trapezoidal groove, such as an
	// Acme or trapezoidal thread (agent-docs/thread-design.md §6).
	Form ThreadForm
	// LeadIn, when set, is the distance of a 45 degree chamfer cut on each
	// convex end of the threaded face before the thread, so a screw or a nut
	// starts without a burr: a positive length (agent-docs/thread-design.md §7).
	LeadIn units.Value
	// LeadInAngle, set with LeadIn, is the angle between the lead-in's bevel and
	// the threaded face's end face, in place of 45 degrees: LeadIn stays the
	// bevel's distance along the end face and the bevel runs LeadIn·tan(angle)
	// along the threaded face. It is an angle above zero and below 90 degrees
	// (agent-docs/thread-design.md §10).
	LeadInAngle units.Value
}

// ThreadForm is a symmetric trapezoidal groove for [ThreadSpec.Form]. Angle is
// the included angle between the two flanks, Depth how far the groove reaches
// from the face to its root, and Flat the width of the root. The groove mouth
// opens a twentieth of the pitch past the face, as the ISO groove's does. Set
// all of Angle, Depth and Flat or none: the zero ThreadForm is the ISO profile.
// Angle is an angle above zero and up to 120 degrees, and Depth and Flat are
// positive lengths. FlankLead and FlankTrail take the place of Angle.
type ThreadForm struct {
	Angle, Depth, Flat units.Value
	// FlankLead and FlankTrail, set in place of Angle, give an asymmetric groove,
	// a buttress thread's: each is the lean of a flank from the radial direction,
	// FlankLead of the flank toward the start of the face (its least position
	// along the axis) and FlankTrail of the one toward its far end. Each is an
	// angle above zero and below 90 degrees, and both must be set.
	FlankLead, FlankTrail units.Value
}

// Thread cuts a 60 degree ISO metric thread into a cylindrical
// face of b and returns the threaded body, retiring b (agent-docs/thread-design.md).
//
// A face whose outward normal points away from the axis is a rod: it is the
// thread's major diameter and a groove is cut into it. A face whose outward
// normal points toward the axis is a bore: it is the thread's minor (drilled)
// diameter and the groove is cut out of the wall around it. The groove follows
// the ISO basic profile, with an external root flat of an eighth of the pitch
// and an internal one of a quarter, and runs the whole length of the face, or
// the spec's Length of it. A groove over the whole face runs out past both of
// its ends, so a larger face or a shoulder beside the threaded one is cut as
// well: thread a rod that ends at the thread, or give a Length.
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
func Thread(ctx context.Context, b *Body, spec ThreadSpec) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationThread, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a thread`, ErrDegenerate)
	}
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
	if spec.Face == nil {
		return nil, errNilSelector
	}
	pitch, err := sectionrecord.MagnitudeIn(spec.Pitch, units.Length, units.Millimeter, "the thread pitch")
	if err != nil {
		return nil, err
	}
	face, cyl, err := resolveCylindricalFace(b, spec.Face)
	if err != nil {
		return nil, err
	}
	frame, length, external, radius, err := threadFrame(face, cyl)
	if err != nil {
		return nil, err
	}
	partial := spec.Length != (units.Value{})
	if partial {
		threaded, err := sectionrecord.MagnitudeIn(spec.Length, units.Length, units.Millimeter, "the thread length")
		if err != nil {
			return nil, err
		}
		if threaded > length {
			return nil, fmt.Errorf(`%w: a thread length of %g mm is longer than the %g mm face`, ErrDegenerate, threaded, length)
		}
		length = threaded
	}
	form, err := threadForm(spec.Form)
	if err != nil {
		return nil, err
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
		coil, err := buildThreadCoil(toolCtx, d, frame, plan, pitch, spec.LeftHand, true)
		if err != nil {
			return nil, err
		}
		tools := []*Body{coil}
		defer removeTools(tools)
		return combineTools(ctx, d, OperationThread, target, meshbool.OpCut, tools, false, nil, capture, consumed...)
	}
	if spec.LeadIn == (units.Value{}) {
		if spec.LeadInAngle != (units.Value{}) {
			return nil, fmt.Errorf(`%w: a thread lead-in angle needs a lead-in distance`, ErrDegenerate)
		}
		return cut(b)
	}
	leadIn, err := sectionrecord.MagnitudeIn(spec.LeadIn, units.Length, units.Millimeter, "the thread lead-in")
	if err != nil {
		return nil, err
	}
	if leadIn == 0 {
		return nil, fmt.Errorf(`%w: a thread lead-in needs a positive distance`, ErrDegenerate)
	}
	leadAngle := 0.0
	if spec.LeadInAngle != (units.Value{}) {
		leadAngle, err = sectionrecord.MagnitudeIn(spec.LeadInAngle, units.Angle, units.Radian, "the thread lead-in angle")
		if err != nil {
			return nil, err
		}
		// A bevel along the end face or the threaded face is no chamfer.
		if leadAngle <= 0 || leadAngle >= math.Pi/2 {
			return nil, fmt.Errorf(`%w: a thread lead-in angle must lie above zero and below 90 degrees`, ErrDegenerate)
		}
	}
	ends := threadLeadInEdges(face)
	if len(ends) == 0 {
		return nil, fmt.Errorf(`%w: the threaded face has no convex circular end to chamfer for a lead-in`, ErrDegenerate)
	}
	chamfer := func(target *Body) (*Body, error) {
		if leadAngle == 0 {
			return Chamfer(ctx, target, edgesQuery(ends), units.Millimeters(leadIn))
		}
		return ChamferAngle(ctx, target, edgesQuery(ends), units.Millimeters(leadIn), Faces(Planar()), units.Radians(leadAngle))
	}
	// Build on a copy, then retire b together with the chamfered copy only when
	// the cut succeeds. The frame and plan still come from b's unchamfered face.
	probe, err := Copy(ctx, b)
	if err != nil {
		return nil, err
	}
	defer removeTools([]*Body{probe})
	chamfered, err := chamfer(probe)
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
	asymmetric := f.FlankLead != (units.Value{}) || f.FlankTrail != (units.Value{})
	if asymmetric {
		if f.Angle != (units.Value{}) || f.FlankLead == (units.Value{}) || f.FlankTrail == (units.Value{}) {
			return threadplan.Form{}, fmt.Errorf(`%w: an asymmetric thread form sets both flanks and no Angle`, ErrDegenerate)
		}
	}
	if f.Depth == (units.Value{}) || f.Flat == (units.Value{}) || (!asymmetric && f.Angle == (units.Value{})) {
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
	depth, err := sectionrecord.MagnitudeIn(f.Depth, units.Length, units.Millimeter, "the thread form depth")
	if err != nil {
		return threadplan.Form{}, err
	}
	flat, err := sectionrecord.MagnitudeIn(f.Flat, units.Length, units.Millimeter, "the thread form root flat")
	if err != nil {
		return threadplan.Form{}, err
	}
	if asymmetric {
		// A flank leans from the radial direction, so it stays under a right angle.
		lead, err := flank(f.FlankLead, "lead flank", math.Pi/2-1e-9)
		if err != nil {
			return threadplan.Form{}, err
		}
		trail, err := flank(f.FlankTrail, "trail flank", math.Pi/2-1e-9)
		if err != nil {
			return threadplan.Form{}, err
		}
		return threadplan.Form{Slope: math.Tan(lead), BackSlope: math.Tan(trail), Depth: depth, Flat: flat}, nil
	}
	angle, err := flank(f.Angle, "angle", 2*math.Pi/3)
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
	var opts []CoilOption
	if leftHand {
		opts = append(opts, WithLeftHand())
	}
	stations := threadCoilStationsPerTurn(plan, pitch)
	if allowReduced && stations < coilshell.StationsPerTurn {
		opts = append(opts, stationsPerTurnOption(stations))
	}
	return Coil(ctx, d, s, s.Profiles()[0], axis, units.Millimeters(pitch), units.Scalar(plan.Turns), opts...)
}

// threadCoilStationsPerTurn lowers the held mesh cost for ordinary thread
// grooves when the helix chord estimate stays well inside Verify's tolerance.
// The shell's recorded displacement remains the authoritative bound.
func threadCoilStationsPerTurn(plan threadplan.Plan, pitch float64) int64 {
	const reducedStations = 96
	const priorReducedStations = 128
	const maxEstimatedDeparture = 0.003

	rhoMin, rhoMax := math.Inf(1), math.Inf(-1)
	maxRadialStep := 0.0
	for i, point := range plan.Profile {
		rhoMin = min(rhoMin, point[0])
		rhoMax = max(rhoMax, point[0])
		if i > 0 {
			maxRadialStep = max(maxRadialStep, math.Abs(point[0]-plan.Profile[i-1][0]))
		}
	}
	maxRadialStep = max(maxRadialStep, math.Abs(plan.Profile[0][0]-plan.Profile[len(plan.Profile)-1][0]))
	if !(rhoMin > 0) || maxRadialStep > rhoMin {
		return coilshell.StationsPerTurn
	}
	allowedDeparture := min(maxEstimatedDeparture, 0.0006*rhoMax)
	k := pitch / (2 * math.Pi)
	departure := func(stations int64) float64 {
		h := math.Pi / float64(stations)
		return rhoMax*h*h/2 + maxRadialStep*h/2*(h+k/rhoMin) +
			maxRadialStep*maxRadialStep*h*h/(8*rhoMin)
	}
	if departure(reducedStations) <= allowedDeparture {
		return reducedStations
	}
	if departure(priorReducedStations) <= allowedDeparture {
		return priorReducedStations
	}
	return coilshell.StationsPerTurn
}
