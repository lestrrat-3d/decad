package decad

import (
	"context"
	"fmt"
	"math"

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
	plan, err := threadplan.Compute(radius, pitch, length, external, partial)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	toolCtx, capture := privateConstructionContext(ctx)
	coil, err := buildThreadCoil(toolCtx, d, frame, plan, pitch, spec.LeftHand)
	if err != nil {
		return nil, err
	}
	tools := []*Body{coil}
	defer removeTools(tools)
	return combineTools(ctx, d, OperationThread, b, meshbool.OpCut, tools, false, nil, capture)
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
func buildThreadCoil(ctx context.Context, d *Document, frame r3.Frame, plan threadplan.Plan, pitch float64, leftHand bool) (*Body, error) {
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
	return Coil(ctx, d, s, s.Profiles()[0], axis, units.Millimeters(pitch), units.Scalar(plan.Turns), opts...)
}
