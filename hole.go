package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/holeplan"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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

// HoleSpec describes one hole.
//
// Face selects the planar face the hole starts in and must resolve to exactly
// one face. At is a world point in millimetres that lies on that face's plane;
// the hole axis runs through it, into the body. Diameter is a length. Depth is
// required. Head is optional, and nil makes a plain hole.
type HoleSpec struct {
	Face     FaceSelector
	At       r3.Vec
	Diameter units.Value
	Depth    HoleDepth
	Head     HoleHead
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
func Hole(ctx context.Context, b *Body, spec HoleSpec) (*Body, error) {
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
	tools, err := buildHoleTools(ctx, d, plan)
	if err != nil {
		return nil, err
	}
	defer removeTools(tools)
	return combineTools(ctx, d, b, meshbool.OpCut, tools, false, nil)
}

// holeInput resolves spec into the plain-data request holeplan reads.
func holeInput(b *Body, spec HoleSpec) (holeplan.Input, error) {
	var in holeplan.Input
	plane, normal, err := resolvePlanarFace(b, spec.Face, "Hole")
	if err != nil {
		return in, err
	}
	box, err := b.Bounds()
	if err != nil {
		return in, err
	}
	in.Frame = plane.Frame
	in.Outward = normal.Value
	in.At = spec.At
	if in.Diameter, err = sectionrecord.MagnitudeIn(spec.Diameter, units.Length, units.Millimeter, "the hole diameter"); err != nil {
		return in, err
	}
	bound, err := box.Bound.In(units.Millimeter)
	if err != nil {
		return in, fmt.Errorf(`%w: the body bounds are not representable: %s`, ErrNotFinite, err)
	}
	in.Reach = holeplan.Reach(box.Min, box.Max, plane.Frame.Origin(), normal.Value, bound)
	if err := holeDepthInput(&in, spec.Depth); err != nil {
		return in, err
	}
	return in, holeHeadInput(&in, spec.Head)
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
	bore, err := holeCylinder(ctx, d, plan, plan.Bore)
	if err != nil {
		return fail(err)
	}
	tools = append(tools, bore)
	switch {
	case plan.Counterbore != nil:
		head, err := holeCylinder(ctx, d, plan, *plan.Counterbore)
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

// holeCylinder extrudes one circle on the face plane inward by c.Length.
func holeCylinder(ctx context.Context, d *Document, plan holeplan.Plan, c holeplan.Cylinder) (*Body, error) {
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
	s.CreateCircle(centre, c.Radius)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	dir := Along
	if plan.Against {
		dir = Against
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(c.Tool()), Dir: dir})
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
	var points [4]*sketch.Point
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
