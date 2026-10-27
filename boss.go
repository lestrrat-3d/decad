package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/holeplan"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// BossBore is the blind pilot hole of a [BossSpec], a round hole drilled down
// from the boss's top face. Depth is measured from that top face and is a
// positive length. Head is optional and widens the pilot's mouth with a
// [Counterbore] or [Countersink], as [HoleSpec].Head does. Thread is optional
// and taps the pilot, as [HoleSpec].Thread does, with Diameter as the tap drill
// (agent-docs/boss-design.md §4). Draft is optional and leans the pilot's wall as
// [HoleSpec].Draft does a hole's, narrower toward its bottom for a positive angle;
// a pilot that is tapped takes none (ErrDegenerate).
type BossBore struct {
	Diameter units.Value
	Depth    units.Value
	Head     HoleHead
	Thread   *HoleThread
	Draft    units.Value
}

// BossRibs is the set of triangular gusset ribs that brace a [BossSpec]'s root,
// spaced evenly round the boss's axis (agent-docs/boss-design.md §9). Each rib
// stands Height up the boss's wall and reaches Length out along the face, its
// outer edge sloping from the face up to the wall, and is Thickness thick, centred
// on the plane through the boss's axis. Count is from 1 to 16, Thickness, Length and
// Height are positive lengths with Height at most the boss's Height, and Angle is the
// angle of the first rib from the face frame's U axis about the boss's axis: the zero
// Value is along U. The ribs are not drawn into the boss's pilot hole.
type BossRibs struct {
	Count                     int
	Thickness, Length, Height units.Value
	Angle                     units.Value
}

// BossSpec describes one boss standing on a planar or cylindrical face: round by
// default, or the shape Shape names.
//
// Face selects the face the boss stands on and must resolve to exactly one face.
// At is a world point in millimetres that lies on that face's plane, or on a
// cylindrical face, where the boss stands on the plane tangent there and takes
// neither a RootFillet nor a Draft (agent-docs/boss-design.md §8); the boss axis
// runs through it, out of the body. Diameter and Height are
// positive lengths, Height measured from Face. Bore is optional, and nil makes
// a solid boss. Draft is optional: a positive angle makes the boss narrower
// toward its top by that lean of its wall, a negative one wider, and the zero
// Value a straight wall (agent-docs/boss-design.md §5). Diameter is the boss's
// diameter at Face either way. RootFillet is optional: a positive length rounds
// the concave edge where the boss meets the face with a fillet of that radius,
// and the zero Value leaves the corner sharp (agent-docs/boss-design.md §6).
// Shape is optional and replaces the round section with a RectangularHole,
// SlotHole or PolygonHole section, centred on At, as a Hole's shape is: Diameter
// is then zero, the shape's size is the boss's size at Face, and a Draft leans
// every wall by its angle (agent-docs/boss-design.md §7). A shaped boss takes no
// RootFillet.
type BossSpec struct {
	Face       FaceSelector
	At         r3.Vec
	Diameter   units.Value
	Height     units.Value
	Bore       *BossBore
	Draft      units.Value
	RootFillet units.Value
	Shape      HoleShape
	// Ribs, when not nil, braces the boss's root with gusset ribs (agent-docs/boss-design.md §9).
	Ribs *BossRibs
	// Tilt, when set, leans the boss's axis from the face's normal by this angle, in the
	// direction TiltDirection names (agent-docs/boss-design.md §10). It is an angle above
	// zero and at most 60 degrees, and the zero Value is an upright boss. Height is then
	// measured along the tilted axis from the face. TiltDirection is the angle about the face
	// normal from the face frame's U axis to the direction the boss leans toward, and its
	// zero Value is along U. A tilted boss is round and takes no Bore, Draft, RootFillet or
	// Ribs, and stands on a planar face.
	Tilt, TiltDirection units.Value
}

// Boss raises a round boss on a planar face of b, with an optional blind pilot
// hole, and returns the result, retiring b (agent-docs/boss-design.md).
//
// The boss is a cylinder that stands on Face, runs Height out of it and reaches
// into the body behind the face so that the union crosses the face instead of
// touching it. The pilot hole is cut from the boss's top face by the same
// tools Hole builds, in the same atomic commit.
//
// b MUST be a live solid; a nil context, body or selector is ErrDegenerate. A
// selector that does not resolve to one face is the selector's own error or
// ErrDegenerate, and a face that is not an analytic plane or cylinder, nor a convex planar face of a faceted
// solid with exact vertices, is ErrUnsupported. A
// Diameter or Height that is not positive and finite is ErrDegenerate or
// ErrUnitKind. A bore as wide as the boss, or one deep enough to leave the
// boss's footing through the back of the body, is ErrDegenerate. An At more
// than 1e-6 mm off the face plane is ErrDegenerate. A boolean the evaluator
// cannot decide is the boolean's own error. Every failure leaves b live and the
// document unchanged.
func Boss(ctx context.Context, b *Body, spec BossSpec) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationBoss, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a boss`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: Boss needs a solid body`, ErrNotSolid)
	}
	if spec.Face == nil {
		return nil, errNilSelector
	}
	in, err := bossInput(ctx, b, spec)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if spec.RootFillet == (units.Value{}) {
		return bossCombine(ctx, d, b, in)
	}
	radius, err := sectionrecord.MagnitudeIn(spec.RootFillet, units.Length, units.Millimeter, "the boss root fillet")
	if err != nil {
		return nil, err
	}
	if radius == 0 {
		return nil, fmt.Errorf(`%w: a boss root fillet needs a positive radius`, ErrDegenerate)
	}
	// The boss and its fillet are tried on a copy first, so a fillet the evaluator
	// refuses leaves b live and the document as it was.
	probe, err := Copy(ctx, b)
	if err != nil {
		return nil, err
	}
	defer removeTools([]*Body{probe})
	probed, err := bossCombine(ctx, d, probe, in)
	if err != nil {
		return nil, err
	}
	filleted, err := filletBossRoot(ctx, probed, in, radius)
	removeTools([]*Body{probed, filleted})
	if err != nil {
		return nil, err
	}
	grown, err := bossCombine(ctx, d, b, in)
	if err != nil {
		return nil, err
	}
	return filletBossRoot(ctx, grown, in, radius)
}

// bossCombine unions the boss's pillar into b and cuts its pilot, in one atomic
// commit that retires b.
func bossCombine(ctx context.Context, d *Document, b *Body, in bossRequest) (*Body, error) {
	toolCtx, capture := privateConstructionContext(ctx)
	pillar, err := buildBossPillar(toolCtx, d, in)
	if err != nil {
		return nil, err
	}
	tools := []*Body{pillar}
	defer func() { removeTools(tools) }()
	steps := []toolStep{{op: meshbool.OpUnion, tool: pillar}}
	if len(in.ribs) > 0 {
		ribs, err := buildBossRibs(toolCtx, d, in)
		if err != nil {
			return nil, err
		}
		tools = append(tools, ribs...)
		// One union of the pillar and its ribs: a second union would meet a target that
		// already holds the first one's coarse mesh.
		steps = []toolStep{{op: meshbool.OpUnion, merge: append([]*Body{pillar}, ribs...)}}
	}
	if in.bore != nil {
		plan, err := holeplan.Compute(*in.bore)
		if err != nil {
			return nil, err
		}
		bore, err := buildHoleTools(toolCtx, d, plan)
		if err != nil {
			return nil, err
		}
		tools = append(tools, bore...)
		pilot := bore
		if in.tap != nil {
			tap, err := planHoleThread(in.tap, false, *in.bore, plan)
			if err != nil {
				return nil, err
			}
			coil, err := buildThreadCoil(toolCtx, d, tap.frame, tap.plan, tap.pitch, tap.leftHand, true)
			if err != nil {
				return nil, err
			}
			tools = append(tools, coil)
			pilot = append(append([]*Body(nil), bore...), coil)
		}
		// One cut by the united pilot tools: a second cut of the boss would meet a
		// target that already holds the first cut's coarse mesh.
		steps = append(steps, toolStep{op: meshbool.OpCut, merge: pilot})
	}
	return combineToolSteps(ctx, d, OperationBoss, b, steps, capture)
}

// bossRequest is a boss spec resolved to plain millimetre data.
type bossRequest struct {
	// frame is the face's plane frame and outward its outward unit normal.
	frame   r3.Frame
	outward r3.Vec
	// u and v are the boss axis in frame's coordinates.
	u, v float64
	// diameter and height are the boss's size, embed how far it reaches into
	// the body behind the face, and draft the lean of its wall in radians.
	diameter, height, embed, draft float64
	// shape, when set, is the section in place of a circle of diameter.
	shape *bossShape
	// tilt, when set, leans the boss: its axis, the direction square to the axis in the plane of
	// the lean, and how far along the axis the pillar starts behind the face.
	tilt *bossTilt
	// ribs, when set, are the gusset ribs round the boss's root, in the order of their angles.
	ribs []bossRibRequest
	// curved, when set, says the boss stands on a cylindrical face: frame is then the
	// plane tangent to the cylinder at the boss's centre.
	curved *curvedHole
	// bore, when set, is the pilot hole's request on the boss's top face, and
	// tap the thread to cut in it.
	bore *holeplan.Input
	tap  *HoleThread
}

// resolveBossSite resolves spec.Face to the plane the boss stands on: a planar
// face is its own plane, and a cylindrical face gives the plane tangent to it at
// spec.At, which MUST lie on the cylinder, as a hole's site does.
func resolveBossSite(ctx context.Context, b *Body, spec BossSpec) (holeSite, error) {
	if faces, err := spec.Face.SelectFaces(b); err == nil && len(faces) == 1 {
		if cyl, ok := faces[0].Surface().(Cylinder); ok {
			return cylinderHoleSite(faces[0], cyl, spec.At)
		}
		if _, ok := faces[0].Surface().(Faceted); ok {
			return facetedHoleSite(ctx, b, faces[0])
		}
	}
	plane, normal, err := resolvePlanarFace(b, spec.Face, "Boss")
	if err != nil {
		return holeSite{}, err
	}
	return holeSite{frame: plane.Frame, outward: normal.Value}, nil
}

func bossInput(ctx context.Context, b *Body, spec BossSpec) (bossRequest, error) {
	var in bossRequest
	site, err := resolveBossSite(ctx, b, spec)
	if err != nil {
		return in, err
	}
	in.frame, in.outward, in.curved = site.frame, site.outward, site.cylinder
	if spec.Tilt != (units.Value{}) {
		switch {
		case in.curved != nil:
			return in, fmt.Errorf(`%w: a boss on a cylindrical face stands upright`, ErrUnsupported)
		case spec.Shape != nil || spec.Ribs != nil || spec.Draft != (units.Value{}) || spec.RootFillet != (units.Value{}):
			return in, fmt.Errorf(`%w: a tilted boss is round and takes no ribs, draft or root fillet`, ErrUnsupported)
		}
	} else if spec.TiltDirection != (units.Value{}) {
		return in, fmt.Errorf(`%w: a tilt direction needs a tilt`, ErrDegenerate)
	}
	if spec.Ribs != nil {
		switch {
		case in.curved != nil:
			return in, fmt.Errorf(`%w: a boss on a cylindrical face takes no ribs`, ErrUnsupported)
		case spec.Draft != (units.Value{}) || spec.RootFillet != (units.Value{}):
			return in, fmt.Errorf(`%w: a boss with ribs takes no draft and no root fillet`, ErrUnsupported)
		}
	}
	if in.curved != nil && spec.RootFillet != (units.Value{}) {
		return in, fmt.Errorf(`%w: a boss on a cylindrical face takes no root fillet`, ErrUnsupported)
	}
	if in.curved != nil && spec.Draft != (units.Value{}) {
		return in, fmt.Errorf(`%w: a boss on a cylindrical face takes no draft`, ErrUnsupported)
	}
	if spec.Shape == nil || spec.Diameter != (units.Value{}) {
		if in.diameter, err = sectionrecord.MagnitudeIn(spec.Diameter, units.Length, units.Millimeter, "the boss diameter"); err != nil {
			return in, err
		}
	}
	if in.height, err = sectionrecord.MagnitudeIn(spec.Height, units.Length, units.Millimeter, "the boss height"); err != nil {
		return in, err
	}
	if spec.Shape != nil {
		if in.diameter != 0 {
			return in, fmt.Errorf(`%w: a shaped boss takes its size from the shape, so Diameter must be zero`, ErrDegenerate)
		}
		if spec.RootFillet != (units.Value{}) {
			return in, fmt.Errorf(`%w: a shaped boss takes no root fillet`, ErrUnsupported)
		}
		if in.shape, err = readBossShape(spec.Shape); err != nil {
			return in, err
		}
		in.diameter = in.shape.across()
	}
	if in.diameter == 0 || in.height == 0 {
		return in, fmt.Errorf(`%w: a boss needs a positive diameter and height`, ErrDegenerate)
	}
	if spec.Tilt != (units.Value{}) {
		if in.tilt, err = readBossTilt(spec, in); err != nil {
			return in, err
		}
	}
	if spec.Ribs != nil {
		if in.ribs, err = readBossRibs(spec.Ribs, in.diameter/2, in.height); err != nil {
			return in, err
		}
	}
	if spec.Draft != (units.Value{}) {
		// A draft is signed, so it is read as an angle rather than as a magnitude.
		if in.draft, err = readDraft(spec.Draft, "the boss draft"); err != nil {
			return in, err
		}
	}
	local := in.frame.ToLocal(spec.At)
	if local.Z > holeplan.OnPlaneTol || local.Z < -holeplan.OnPlaneTol {
		return in, fmt.Errorf(`%w: the boss centre lies %g mm off the face plane`, ErrDegenerate, local.Z)
	}
	in.u, in.v = local.X, local.Y
	box, err := b.Bounds()
	if err != nil {
		return in, err
	}
	bound, err := box.Bound.In(units.Millimeter)
	if err != nil {
		return in, fmt.Errorf(`%w: the body bounds are not representable: %s`, ErrNotFinite, err)
	}
	reach := holeplan.Reach(box.Min, box.Max, in.frame.Origin(), in.outward, bound) - holeplan.ThroughMargin
	if !(reach > 0) {
		return in, fmt.Errorf(`%w: the body has no material behind the face`, ErrDegenerate)
	}
	in.embed = min(holeplan.MouthMargin, reach/2)
	if c := in.curved; c != nil {
		// The pillar starts embed behind the plane tangent to the cylinder at the
		// boss's centre. A convex wall falls away from that plane by the sagitta
		// R - sqrt(R² - r²) at distance r, which the footing must reach past to lie
		// inside the body across its whole section; a concave wall rises toward it,
		// into the material the footing already overlaps.
		in.embed = holeplan.MouthMargin
		if !c.concave {
			half := in.diameter / 2
			if half >= c.radius {
				return in, fmt.Errorf(`%w: a boss of %g mm across is as wide as the %g mm radius cylinder it stands on`,
					ErrDegenerate, in.diameter, c.radius)
			}
			in.embed += c.radius - math.Sqrt(c.radius*c.radius-half*half)
		}
		if in.embed >= reach {
			return in, fmt.Errorf(`%w: the body is too thin behind the cylindrical face for the boss's footing`, ErrDegenerate)
		}
	}
	if in.tilt != nil {
		// The pillar's base disc is tilted, so its far edge dips r·sin(a) below the middle of
		// the base and the footing is embed·cos(a) below the face there: both must lie in the body.
		in.tilt.embed = (in.diameter/2*math.Sin(in.tilt.angle) + holeplan.MouthMargin) / math.Cos(in.tilt.angle)
		if low := in.tilt.embed*math.Cos(in.tilt.angle) + in.diameter/2*math.Sin(in.tilt.angle); low >= reach {
			return in, fmt.Errorf(`%w: the body is too thin behind the face for a boss tilted %g degrees`,
				ErrDegenerate, in.tilt.angle*180/math.Pi)
		}
	}
	if spec.Bore == nil {
		return in, nil
	}
	diameter, err := sectionrecord.MagnitudeIn(spec.Bore.Diameter, units.Length, units.Millimeter, "the pilot diameter")
	if err != nil {
		return in, err
	}
	depth, err := sectionrecord.MagnitudeIn(spec.Bore.Depth, units.Length, units.Millimeter, "the pilot depth")
	if err != nil {
		return in, err
	}
	if diameter == 0 || depth == 0 {
		return in, fmt.Errorf(`%w: a pilot hole needs a positive diameter and depth`, ErrDegenerate)
	}
	// The pilot opens on the boss's top face, where a drafted boss is narrower.
	topDiameter := in.diameter - 2*in.height*math.Tan(in.draft)
	if diameter >= min(in.diameter, topDiameter) {
		return in, fmt.Errorf(`%w: the pilot diameter %g mm must be less than the boss diameter %g mm at its top`,
			ErrDegenerate, diameter, min(in.diameter, topDiameter))
	}
	// The pilot runs along the boss's axis from the middle of its top face: the face's normal for an upright boss, and for
	// a tilted one the tilt axis, with the pilot's bottom disc square to it, so its lowest point is half its diameter
	// times sin(tilt) below the middle of the bottom along the face's normal.
	var topFrame r3.Frame
	axis, base, cosTilt, sinTilt := in.outward, in.frame.ToWorldUV(in.u, in.v), 1.0, 0.0
	if in.tilt != nil {
		axis, cosTilt, sinTilt = in.tilt.axis, math.Cos(in.tilt.angle), math.Sin(in.tilt.angle)
	}
	reachAlong := in.height + (reach-diameter/2*sinTilt)/cosTilt
	if depth >= reachAlong {
		return in, fmt.Errorf(`%w: a pilot %g mm deep would leave the boss's footing through the back of the body`,
			ErrDegenerate, depth)
	}
	origin := base.Add(axis.Scale(in.height))
	if in.tilt != nil {
		if topFrame, err = r3.NewFrame(origin, in.tilt.across, axis.Cross(in.tilt.across)); err != nil {
			return in, fmt.Errorf(`%w: the tilted top frame is degenerate: %s`, ErrDegenerate, err)
		}
	} else if topFrame, err = r3.NewFrame(in.frame.Origin().Add(in.outward.Scale(in.height)), in.frame.U(), in.frame.V()); err != nil {
		return in, fmt.Errorf(`%w: the face frame is degenerate: %s`, ErrDegenerate, err)
	}
	at := topFrame.ToWorldUV(in.u, in.v)
	if in.tilt != nil {
		at = origin
	}
	in.bore = &holeplan.Input{
		Frame:    topFrame,
		Outward:  axis,
		At:       at,
		Diameter: diameter,
		Depth:    depth,
		Reach:    reachAlong,
	}
	in.tap = spec.Bore.Thread
	if spec.Bore.Draft != (units.Value{}) {
		if spec.Bore.Thread != nil {
			return in, fmt.Errorf(`%w: a tapped pilot is straight and takes no draft`, ErrDegenerate)
		}
		if in.bore.Draft, err = readDraft(spec.Bore.Draft, "the pilot draft"); err != nil {
			return in, err
		}
	}
	return in, holeHeadInput(in.bore, spec.Bore.Head)
}

// buildBossPillar extrudes the boss's cylinder out of the face, starting embed
// behind it.
func buildBossPillar(ctx context.Context, d *Document, in bossRequest) (*Body, error) {
	if in.tilt != nil {
		return buildTiltedPillar(ctx, d, in)
	}
	footing, err := r3.NewFrame(in.frame.Origin().Sub(in.outward.Scale(in.embed)), in.frame.U(), in.frame.V())
	if err != nil {
		return nil, fmt.Errorf(`%w: the face frame is degenerate: %s`, ErrDegenerate, err)
	}
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(footing)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	// The taper runs from the footing plane, embed below the face, so the section
	// there is as much wider as the wall leans over that depth: the wall is then
	// the stated size at the face.
	grow := in.embed * math.Tan(in.draft)
	if in.shape != nil {
		in.shape.sketchSection(s, in.u, in.v, grow)
	} else {
		centre := s.CreatePoint(in.u, in.v)
		s.Fix(centre)
		s.CreateCircle(centre, in.diameter/2+grow)
	}
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	dir := Against
	if in.outward.Dot(footing.N()) > 0 {
		dir = Along
	}
	var taper []ExtrudeOption
	if in.draft != 0 {
		taper = []ExtrudeOption{WithTaper(units.Radians(in.draft))}
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(in.height + in.embed), Dir: dir}, taper...)
}

// bossRibRequest is one gusset rib in plain data: its angle about the boss's axis from the
// face frame's U axis, in radians, and its thickness, length and height in millimetres.
type bossRibRequest struct {
	angle, thickness, length, height float64
}

// readBossRibs reads r into one request for each rib. radius is the boss's radius at the
// face and height its height.
func readBossRibs(r *BossRibs, radius, height float64) ([]bossRibRequest, error) {
	if r.Count < 1 || r.Count > 16 {
		return nil, fmt.Errorf(`%w: a boss takes from 1 to 16 ribs, not %d`, ErrDegenerate, r.Count)
	}
	thickness, err := sectionrecord.MagnitudeIn(r.Thickness, units.Length, units.Millimeter, "the boss rib thickness")
	if err != nil {
		return nil, err
	}
	length, err := sectionrecord.MagnitudeIn(r.Length, units.Length, units.Millimeter, "the boss rib length")
	if err != nil {
		return nil, err
	}
	rise, err := sectionrecord.MagnitudeIn(r.Height, units.Length, units.Millimeter, "the boss rib height")
	if err != nil {
		return nil, err
	}
	if thickness == 0 || length == 0 || rise == 0 {
		return nil, fmt.Errorf(`%w: a boss rib needs a positive thickness, length and height`, ErrDegenerate)
	}
	if rise > height {
		return nil, fmt.Errorf(`%w: a boss rib %g mm tall is taller than the %g mm boss`, ErrDegenerate, rise, height)
	}
	if thickness >= 2*radius {
		return nil, fmt.Errorf(`%w: a boss rib %g mm thick is as thick as the %g mm boss is wide`, ErrDegenerate, thickness, 2*radius)
	}
	// Neighbouring ribs meet at the boss's wall when half a rib's thickness reaches the
	// half-way line between them.
	if r.Count > 1 && thickness/2 >= radius*math.Sin(math.Pi/float64(r.Count)) {
		return nil, fmt.Errorf(`%w: %d ribs %g mm thick meet at the root of a boss of radius %g mm`, ErrDegenerate, r.Count, thickness, radius)
	}
	start := 0.0
	if r.Angle != (units.Value{}) {
		if start, err = r.Angle.In(units.Radian); err != nil {
			return nil, fmt.Errorf(`%w: the boss rib angle must be an angle: %s`, ErrUnitKind, err)
		}
		if math.IsNaN(start) || math.IsInf(start, 0) {
			return nil, fmt.Errorf(`%w: the boss rib angle is not finite`, ErrNotFinite)
		}
	}
	out := make([]bossRibRequest, r.Count)
	for k := range out {
		out[k] = bossRibRequest{angle: start + 2*math.Pi*float64(k)/float64(r.Count), thickness: thickness, length: length, height: rise}
	}
	return out, nil
}

// buildBossRibs extrudes one gusset rib for each request: a polygon in the plane through
// the boss's axis at the rib's angle, its coordinates the distance along the rib out from
// the axis and the height above the face, extruded half its thickness each way. The polygon
// starts half way to the axis (the boss's smallest radius, for a shaped boss) and half of embed below the face so it overlaps the
// boss and the body without lying in the pillar's footing plane, runs out to the boss's wall along the rib's plane plus the rib's length,
// and slopes from the face at that end up to the rib's height at the wall.
func buildBossRibs(ctx context.Context, d *Document, in bossRequest) ([]*Body, error) {
	radius := in.diameter / 2
	var tools []*Body
	fail := func(err error) ([]*Body, error) {
		removeTools(tools)
		return nil, err
	}
	for _, rib := range in.ribs {
		dir := in.frame.U().Scale(math.Cos(rib.angle)).Add(in.frame.V().Scale(math.Sin(rib.angle)))
		frame, err := r3.NewFrame(in.frame.ToWorldUV(in.u, in.v), dir, in.outward)
		if err != nil {
			return fail(fmt.Errorf(`%w: a boss rib's plane is degenerate: %s`, ErrDegenerate, err))
		}
		w := sketch.NewWorld()
		pl, err := w.CreatePlaneFromFrame(frame)
		if err != nil {
			return fail(err)
		}
		s, err := w.CreateSketch(pl)
		if err != nil {
			return fail(err)
		}
		wall := radius // the distance from the axis to the boss's wall along the rib's plane
		if in.shape != nil {
			wall = in.shape.wallDistance(rib.angle)
		}
		corners := [][2]float64{
			{radius / 2, -in.embed / 2},
			{wall + rib.length, -in.embed / 2},
			{wall + rib.length, 0},
			{wall, rib.height},
			{radius / 2, rib.height},
		}
		points := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			points[i] = s.CreatePoint(c[0], c[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		if _, err := s.Solve(ctx); err != nil {
			return fail(err)
		}
		half := units.Millimeters(rib.thickness / 2)
		tool, err := Extrude(ctx, d, s, s.Profiles()[0], TwoSided{One: DistanceSide{D: half}, Two: DistanceSide{D: half}})
		if err != nil {
			return fail(err)
		}
		tools = append(tools, tool)
	}
	return tools, nil
}

// bossTilt is a tilted boss in plain data: angle is the lean from the face's normal in
// radians, axis the boss's unit axis, across a unit vector square to it in the plane of the
// lean, and embed how far along the axis the pillar starts behind the face's point.
type bossTilt struct {
	angle  float64
	axis   r3.Vec
	across r3.Vec
	embed  float64
}

// maxBossTilt is the largest lean, in radians, a boss may take: 60 degrees.
const maxBossTilt = math.Pi / 3

// readBossTilt reads spec.Tilt and spec.TiltDirection against the face frame of in.
func readBossTilt(spec BossSpec, in bossRequest) (*bossTilt, error) {
	angle, err := spec.Tilt.In(units.Radian)
	if err != nil {
		return nil, fmt.Errorf(`%w: the boss tilt must be an angle: %s`, ErrUnitKind, err)
	}
	switch {
	case math.IsNaN(angle) || math.IsInf(angle, 0):
		return nil, fmt.Errorf(`%w: the boss tilt is not finite`, ErrNotFinite)
	case angle < 0:
		return nil, fmt.Errorf(`%w: the boss tilt is negative`, ErrNegativeMagnitude)
	case angle == 0 || angle > maxBossTilt:
		return nil, fmt.Errorf(`%w: a boss tilt must lie above zero and at most 60 degrees`, ErrDegenerate)
	}
	direction := 0.0
	if spec.TiltDirection != (units.Value{}) {
		if direction, err = spec.TiltDirection.In(units.Radian); err != nil {
			return nil, fmt.Errorf(`%w: the boss tilt direction must be an angle: %s`, ErrUnitKind, err)
		}
		if math.IsNaN(direction) || math.IsInf(direction, 0) {
			return nil, fmt.Errorf(`%w: the boss tilt direction is not finite`, ErrNotFinite)
		}
	}
	lean := in.frame.U().Scale(math.Cos(direction)).Add(in.frame.V().Scale(math.Sin(direction)))
	sin, cos := math.Sincos(angle)
	return &bossTilt{
		angle:  angle,
		axis:   in.outward.Scale(cos).Add(lean.Scale(sin)),
		across: lean.Scale(cos).Sub(in.outward.Scale(sin)),
	}, nil
}

// buildTiltedPillar extrudes the tilted boss's cylinder: a circle in the plane square to its
// axis, centred embed along the axis behind the boss's point on the face, swept along the axis
// by the boss's height plus that embed.
func buildTiltedPillar(ctx context.Context, d *Document, in bossRequest) (*Body, error) {
	t := in.tilt
	centre := in.frame.ToWorldUV(in.u, in.v).Sub(t.axis.Scale(t.embed))
	footing, err := r3.NewFrame(centre, t.across, t.axis.Cross(t.across))
	if err != nil {
		return nil, fmt.Errorf(`%w: the tilted footing frame is degenerate: %s`, ErrDegenerate, err)
	}
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(footing)
	if err != nil {
		return nil, err
	}
	s, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	middle := s.CreatePoint(0, 0)
	s.Fix(middle)
	s.CreateCircle(middle, in.diameter/2)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	dir := Against
	if t.axis.Dot(footing.N()) > 0 {
		dir = Along
	}
	return Extrude(ctx, d, s, s.Profiles()[0], Distance{D: units.Millimeters(in.height + t.embed), Dir: dir})
}
