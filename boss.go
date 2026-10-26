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

// BossSpec describes one round boss standing on a planar face.
//
// Face selects the planar face the boss stands on and must resolve to exactly
// one face. At is a world point in millimetres that lies on that face's plane;
// the boss axis runs through it, out of the body. Diameter and Height are
// positive lengths, Height measured from Face. Bore is optional, and nil makes
// a solid boss. Draft is optional: a positive angle makes the boss narrower
// toward its top by that lean of its wall, a negative one wider, and the zero
// Value a straight wall (agent-docs/boss-design.md §5). Diameter is the boss's
// diameter at Face either way.
type BossSpec struct {
	Face     FaceSelector
	At       r3.Vec
	Diameter units.Value
	Height   units.Value
	Bore     *BossBore
	Draft    units.Value
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
// ErrDegenerate, and a face that is not an analytic plane is ErrUnsupported. A
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
	in, err := bossInput(b, spec)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	toolCtx, capture := privateConstructionContext(ctx)
	pillar, err := buildBossPillar(toolCtx, d, in)
	if err != nil {
		return nil, err
	}
	tools := []*Body{pillar}
	defer func() { removeTools(tools) }()
	steps := []toolStep{{op: meshbool.OpUnion, tool: pillar}}
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
			coil, err := buildThreadCoil(toolCtx, d, tap.frame, tap.plan, tap.pitch, tap.leftHand)
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
	// bore, when set, is the pilot hole's request on the boss's top face, and
	// tap the thread to cut in it.
	bore *holeplan.Input
	tap  *HoleThread
}

func bossInput(b *Body, spec BossSpec) (bossRequest, error) {
	var in bossRequest
	plane, normal, err := resolvePlanarFace(b, spec.Face, "Boss")
	if err != nil {
		return in, err
	}
	in.frame, in.outward = plane.Frame, normal.Value
	if in.diameter, err = sectionrecord.MagnitudeIn(spec.Diameter, units.Length, units.Millimeter, "the boss diameter"); err != nil {
		return in, err
	}
	if in.height, err = sectionrecord.MagnitudeIn(spec.Height, units.Length, units.Millimeter, "the boss height"); err != nil {
		return in, err
	}
	if in.diameter == 0 || in.height == 0 {
		return in, fmt.Errorf(`%w: a boss needs a positive diameter and height`, ErrDegenerate)
	}
	if spec.Draft != (units.Value{}) {
		// A draft is signed, so it is read as an angle rather than as a magnitude.
		if in.draft, err = readDraft(spec.Draft, "the boss draft"); err != nil {
			return in, err
		}
	}
	local := plane.Frame.ToLocal(spec.At)
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
	reach := holeplan.Reach(box.Min, box.Max, plane.Frame.Origin(), in.outward, bound) - holeplan.ThroughMargin
	if !(reach > 0) {
		return in, fmt.Errorf(`%w: the body has no material behind the face`, ErrDegenerate)
	}
	in.embed = min(holeplan.MouthMargin, reach/2)
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
	if depth >= in.height+reach {
		return in, fmt.Errorf(`%w: a pilot %g mm deep would leave the boss's footing through the back of the body`,
			ErrDegenerate, depth)
	}
	top, err := r3.NewFrame(plane.Frame.Origin().Add(in.outward.Scale(in.height)), plane.Frame.U(), plane.Frame.V())
	if err != nil {
		return in, fmt.Errorf(`%w: the face frame is degenerate: %s`, ErrDegenerate, err)
	}
	in.bore = &holeplan.Input{
		Frame:    top,
		Outward:  in.outward,
		At:       top.ToWorldUV(in.u, in.v),
		Diameter: diameter,
		Depth:    depth,
		Reach:    in.height + reach,
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
	centre := s.CreatePoint(in.u, in.v)
	s.Fix(centre)
	// The taper runs from the footing plane, embed below the face, so the circle
	// there is as much wider as the wall leans over that depth: the wall is then
	// the stated diameter at the face.
	s.CreateCircle(centre, in.diameter/2+in.embed*math.Tan(in.draft))
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
