package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/ribplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// RibOption configures Rib and Web. WithRibOverrun makes one.
type RibOption interface{ ribOption() }

type ribOverrunOption struct{ overrun units.Value }

func (ribOverrunOption) ribOption() {}

// WithRibOverrun lengthens every rib by overrun beyond both ends of its line.
// A rib whose ends only touch a wall of the body is a coplanar contact that
// the union refuses, so a rib that spans a cavity draws its line wall to wall
// and adds an overrun to bury both ends in the walls. overrun is a
// non-negative length, and the last occurrence wins. The default is zero.
func WithRibOverrun(overrun units.Value) RibOption { return ribOverrunOption{overrun: overrun} }

// ribTolerance is the slack, in millimetres, within which Rib treats a tool
// and the body as touching when it compares their bounding boxes.
const ribTolerance = 1e-9

// Rib adds a rib to b along line and returns the result, retiring b.
//
// line is a straight line of s. The rib is that line thickened across its
// width by thickness, centred on the line, and extruded by e from the plane of
// s, then joined to b with Union. e is any Extent that Extrude accepts,
// resolved against the plane of s exactly as it would be for an Extrude of the
// rib's outline. The rib has the exactness Union gives for the same two
// bodies.
//
// b MUST be a live solid. A nil context, sketch, line or body, a line that is
// not an entity of s, is stale or has no length, a non-positive thickness and
// a negative overrun are ErrDegenerate. A rib whose bounding box is disjoint
// from b's, and a rib that Union certifies adds no volume to b, are
// ErrDegenerate. A union the evaluator cannot decide is the union's own error.
// Every failure leaves the document unchanged and b live.
func Rib(ctx context.Context, b *Body, s *sketch.Sketch, line *sketch.Line, thickness units.Value, e Extent,
	opts ...RibOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationRib, retErr) }()
	if line == nil {
		return nil, fmt.Errorf(`%w: a nil line names no rib`, ErrDegenerate)
	}
	return web(ctx, OperationRib, b, s, []*sketch.Line{line}, thickness, e, opts...)
}

// Web adds one rib along each of lines and returns the result, retiring b.
// Every rib has Rib's thickness, extent and overrun, and the ribs may cross
// each other. Web joins them to b in the order of lines and commits once, so a
// failure on any rib leaves the document unchanged and b live. lines MUST NOT
// be empty (ErrDegenerate); every other rule is Rib's.
func Web(ctx context.Context, b *Body, s *sketch.Sketch, lines []*sketch.Line, thickness units.Value, e Extent,
	opts ...RibOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationWeb, retErr) }()
	return web(ctx, OperationWeb, b, s, lines, thickness, e, opts...)
}

// ribSetup is what Rib, Web and RibArc read from their common arguments.
type ribSetup struct {
	d              *Document
	thick, overrun float64
	frame          r3.Frame
}

// newRibSetup runs the gates the rib calls share: a context, a live solid body,
// a sketch, a thickness and an overrun.
func newRibSetup(ctx context.Context, b *Body, s *sketch.Sketch, thickness units.Value, opts []RibOption) (ribSetup, error) {
	var rs ribSetup
	if ctx == nil {
		return rs, fmt.Errorf(`%w: a nil context cannot control a rib`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return rs, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	rs.d = documentHandle(b.n.Doc)
	if err := rs.d.requireLive(b); err != nil {
		return rs, err
	}
	if b.Kind() != BodySolid {
		return rs, fmt.Errorf(`%w: a rib needs a solid body`, ErrNotSolid)
	}
	if s == nil {
		return rs, fmt.Errorf(`%w: a rib needs a sketch`, ErrDegenerate)
	}
	var err error
	if rs.thick, err = sectionrecord.MagnitudeIn(thickness, units.Length, units.Millimeter, "the rib thickness"); err != nil {
		return rs, err
	}
	if rs.overrun, err = ribOverrun(opts); err != nil {
		return rs, err
	}
	if rs.frame, err = s.Plane().Frame(); err != nil {
		return rs, fmt.Errorf(`%w: the sketch plane has no frame: %s`, ErrDegenerate, err)
	}
	return rs, nil
}

func web(ctx context.Context, kind OperationKind, b *Body, s *sketch.Sketch, lines []*sketch.Line,
	thickness units.Value, e Extent, opts ...RibOption) (*Body, error) {
	entities := make([]sketch.Entity, len(lines))
	for i, line := range lines {
		if line == nil {
			return nil, fmt.Errorf(`%w: a nil line names no rib`, ErrDegenerate)
		}
		entities[i] = line
	}
	return webEntities(ctx, kind, b, s, entities, thickness, e, opts...)
}

// WebPath adds one rib along each of path, a run of lines and arcs of s in the
// order given, and returns the result, retiring b (agent-docs/rib-design.md §6).
// A *sketch.Line is a rib as Rib builds it and a *sketch.Arc one as RibArc builds
// it; the ribs have one thickness, extent and overrun and may cross or meet.
// They are joined to b in order and committed once, so a failure on any rib
// leaves the document unchanged and b live.
//
// Ribs that meet end to end, as the legs of a polyline or an arc and the lines
// either side of it do, need an overrun to overlap at the joint: with the
// default of zero, two ribs that only touch end faces meet in coplanar contact,
// which Union refuses. path MUST NOT be empty and must hold only lines and arcs,
// each otherwise ErrDegenerate; every other rule is Rib's and RibArc's.
func WebPath(ctx context.Context, b *Body, s *sketch.Sketch, path []sketch.Entity, thickness units.Value, e Extent,
	opts ...RibOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationWeb, retErr) }()
	return webEntities(ctx, OperationWeb, b, s, path, thickness, e, opts...)
}

// webEntities builds the outline of each entity, a line or an arc of s, and
// joins the ribs to b.
func webEntities(ctx context.Context, kind OperationKind, b *Body, s *sketch.Sketch, entities []sketch.Entity,
	thickness units.Value, e Extent, opts ...RibOption) (*Body, error) {
	rs, err := newRibSetup(ctx, b, s, thickness, opts)
	if err != nil {
		return nil, err
	}
	if len(entities) == 0 {
		return nil, fmt.Errorf(`%w: a rib needs at least one line or arc`, ErrDegenerate)
	}
	var outlines []ribOutline
	for _, entity := range entities {
		if entity == nil || !slices.Contains(s.Entities(), entity) {
			return nil, fmt.Errorf(`%w: a rib entity is nil or is not an entity of the sketch`, ErrDegenerate)
		}
		switch v := entity.(type) {
		case *sketch.Line:
			if v.IsStale() {
				return nil, fmt.Errorf(`%w: a rib line is stale`, ErrDegenerate)
			}
			corners, err := ribplan.Footprint(v.Start.X(), v.Start.Y(), v.End.X(), v.End.Y(), rs.thick, rs.overrun)
			if err != nil {
				return nil, err
			}
			outlines = append(outlines, ribOutline{corners: &corners})
		case *sketch.Arc:
			if v.IsStale() {
				return nil, fmt.Errorf(`%w: a rib arc is stale`, ErrDegenerate)
			}
			band, err := ribplan.ArcBand(v.Center.X(), v.Center.Y(), v.Start.X(), v.Start.Y(),
				v.End.X(), v.End.Y(), rs.thick, rs.overrun)
			if err != nil {
				return nil, err
			}
			outlines = append(outlines, ribOutline{band: &band})
		default:
			return nil, fmt.Errorf(`%w: a rib follows a line or an arc, not a %T`, ErrDegenerate, entity)
		}
	}
	return joinRibs(ctx, kind, rs, b, s, outlines, e)
}

// RibArc adds a rib to b along arc and returns the result, retiring b.
//
// arc is a circular arc of s, which sweeps counter-clockwise from its Start to
// its End. The rib is the arc thickened across its width by thickness, centred
// on the arc, extruded by e from the plane of s and joined to b with Union, as
// Rib does for a line (agent-docs/rib-design.md §5). WithRibOverrun lengthens
// the rib by that arc length beyond both ends.
//
// The arc's end points MUST lie on one circle about its centre. A nil context,
// sketch, arc or body, an arc that is not an entity of s or is stale, a
// non-positive thickness, a negative overrun, a rib so thick that its inner edge
// reaches the arc's centre and an overrun that would wrap the rib a full turn
// are ErrDegenerate. Every other rule is Rib's.
func RibArc(ctx context.Context, b *Body, s *sketch.Sketch, arc *sketch.Arc, thickness units.Value, e Extent,
	opts ...RibOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationRib, retErr) }()
	rs, err := newRibSetup(ctx, b, s, thickness, opts)
	if err != nil {
		return nil, err
	}
	if arc == nil || !slices.Contains(s.Entities(), sketch.Entity(arc)) {
		return nil, fmt.Errorf(`%w: a rib arc is nil or is not an entity of the sketch`, ErrDegenerate)
	}
	if arc.IsStale() {
		return nil, fmt.Errorf(`%w: a rib arc is stale`, ErrDegenerate)
	}
	band, err := ribplan.ArcBand(arc.Center.X(), arc.Center.Y(), arc.Start.X(), arc.Start.Y(),
		arc.End.X(), arc.End.Y(), rs.thick, rs.overrun)
	if err != nil {
		return nil, err
	}
	return joinRibs(ctx, OperationRib, rs, b, s, []ribOutline{{band: &band}}, e)
}

// ribOutline is the footprint of one rib on the sketch plane: the four corners
// of a thickened line, or the band of a thickened arc.
type ribOutline struct {
	corners *[4][2]float64
	band    *ribplan.Band
}

// joinRibs builds a tool for each outline and joins the tools to b.
func joinRibs(ctx context.Context, kind OperationKind, rs ribSetup, b *Body, s *sketch.Sketch,
	outlines []ribOutline, e Extent) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := rs.d.requireSameContainer([]*Body{b}, s); err != nil {
		return nil, err
	}
	var tools []*Body
	defer func() { removeTools(tools) }()
	toolCtx, history := privateConstructionContext(ctx)
	for _, outline := range outlines {
		tool, err := ribTool(toolCtx, rs.d, rs.frame, outline, e)
		if err != nil {
			return nil, err
		}
		tools = append(tools, tool)
		if err := requireRibTouches(b, tool); err != nil {
			return nil, err
		}
	}
	return combineTools(ctx, rs.d, kind, b, meshbool.OpUnion, tools, true, func(result *Body) error {
		return requireRibAddsVolume(b, result)
	}, history, history.snapshot()...)
}

// ribOverrun reads the last WithRibOverrun in opts, in millimetres.
func ribOverrun(opts []RibOption) (float64, error) {
	var overrun units.Value
	for _, raw := range opts {
		o, ok := raw.(ribOverrunOption)
		if !ok {
			return 0, fmt.Errorf(`%w: the rib option is not a decad rib option (%T)`, ErrDegenerate, raw)
		}
		overrun = o.overrun
	}
	if overrun == (units.Value{}) {
		return 0, nil
	}
	return sectionrecord.MagnitudeIn(overrun, units.Length, units.Millimeter, "the rib overrun")
}

// ribTool extrudes one rib outline on the plane of frame.
func ribTool(ctx context.Context, d *Document, frame r3.Frame, outline ribOutline, e Extent) (*Body, error) {
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, err
	}
	rs, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	fixed := func(u, v float64) *sketch.Point {
		p := rs.CreatePoint(u, v)
		rs.Fix(p)
		return p
	}
	switch {
	case outline.corners != nil:
		var points [4]*sketch.Point
		for i, c := range outline.corners {
			points[i] = fixed(c[0], c[1])
		}
		for i := range points {
			rs.CreateLine(points[i], points[(i+1)%len(points)])
		}
	default:
		band := outline.band
		centre := fixed(band.CentreU, band.CentreV)
		u0, v0 := band.Point(band.Inner, band.Start)
		u1, v1 := band.Point(band.Inner, band.End)
		u2, v2 := band.Point(band.Outer, band.End)
		u3, v3 := band.Point(band.Outer, band.Start)
		innerStart, innerEnd, outerEnd, outerStart := fixed(u0, v0), fixed(u1, v1), fixed(u2, v2), fixed(u3, v3)
		rs.CreateArc(centre, innerStart, innerEnd)
		rs.CreateLine(innerEnd, outerEnd)
		rs.CreateArc(centre, outerStart, outerEnd)
		rs.CreateLine(outerStart, innerStart)
	}
	if _, err := rs.Solve(ctx); err != nil {
		return nil, err
	}
	return Extrude(ctx, d, rs, rs.Profiles()[0], e)
}

// requireRibTouches refuses a rib whose bounding box certifiably misses b's.
func requireRibTouches(b, tool *Body) error {
	disjoint, err := boundsDisjoint(b, tool)
	if err != nil {
		return err
	}
	if disjoint {
		return fmt.Errorf(`%w: the rib lies outside the body, so it would not attach`, ErrDegenerate)
	}
	return nil
}

// boundsDisjoint reports whether the bounding boxes of a and b certifiably
// share no point. Each box is grown by its own bound, so a true answer is
// certain and a false answer proves nothing.
func boundsDisjoint(a, b *Body) (bool, error) {
	ab, err := a.Bounds()
	if err != nil {
		return false, err
	}
	bb, err := b.Bounds()
	if err != nil {
		return false, err
	}
	aBound, err := ab.Bound.In(units.Millimeter)
	if err != nil {
		return false, fmt.Errorf(`%w: the bounds are not representable: %s`, ErrNotFinite, err)
	}
	bBound, err := bb.Bound.In(units.Millimeter)
	if err != nil {
		return false, fmt.Errorf(`%w: the bounds are not representable: %s`, ErrNotFinite, err)
	}
	return ribplan.Disjoint(ab.Min, ab.Max, bb.Min, bb.Max, aBound+bBound+ribTolerance), nil
}

// requireRibAddsVolume refuses a union whose volume certifiably does not
// exceed b's: the ribs lie inside the material already.
func requireRibAddsVolume(before, after *Body) error {
	vb, err := before.Volume()
	if err != nil {
		return err
	}
	va, err := after.Volume()
	if err != nil {
		return err
	}
	if ribplan.AddsNothing(vb.Value.Base(), vb.Bound.Base(), va.Value.Base(), va.Bound.Base()) {
		return fmt.Errorf(`%w: the rib lies inside the body and adds no material`, ErrDegenerate)
	}
	return nil
}
