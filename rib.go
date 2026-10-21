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
	opts ...RibOption) (*Body, error) {
	if line == nil {
		return nil, fmt.Errorf(`%w: a nil line names no rib`, ErrDegenerate)
	}
	return Web(ctx, b, s, []*sketch.Line{line}, thickness, e, opts...)
}

// Web adds one rib along each of lines and returns the result, retiring b.
// Every rib has Rib's thickness, extent and overrun, and the ribs may cross
// each other. Web joins them to b in the order of lines and commits once, so a
// failure on any rib leaves the document unchanged and b live. lines MUST NOT
// be empty (ErrDegenerate); every other rule is Rib's.
func Web(ctx context.Context, b *Body, s *sketch.Sketch, lines []*sketch.Line, thickness units.Value, e Extent,
	opts ...RibOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a rib`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: a rib needs a solid body`, ErrNotSolid)
	}
	if s == nil || len(lines) == 0 {
		return nil, fmt.Errorf(`%w: a rib needs a sketch and at least one line`, ErrDegenerate)
	}
	thick, err := sectionrecord.MagnitudeIn(thickness, units.Length, units.Millimeter, "the rib thickness")
	if err != nil {
		return nil, err
	}
	overrun, err := ribOverrun(opts)
	if err != nil {
		return nil, err
	}
	frame, err := s.Plane().Frame()
	if err != nil {
		return nil, fmt.Errorf(`%w: the sketch plane has no frame: %s`, ErrDegenerate, err)
	}
	var footprints [][4][2]float64
	for _, line := range lines {
		if line == nil || !slices.Contains(s.Entities(), sketch.Entity(line)) {
			return nil, fmt.Errorf(`%w: a rib line is nil or is not an entity of the sketch`, ErrDegenerate)
		}
		if line.IsStale() {
			return nil, fmt.Errorf(`%w: a rib line is stale`, ErrDegenerate)
		}
		corners, err := ribplan.Footprint(line.Start.X(), line.Start.Y(), line.End.X(), line.End.Y(), thick, overrun)
		if err != nil {
			return nil, err
		}
		footprints = append(footprints, corners)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var tools []*Body
	defer func() { removeTools(tools) }()
	for _, corners := range footprints {
		tool, err := ribTool(ctx, d, frame, corners, e)
		if err != nil {
			return nil, err
		}
		tools = append(tools, tool)
		if err := requireRibTouches(b, tool); err != nil {
			return nil, err
		}
	}
	return combineTools(ctx, d, b, meshbool.OpUnion, tools, true, func(result *Body) error {
		return requireRibAddsVolume(b, result)
	})
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

// ribTool extrudes one rib outline, a rectangle on the plane of frame.
func ribTool(ctx context.Context, d *Document, frame r3.Frame, corners [4][2]float64, e Extent) (*Body, error) {
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(frame)
	if err != nil {
		return nil, err
	}
	rs, err := w.CreateSketch(pl)
	if err != nil {
		return nil, err
	}
	var points [4]*sketch.Point
	for i, c := range corners {
		points[i] = rs.CreatePoint(c[0], c[1])
		rs.Fix(points[i])
	}
	for i := range points {
		rs.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := rs.Solve(ctx); err != nil {
		return nil, err
	}
	return Extrude(ctx, d, rs, rs.Profiles()[0], e)
}

// requireRibTouches refuses a rib whose bounding box certifiably misses b's.
func requireRibTouches(b, tool *Body) error {
	bb, err := b.Bounds()
	if err != nil {
		return err
	}
	tb, err := tool.Bounds()
	if err != nil {
		return err
	}
	bbBound, err := bb.Bound.In(units.Millimeter)
	if err != nil {
		return fmt.Errorf(`%w: the body bounds are not representable: %s`, ErrNotFinite, err)
	}
	tbBound, err := tb.Bound.In(units.Millimeter)
	if err != nil {
		return fmt.Errorf(`%w: the rib bounds are not representable: %s`, ErrNotFinite, err)
	}
	if ribplan.Disjoint(bb.Min, bb.Max, tb.Min, tb.Max, bbBound+tbBound+ribTolerance) {
		return fmt.Errorf(`%w: the rib lies outside the body, so it would not attach`, ErrDegenerate)
	}
	return nil
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
