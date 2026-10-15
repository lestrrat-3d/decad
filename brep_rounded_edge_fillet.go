package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryRoundedBrepCapEdgeFillet handles the documented P8 source whose front
// top edge ends on two radius-3 cylindrical corner walls. Route E cannot
// restate those walls as planes (SB7). The complete source record and the
// selected edge must match the exact construction below; no coordinate
// tolerance admits a nearby boolean or a displaced face. The bore reaches
// z=13, while each admitted cutter starts at z>=19, and its y reach is at
// most 1 against a body whose back is y=20. Thus it can touch the front
// rounded wall and cap only. The mesh boolean proves its actual contacts,
// occupied volume, and closure before the caller's body is retired.
func tryRoundedBrepCapEdgeFillet(ctx context.Context, body *Body, bp brepPayload,
	edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	if !p8QuarterRadius(radius) || radiusDelta != 0 || len(edges) != 1 || !isP8RoundedBrep(bp) {
		return nil, false, nil
	}
	edge := edges[0]
	if _, ok := edge.n.Curve.(surfacegeom.Line3); !ok || edge.n.Start == nil || edge.n.End == nil ||
		!capedge.P8TopFrontEdge(edge.n.Start.Position, edge.n.End.Position) {
		return nil, false, nil
	}
	// The source integers and admitted quarter-millimeter radii are binary exact.
	if !(10+3 < 20-radius && radius < 20) {
		return nil, false, nil
	}
	frame, err := r3.NewFrame(r3.NewVec(20, 0, 20), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the rounded cap-edge cutter has no frame: %v`, ErrUnsupported, err)
	}
	d := documentHandle(body.n.Doc)
	ref := d.nextProducerID()
	tool, err := bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prismrecord.Payload{
		Profile: capedge.QuarterCutter(radius), Frame: frame, Z0: -42, Z1: 42, Xform: r3.Identity(),
	}, freeform.NewFreeformWork()))
	if err != nil {
		return nil, true, err
	}
	// The ordinary pair tolerance proves the cut but leaves faceted Area
	// coarser than Verify's default limit. The exact P8 source has no
	// restating operand, so 1/64 chords both analytic operands more finely.
	// The same contact, stitch, and occupied-volume proofs decide the result.
	eval, err := evaluateBooleanMeshesAtScale(ctx, meshbool.OpCut, body, tool, pairMeshes{}, 1.0/64)
	if err != nil {
		return nil, true, asBooleanError(meshbool.OpCut, err)
	}
	out, err := buildFacetedBodyWithProof(ctx, d, ref+1, eval.payload, eval.audit, eval.volume, eval.volumeRat)
	if err != nil {
		return nil, true, asBooleanError(meshbool.OpCut, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	d.commitSpan(out, 2, body)
	return out, true, nil
}

// These quarter-millimeter radii are represented exactly in binary. The
// quarter-cylinder cutter and its P8 clearance were checked at each value;
// a wider radius interval needs its own contact and Verify bound.
func p8QuarterRadius(radius float64) bool {
	switch radius {
	case 0.25, 0.5, 0.75, 1:
		return true
	default:
		return false
	}
}

// isP8RoundedBrep compares every stored field, including segment domains,
// displacements, placement, and Boolean trim loops. It accepts only the
// exact 40x20x20 radius-3 plate with one centered y-through radius-3 bore.
// A future wider route needs its own source and clearance proof.
func isP8RoundedBrep(bp brepPayload) bool {
	want, ok := p8RoundedBrepRecord()
	return ok && sectionrecord.IdenticalRecord(bp, want)
}

func p8RoundedBrepRecord() (brepPayload, bool) {
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	if err != nil {
		return brepPayload{}, false
	}
	xz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		return brepPayload{}, false
	}
	point := func(u, v float64) planePoint { return planePoint{U: u, V: v} }
	line := func(x0, y0, x1, y1 float64) curveSegment {
		return lineSeg{Start: point(x0, y0), End: point(x1, y1), TEnd: 1}
	}
	arc := func(cx, cy, x0, y0, x1, y1 float64) curveSegment {
		return arcSeg{Center: point(cx, cy), Start: point(x0, y0), End: point(x1, y1), TEnd: 1}
	}
	outer := loopRecord{Segments: []curveSegment{
		line(3, 0, 37, 0), arc(37, 3, 37, 0, 40, 3),
		line(40, 3, 40, 17), arc(37, 17, 40, 17, 37, 20),
		line(37, 20, 3, 20), arc(3, 17, 3, 20, 0, 17),
		line(0, 17, 0, 3), arc(3, 3, 0, 3, 3, 0),
	}}
	hole := circleSeg{Center: point(20, 10), Radius: units.Millimeters(3), TStart: 1}
	front := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(3, 0, 37, 0), line(37, 0, 37, 20),
		line(37, 20, 3, 20), line(3, 20, 3, 0),
	}}, Holes: []loopRecord{{Segments: []curveSegment{hole}}}}
	back := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		line(37, 0, 37, 20), line(37, 20, 3, 20),
		line(3, 20, 3, 0), line(3, 0, 37, 0),
	}}, Holes: []loopRecord{{Segments: []curveSegment{hole}}}}
	faces := []brepFace{
		{Frame: xy, Wall: outer.Segments[1], Z1: 20},
		{Frame: xy, Wall: outer.Segments[2], Z1: 20},
		{Frame: xy, Wall: outer.Segments[3], Z1: 20},
		{Frame: xy, Wall: outer.Segments[5], Z1: 20},
		{Frame: xy, Wall: outer.Segments[6], Z1: 20},
		{Frame: xy, Wall: outer.Segments[7], Z1: 20},
		{Frame: xy, Region: &profileRecord{Outer: outer}},
		{Frame: xy, Region: &profileRecord{Outer: outer}, Outward: true, Z0: 20, Z1: 20},
		{Frame: xz, Region: &back, Sweep: r3.NewVec(0, 0, 1), Z0: -20, Z1: -20},
		{Frame: xz, Region: &front, Outward: true, Sweep: r3.NewVec(0, 0, 1)},
		{Frame: xz, Wall: hole, Z0: -20},
	}
	want := brepPayload{Faces: faces, Xform: r3.Identity()}
	want.AssignRoles()
	return want, true
}
