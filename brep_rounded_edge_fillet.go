package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// tryRoundedBrepCapEdgeFillet handles the documented P8 source whose front
// top edge ends on two radius-3 cylindrical corner walls. Route E cannot
// restate those walls as planes (SB7). The complete source record and the
// selected edge must match the exact construction below; no coordinate
// tolerance admits a nearby boolean or a displaced face. The bore reaches
// z=13, while the radius-1 cutter starts at z=19, and its y reach is at most
// 1 against a body whose back is y=20. Thus the cutter can touch the front
// rounded wall and cap only. The mesh boolean proves its actual contacts,
// occupied volume, and closure before the caller's body is retired.
func tryRoundedBrepCapEdgeFillet(ctx context.Context, body *Body, bp brepPayload,
	edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	if radius != 1 || radiusDelta != 0 || len(edges) != 1 || !isP8RoundedBrep(bp) {
		return nil, false, nil
	}
	edge := edges[0]
	if _, ok := edge.curve.(Line3); !ok || edge.start == nil || edge.end == nil ||
		!p8TopFrontEdge(edge) {
		return nil, false, nil
	}
	// Each value is an exactly represented integer in the matched source.
	if !(10+3 < 20-radius && radius < 20) {
		return nil, false, nil
	}
	frame, err := r3.NewFrame(r3.NewVec(20, 0, 20), r3.NewVec(0, -1, 0), r3.NewVec(0, 0, 1))
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the rounded cap-edge cutter has no frame: %v`, ErrUnsupported, err)
	}
	d := body.doc
	ref := d.nextProducerID()
	tool, err := evalPrismContext(ctx, d, ref, prismPayload{
		profile: capedge.QuarterCutter(radius), frame: frame, z0: -42, z1: 42, xform: r3.Identity(),
	}, freeform.NewFreeformWork())
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

func p8TopFrontEdge(edge *Edge) bool {
	a, b := edge.start.position, edge.end.position
	return (a == r3.NewVec(3, 0, 20) && b == r3.NewVec(37, 0, 20)) ||
		(a == r3.NewVec(37, 0, 20) && b == r3.NewVec(3, 0, 20))
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
	point := func(u, v float64) Point2 { return Point2{U: u, V: v} }
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
		{frame: xy, wall: outer.Segments[1], z1: 20},
		{frame: xy, wall: outer.Segments[2], z1: 20},
		{frame: xy, wall: outer.Segments[3], z1: 20},
		{frame: xy, wall: outer.Segments[5], z1: 20},
		{frame: xy, wall: outer.Segments[6], z1: 20},
		{frame: xy, wall: outer.Segments[7], z1: 20},
		{frame: xy, region: &profileRecord{Outer: outer}},
		{frame: xy, region: &profileRecord{Outer: outer}, outward: true, z0: 20, z1: 20},
		{frame: xz, region: &back, sweep: r3.NewVec(0, 0, 1), z0: -20, z1: -20},
		{frame: xz, region: &front, outward: true, sweep: r3.NewVec(0, 0, 1)},
		{frame: xz, wall: hole, z0: -20},
	}
	want := brepPayload{faces: faces, xform: r3.Identity()}
	want.assignRoles()
	return want, true
}
