package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// tryObliqueCapEdgeFillet rounds one straight edge of a convex prism cap when
// an oblique neighbour prevents route E's planar-wall restatement. A quarter
// cylinder cutter runs beyond the prism in the edge direction. The boolean
// trims it against the actual neighbouring walls, including their slanted
// terminal curves, and publishes the resulting faceted body with mesh proof.
// A false result leaves the ordinary analytic route in charge.
func tryObliqueCapEdgeFillet(ctx context.Context, body *Body, pp prismPayload, edges []*Edge,
	radius, radiusDelta float64) (*Body, bool, error) {
	if len(edges) != 1 || radiusDelta != 0 || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 {
		return nil, false, nil
	}
	if _, ok := edges[0].curve.(Line3); !ok || edges[0].start == nil || edges[0].end == nil {
		return nil, false, nil
	}
	if len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) < 3 ||
		pp.z1-pp.z0 <= 4*radius || !obliqueCapEdgeProfile(pp, edges[0], radius) {
		return nil, false, nil
	}
	facing := edges[0].Faces()
	if len(facing) != 2 {
		return nil, false, nil
	}
	var sideNormal, capNormal r3.Vec
	mid := edges[0].start.position.Add(edges[0].end.position).Scale(0.5)
	axis := pp.dir(0, 0, 1)
	for _, face := range facing {
		if _, ok := face.Surface().(Plane); !ok {
			return nil, false, nil
		}
		n, err := face.NormalAt(mid)
		if err != nil {
			return nil, true, err
		}
		if math.Abs(n.Value.Dot(axis)) > 1-1e-12 {
			capNormal = n.Value
		} else {
			sideNormal = n.Value
		}
	}
	if capNormal.Len() == 0 || sideNormal.Len() == 0 || math.Abs(sideNormal.Dot(capNormal)) > 1e-12 {
		return nil, false, nil
	}
	frame, err := r3.NewFrame(mid, sideNormal, capNormal)
	if err != nil {
		return nil, true, fmt.Errorf(`%w: the cap-edge cutter has no frame: %v`, ErrUnsupported, err)
	}
	span := proofbound.AbsSumUpper(edges[0].end.position.Sub(edges[0].start.position).Len(), 8*radius)
	if proofbound.IsNonFinite(span) {
		return nil, true, fmt.Errorf(`%w: the cap-edge cutter has no finite sweep span`, ErrNotFinite)
	}
	profile := quarterCornerCutter(radius)
	d := body.doc
	ref := d.nextProducerID()
	tool, err := evalPrismContext(ctx, d, ref, prismPayload{
		profile: profile, frame: frame, z0: -span, z1: span, xform: r3.Identity(),
	}, freeform.NewFreeformWork())
	if err != nil {
		return nil, true, err
	}
	out, err := booleanBody(ctx, meshbool.OpCut, body, tool, ref+1)
	if err != nil {
		return nil, true, err
	}
	if err := ctx.Err(); err != nil {
		return nil, true, err
	}
	d.commitSpan(out, 2, body)
	return out, true, nil
}

// obliqueCapEdgeProfile admits a strictly convex straight section whose
// selected cap edge is its only contact with the nearby supporting line.
// Other corners must stay more than four radii from that line, leaving the
// cutter's quarter-circle cross section clear of unrelated wall features.
func obliqueCapEdgeProfile(pp prismPayload, edge *Edge, radius float64) bool {
	segments := pp.profile.Outer.Segments
	lines := make([]lineSeg, len(segments))
	vertices := make([]Point2, len(segments))
	for i, segment := range segments {
		line, ok := segment.(lineSeg)
		if !ok || !naturalRange(line) {
			return false
		}
		lines[i] = line
		vertices[i] = line.Start
	}
	for i, line := range lines {
		if line.End != vertices[(i+1)%len(vertices)] {
			return false
		}
	}
	var turnSign int
	for i := range vertices {
		a, b, c := vertices[i], vertices[(i+1)%len(vertices)], vertices[(i+2)%len(vertices)]
		abU := new(big.Rat).Sub(proofarith.FloatRat(b.U), proofarith.FloatRat(a.U))
		abV := new(big.Rat).Sub(proofarith.FloatRat(b.V), proofarith.FloatRat(a.V))
		bcU := new(big.Rat).Sub(proofarith.FloatRat(c.U), proofarith.FloatRat(b.U))
		bcV := new(big.Rat).Sub(proofarith.FloatRat(c.V), proofarith.FloatRat(b.V))
		turn := new(big.Rat).Sub(new(big.Rat).Mul(abU, bcV), new(big.Rat).Mul(abV, bcU)).Sign()
		if turn == 0 || (turnSign != 0 && turn != turnSign) {
			return false
		}
		turnSign = turn
	}
	match := -1
	for i, a := range vertices {
		b := vertices[(i+1)%len(vertices)]
		for _, level := range []float64{pp.z0, pp.z1} {
			if !matchEndpoints(edge.start.position, edge.end.position,
				pp.point(a.U, a.V, level), pp.point(b.U, b.V, level), 1e-6) {
				continue
			}
			if match >= 0 {
				return false
			}
			match = i
		}
	}
	if match < 0 {
		return false
	}
	a, b := vertices[match], vertices[(match+1)%len(vertices)]
	du, dv := b.U-a.U, b.V-a.V
	prev := vertices[(match+len(vertices)-1)%len(vertices)]
	next := vertices[(match+2)%len(vertices)]
	if (du == 0) != (dv == 0) &&
		du*(a.U-prev.U)+dv*(a.V-prev.V) == 0 &&
		du*(next.U-b.U)+dv*(next.V-b.V) == 0 {
		return false
	}
	for _, neighbour := range [2]Point2{{U: a.U - prev.U, V: a.V - prev.V},
		{U: next.U - b.U, V: next.V - b.V}} {
		parallel := math.Abs(du*neighbour.U + dv*neighbour.V)
		cross := math.Abs(du*neighbour.V - dv*neighbour.U)
		if cross == 0 || parallel >= 4*cross {
			return false
		}
	}
	length := math.Hypot(b.U-a.U, b.V-a.V)
	if length <= 4*radius {
		return false
	}
	for i, v := range vertices {
		if i == match || i == (match+1)%len(vertices) {
			continue
		}
		distance := math.Abs((b.U-a.U)*(v.V-a.V)-(b.V-a.V)*(v.U-a.U)) / length
		if distance <= 4*radius || proofbound.IsNonFinite(distance) {
			return false
		}
	}
	return true
}

// quarterCornerCutter is the part outside a radius-r quarter disk at the
// intersection of two outward coordinate half-planes. Its straight sides
// extend a radius beyond both original faces so the boolean sees crossings
// instead of overlapping coplanar facets.
func quarterCornerCutter(r float64) profileRecord {
	p0, p1, p2 := Point2{U: -r, V: 0}, Point2{U: -r, V: r}, Point2{U: r, V: r}
	p3, p4, center := Point2{U: r, V: -r}, Point2{U: 0, V: -r}, Point2{U: -r, V: -r}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		arcSeg{Center: center, Start: p4, End: p0, TStart: 1, TEnd: 0},
		lineSeg{Start: p4, End: p3, TStart: 0, TEnd: 1},
		lineSeg{Start: p3, End: p2, TStart: 0, TEnd: 1},
		lineSeg{Start: p2, End: p1, TStart: 0, TEnd: 1},
		lineSeg{Start: p1, End: p0, TStart: 0, TEnd: 1},
	}}}
}
