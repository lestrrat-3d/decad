package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// bossRootTol is how far, in millimetres, the boss's root circle may lie from
// its stated plane, centre and radius when it is found in the result.
const bossRootTol = 1e-6

// filletBossRoot rounds the concave root where the boss of in meets its face in
// body, with a fillet of radius millimetres, retiring body. The root is the
// whole circle, or the arcs of it where the face ends (bossRootEdges).
func filletBossRoot(ctx context.Context, body *Body, in bossRequest, radius float64) (*Body, error) {
	roots, err := bossRootEdges(body, in)
	if err != nil {
		return nil, err
	}
	return Fillet(ctx, body, edgesQuery(roots), units.Millimeters(radius))
}

// edgesQuery selects exactly the given edges of a body by their endpoints and,
// for a straight edge its direction and for a curved one its curvature and its
// convexity, so no other edge matches: a root arc and a hole's rim arc can
// share both ends. edges is not empty.
func edgesQuery(edges []*Edge) *EdgeQuery {
	var q *EdgeQuery
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		preds := []EdgePredicate{EndpointAt(a), EndpointAt(b)}
		switch _, line := e.Curve().(Line3); {
		case line:
			preds = append(preds, ParallelTo(b.Sub(a)))
		case e.IsConvex():
			preds = append(preds, Circular(), Convex())
		default:
			preds = append(preds, Circular(), Concave())
		}
		if q == nil {
			q = Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// bossRootEdges finds the edges that make up the root of the boss of in on body.
//
// A boss standing wholly on its face has one root: a planar face in the plane the boss stands on, with a loop of
// exactly one circular edge of the boss's radius about its axis. That edge is returned alone.
//
// A boss whose root circle runs off its face has no such loop. Its root is then every concave arc of the boss's
// circle, about the boss axis and parallel to the plane, on any planar face in that plane. An arc ends where the
// face ends, on a cliff face (boss-root-overflow-design rule CO, C2). Each edge is counted once, though it borders
// two faces. A root arc that sweeps half its circle or more is refused with ErrUnsupported.
func bossRootEdges(body *Body, in bossRequest) ([]*Edge, error) {
	centre := in.frame.ToWorldUV(in.u, in.v)
	radius := in.diameter / 2
	type planeFace struct {
		face   *Face
		normal r3.Vec
	}
	var planes []planeFace
	for _, f := range body.Faces() {
		plane, ok := f.Surface().(Plane)
		if !ok || math.Abs(plane.Frame.ToLocal(in.frame.Origin()).Z) > bossRootTol {
			continue
		}
		planes = append(planes, planeFace{f, plane.Frame.N()})
		for _, loop := range f.Loops() {
			edges := loop.Edges()
			if len(edges) != 1 {
				continue
			}
			circle, ok := edges[0].Curve().(Circle3)
			if !ok {
				continue
			}
			r, err := circle.Radius.In(units.Millimeter)
			if err != nil || math.Abs(r-radius) > bossRootTol || circle.Center.Sub(centre).Len() > bossRootTol {
				continue
			}
			return []*Edge{edges[0]}, nil
		}
	}
	var arcs []*Edge
	var spans []float64
	seen := map[*Edge]bool{}
	for _, pf := range planes {
		for _, e := range pf.face.Edges() {
			if seen[e] {
				continue
			}
			arc, ok := bossRootArc(e, pf.normal, centre, radius)
			if !ok {
				continue
			}
			seen[e] = true
			arcs = append(arcs, e)
			spans = append(spans, arcSpan(e, arc))
		}
	}
	if len(arcs) == 0 {
		return nil, fmt.Errorf(`%w: the boss's root circle is not an edge of the result, so it has nothing to fillet`, ErrUnsupported)
	}
	for _, span := range spans {
		if span >= math.Pi {
			return nil, fmt.Errorf("%w: a root arc of the boss spans half its circle or more", ErrUnsupported)
		}
	}
	return arcs, nil
}

// bossRootArc returns the arc of e when e is a concave arc of the circle of the given radius about centre, whose
// axis is parallel to normal.
func bossRootArc(e *Edge, normal, centre r3.Vec, radius float64) (Arc3, bool) {
	arc, ok := e.Curve().(Arc3)
	if !ok || e.IsConvex() {
		return Arc3{}, false
	}
	r, err := arc.Radius.In(units.Millimeter)
	if err != nil || math.Abs(r-radius) > bossRootTol || arc.Center.Sub(centre).Len() > bossRootTol {
		return Arc3{}, false
	}
	axis, ok := arc.Axis.Normalize()
	if !ok {
		return Arc3{}, false
	}
	n, ok := normal.Normalize()
	return arc, ok && axis.Cross(n).Len() <= 1e-9
}

// arcSpan is the angle an Arc3 edge sweeps counter-clockwise about its axis from its start vertex to its end
// vertex, in [0, 2π). An arc whose two ends coincide sweeps a whole turn.
func arcSpan(e *Edge, arc Arc3) float64 {
	axis, _ := arc.Axis.Normalize()
	u := e.Start().Position().Value.Sub(arc.Center)
	w := e.End().Position().Value.Sub(arc.Center)
	span := math.Atan2(axis.Dot(u.Cross(w)), u.Dot(w))
	if span < 0 {
		span += 2 * math.Pi
	}
	if span < 1e-12 {
		return 2 * math.Pi
	}
	return span
}
