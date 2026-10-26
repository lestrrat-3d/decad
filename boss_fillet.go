package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// bossRootTol is how far, in millimetres, the boss's root circle may lie from
// its stated plane, centre and radius when it is found in the result.
const bossRootTol = 1e-6

// filletBossRoot rounds the concave circle where the boss of in meets its face
// in body, with a fillet of radius millimetres, retiring body.
func filletBossRoot(ctx context.Context, body *Body, in bossRequest, radius float64) (*Body, error) {
	root, err := bossRootLoop(body, in)
	if err != nil {
		return nil, err
	}
	return Fillet(ctx, body, edgesQuery(root.Edges()), units.Millimeters(radius))
}

// edgesQuery selects exactly the given edges of a body by their endpoints and,
// for a straight edge its direction and for a curved one its curvature, so no
// other edge matches. edges is not empty.
func edgesQuery(edges []*Edge) *EdgeQuery {
	var q *EdgeQuery
	for _, e := range edges {
		a, b := e.Start().Position().Value, e.End().Position().Value
		preds := []EdgePredicate{EndpointAt(a), EndpointAt(b)}
		if _, line := e.Curve().(Line3); line {
			preds = append(preds, ParallelTo(b.Sub(a)))
		} else {
			preds = append(preds, Circular())
		}
		if q == nil {
			q = Edges(preds...)
			continue
		}
		q.Or(preds...)
	}
	return q.Exactly(len(edges))
}

// bossRootLoop finds the loop of one circular edge, of the boss's radius about
// its axis, on a planar face of body lying in the plane the boss stands on.
func bossRootLoop(body *Body, in bossRequest) (*Loop, error) {
	centre := in.frame.ToWorldUV(in.u, in.v)
	for _, f := range body.Faces() {
		plane, ok := f.Surface().(Plane)
		if !ok {
			continue
		}
		if math.Abs(plane.Frame.ToLocal(in.frame.Origin()).Z) > bossRootTol {
			continue
		}
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
			if err != nil || math.Abs(r-in.diameter/2) > bossRootTol || circle.Center.Sub(centre).Len() > bossRootTol {
				continue
			}
			return loop, nil
		}
	}
	return nil, fmt.Errorf(`%w: the boss's root circle is not an edge of the result, so it has nothing to fillet`, ErrUnsupported)
}
