package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// facetedHoleSite is the site of a hole in a planar face of a faceted solid with exact vertices
// (agent-docs/orthogonal-boolean-design.md B20). The face's plane is read exactly from its vertices
// (convexExactPolygon) and its unit normal, rounded once to a float, is the hole's axis: the hole runs along that float
// direction, the way it runs along an analytic plane's frame normal, and the tool starts outside the body, so the mouth
// plane is not the face's. The outward side is the one the face's area centroid certifiably leaves the body on.
func facetedHoleSite(ctx context.Context, b *Body, face *Face) (holeSite, error) {
	if _, ok := orthogonalOperand(b); !ok {
		return holeSite{}, fmt.Errorf(`%w: Hole needs a face of a faceted solid with exact vertices, this body holds a vertex bound`,
			ErrUnsupported)
	}
	if len(face.Loops()) != 1 {
		return holeSite{}, fmt.Errorf(`%w: Hole needs a face with one loop, this one has %d`, ErrUnsupported, len(face.Loops()))
	}
	var points []r3.Vec
	for _, e := range face.Edges() {
		if e.Start() == nil || e.End() == nil {
			return holeSite{}, fmt.Errorf(`%w: the face has an edge with no end`, ErrUnsupported)
		}
		points = append(points, e.Start().Position().Value, e.End().Position().Value)
	}
	poly, ok := convexExactPolygon(points)
	if !ok {
		return holeSite{}, fmt.Errorf(`%w: Hole needs a face that is exactly one convex plane polygon`, ErrUnsupported)
	}
	normal := polygonNormal(poly)
	centre, err := face.Centroid()
	if err != nil {
		return holeSite{}, fmt.Errorf(`%w: the face has no area centroid to find its outward side`, ErrUnsupported)
	}
	reach := units.Millimeters(holeTolerance)
	step := 4 * holeTolerance
	outward := normal
	front, errF := b.LocatePoint(ctx, centre.Value.Add(normal.Scale(step)), reach)
	back, errB := b.LocatePoint(ctx, centre.Value.Sub(normal.Scale(step)), reach)
	switch {
	case errF == nil && errB == nil && front == PointOutside && back == PointInside:
	case errF == nil && errB == nil && front == PointInside && back == PointOutside:
		outward = normal.Scale(-1)
	default:
		return holeSite{}, fmt.Errorf(`%w: the outward side of the face is not certain`, ErrUnsupported)
	}
	u, ok := poly[1].Sub(poly[0]).Sub(outward.Scale(poly[1].Sub(poly[0]).Dot(outward))).Normalize()
	if !ok {
		return holeSite{}, fmt.Errorf(`%w: the face has no in-plane direction`, ErrDegenerate)
	}
	frame, err := r3.NewFrame(poly[0], u, outward.Cross(u))
	if err != nil {
		return holeSite{}, fmt.Errorf(`%w: the hole frame is degenerate: %s`, ErrDegenerate, err)
	}
	return holeSite{frame: frame, outward: outward}, nil
}
