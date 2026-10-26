package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PointLocation is a certified point classification against a solid body.
// PointUndecided carries no inside or outside claim.
type PointLocation int

const (
	PointUndecided PointLocation = iota
	PointOutside
	PointInside
	PointOnBoundary
)

// String names the certified point classification.
func (l PointLocation) String() string {
	switch l {
	case PointOutside:
		return "Outside"
	case PointInside:
		return "Inside"
	case PointOnBoundary:
		return "OnBoundary"
	default:
		return "Undecided"
	}
}

// LocatePoint classifies a world-coordinate point in millimetres against b.
// tol is the positive tessellation chord tolerance. An approximate boundary
// can leave points near it undecided; a finer tolerance may decide them.
// Retired bodies remain readable. A sheet or unsound solid is ErrNotSolid.
func (b *Body) LocatePoint(ctx context.Context, p r3.Vec, tol units.Value) (PointLocation, error) {
	if ctx == nil || b == nil || b.n.Doc == nil {
		return PointUndecided, fmt.Errorf(`%w: point location requires a body and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return PointUndecided, err
	}
	if !proofbound.FiniteVec(p) {
		return PointUndecided, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !b.n.Solid {
		return PointUndecided, ErrNotSolid
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyAll)
	if err != nil {
		return PointUndecided, err
	}
	if !mesh.n.BoundaryOK || !mesh.n.SymDiffOK {
		return PointUndecided, nil
	}
	result, err := pointquery.Locate(ctx, p, mesh.n.Vertices, mesh.n.Triangles, mesh.n.Bound, mesh.n.VolSymDiff)
	return PointLocation(result), err
}

// DistanceToPoint measures the minimum distance in millimetres from p to this
// solid. An interior or boundary point has distance zero. The result encloses
// the true distance using a verified mesh boundary; when membership cannot be
// proved, its lower bound is zero. tol is the positive tessellation chord
// tolerance. Retired bodies remain readable; sheets return ErrNotSolid.
func (b *Body) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error) {
	if ctx == nil || b == nil || b.n.Doc == nil {
		return Measurement{}, fmt.Errorf(`%w: point distance requires a body and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return Measurement{}, err
	}
	if !proofbound.FiniteVec(p) {
		return Measurement{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !b.n.Solid {
		return Measurement{}, ErrNotSolid
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyAll)
	if err != nil {
		return Measurement{}, err
	}
	return distanceResult(pointquery.BodyDistance(ctx, p, &mesh.n))
}

// DistanceToPoint measures the minimum distance in millimetres from p to this
// face's trimmed patch. The face may belong to a solid or sheet, including a
// retired body. tol is the positive tessellation chord tolerance. A verified
// two-sided face displacement bounds the returned measurement.
func (f *Face) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error) {
	if ctx == nil || f == nil || f.n.Body == nil || f.n.Body.Doc == nil {
		return Measurement{}, fmt.Errorf(`%w: face distance requires a face and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return Measurement{}, err
	}
	if !proofbound.FiniteVec(p) {
		return Measurement{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	mesh, err := tessellateContext(ctx, bodyHandle(f.n.Body), tol, VerifyBoundary)
	if err != nil {
		return Measurement{}, err
	}
	return distanceResult(pointquery.FaceDistance(ctx, p, &mesh.n, faceNode(f)))
}

// ClosestPoint is the nearest point to a query point that Body.ClosestPoint and
// Face.ClosestPoint read. Point lies within OffSurface of the true boundary (or
// patch), and its distance from the query point is within OffSurface of the
// true minimum distance. Distance is the reading DistanceToPoint gives. A
// query point inside or on a solid is its own closest point, with OffSurface
// zero.
type ClosestPoint struct {
	Point      r3.Vec
	Distance   Measurement
	OffSurface units.Value
}

// ClosestPoint finds the point of this solid nearest p (agent-docs/point-containment-design.md).
// It reads the verified mesh boundary DistanceToPoint measures: the nearest
// mesh point is found in exact rational arithmetic and rounded to floats, and
// the result states how far that point can lie from the true boundary. When
// p is inside or on the solid, p is returned. tol is the positive
// tessellation chord tolerance. Retired bodies remain readable; sheets return
// ErrNotSolid.
func (b *Body) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error) {
	if ctx == nil || b == nil || b.n.Doc == nil {
		return ClosestPoint{}, fmt.Errorf(`%w: closest point requires a body and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return ClosestPoint{}, err
	}
	if !proofbound.FiniteVec(p) {
		return ClosestPoint{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !b.n.Solid {
		return ClosestPoint{}, ErrNotSolid
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyAll)
	if err != nil {
		return ClosestPoint{}, err
	}
	return closestResult(pointquery.BodyClosest(ctx, p, &mesh.n))
}

// ClosestPoint finds the point of this face's trimmed patch nearest p. It
// reads the triangles a verified mesh attributes to the face, as
// Face.DistanceToPoint does, so Point lies within OffSurface of the patch.
// The face may belong to a solid or sheet, including a retired body. tol is
// the positive tessellation chord tolerance.
func (f *Face) ClosestPoint(ctx context.Context, p r3.Vec, tol units.Value) (ClosestPoint, error) {
	if ctx == nil || f == nil || f.n.Body == nil || f.n.Body.Doc == nil {
		return ClosestPoint{}, fmt.Errorf(`%w: closest point requires a face and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return ClosestPoint{}, err
	}
	if !proofbound.FiniteVec(p) {
		return ClosestPoint{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	mesh, err := tessellateContext(ctx, bodyHandle(f.n.Body), tol, VerifyBoundary)
	if err != nil {
		return ClosestPoint{}, err
	}
	return closestResult(pointquery.FaceClosest(ctx, p, &mesh.n, faceNode(f)))
}

// closestResult converts a closest-point reading to its public form.
func closestResult(c pointquery.Closest, err error) (ClosestPoint, error) {
	if err != nil {
		return ClosestPoint{}, err
	}
	return ClosestPoint{Point: c.Point, Distance: c.Distance, OffSurface: units.Millimeters(c.OffSurface)}, nil
}

// distanceResult converts a distance reading to its public measurement.
func distanceResult(m measurement.Measurement, err error) (Measurement, error) {
	if err != nil {
		return Measurement{}, err
	}
	return m, nil
}
