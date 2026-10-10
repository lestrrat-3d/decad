package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pointquery"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	if ctx == nil || b == nil || b.doc == nil {
		return PointUndecided, fmt.Errorf(`%w: point location requires a body and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return PointUndecided, err
	}
	if !proofbound.FiniteVec(p) {
		return PointUndecided, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !b.solid {
		return PointUndecided, ErrNotSolid
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyAll)
	if err != nil {
		return PointUndecided, err
	}
	if !mesh.boundaryOK || !mesh.symDiffOK {
		return PointUndecided, nil
	}
	result, err := pointquery.Locate(ctx, p, mesh.vertices, mesh.triangles, mesh.bound, mesh.volSymDiff)
	return PointLocation(result), err
}

// DistanceToPoint measures the minimum distance in millimetres from p to this
// solid. An interior or boundary point has distance zero. The result encloses
// the true distance using a verified mesh boundary; when membership cannot be
// proved, its lower bound is zero. tol is the positive tessellation chord
// tolerance. Retired bodies remain readable; sheets return ErrNotSolid.
func (b *Body) DistanceToPoint(ctx context.Context, p r3.Vec, tol units.Value) (Measurement, error) {
	if ctx == nil || b == nil || b.doc == nil {
		return Measurement{}, fmt.Errorf(`%w: point distance requires a body and context`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return Measurement{}, err
	}
	if !proofbound.FiniteVec(p) {
		return Measurement{}, fmt.Errorf(`%w: the query point must be finite`, ErrNotFinite)
	}
	if !b.solid {
		return Measurement{}, ErrNotSolid
	}
	mesh, err := tessellateContext(ctx, b, tol, VerifyAll)
	if err != nil {
		return Measurement{}, err
	}
	if !mesh.boundaryOK || proofbound.IsNonFinite(mesh.bound) || mesh.bound < 0 {
		return Measurement{}, fmt.Errorf(`%w: point distance needs a verified finite mesh boundary bound`, ErrUnsupported)
	}
	distanceSquared, err := pointquery.MeshDistanceSquared(ctx, p, mesh.vertices, mesh.triangles)
	if err != nil {
		return Measurement{}, err
	}
	location := pointquery.Undecided
	if mesh.symDiffOK {
		location, err = pointquery.LocateWithDistance(ctx, p, mesh.vertices, mesh.triangles,
			mesh.bound, mesh.volSymDiff, distanceSquared)
		if err != nil {
			return Measurement{}, err
		}
	}
	if location == pointquery.Inside || location == pointquery.OnBoundary {
		return Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}, nil
	}
	distanceLo := proofbound.RatSqrtDown(distanceSquared)
	distanceHi := proofbound.RatSqrtUp(distanceSquared)
	if proofbound.IsNonFinite(distanceLo) || proofbound.IsNonFinite(distanceHi) {
		return Measurement{}, fmt.Errorf(`%w: point distance exceeds the representable range`, ErrUnsupported)
	}
	bound := proofarith.FloatRat(mesh.bound)
	lo := new(big.Rat)
	if location == pointquery.Outside {
		lo.Sub(proofarith.FloatRat(distanceLo), bound)
		if lo.Sign() < 0 {
			lo.SetInt64(0)
		}
	}
	hi := new(big.Rat).Add(proofarith.FloatRat(distanceHi), bound)
	result, ok := ratIntervalMeasurement(lo, hi)
	if !ok {
		return Measurement{}, fmt.Errorf(`%w: point distance has no finite measurement`, ErrUnsupported)
	}
	return result, nil
}
