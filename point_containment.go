package decad

import (
	"context"
	"fmt"

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
