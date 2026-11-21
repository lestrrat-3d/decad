package facetproof

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PlaceRotatedMesh first rounds the affine approximation. If rounding folds its
// facets, try the float boxes of the actual rational rotation. Both attempts
// prove embedding and charge their own displacements from the denoted image.
func PlaceRotatedMesh(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	beta []float64, vol, area float64, d r3.Transform, rotation *exactrotation.Rotation) (PlacedMesh, error) {
	linearGap, translationGap, ok := rotation.Gap(d)
	if !ok {
		return PlacedMesh{}, fmt.Errorf("%w: the exact rotation has no finite gap to the float placement", decaderr.ErrUnsupported)
	}
	placed, err := PlaceMeshDenoting(ctx, budget, verts, tris, beta, vol, area, d, linearGap, translationGap)
	if _, folded := errors.AsType[*meshbool.HeldFoldError](err); !folded {
		return placed, err
	}
	retry, retryErr := placeMeshGap(ctx, budget, verts, tris, beta, vol, area, 0, 0, d, 1, &denotingGap{rotation: rotation})
	if retryErr != nil {
		return PlacedMesh{}, retryErr
	}
	// An embedded, sub-ulp shell can still round inside out. Check the closed
	// shell orientation before a copied proxy or faceted body can consume it.
	if _, auditErr := AuditFacetedMesh(ctx, retry.Verts, retry.Tris); auditErr != nil {
		if errors.Is(auditErr, decaderr.ErrBooleanFailed) {
			return PlacedMesh{}, err
		}
		return PlacedMesh{}, auditErr
	}
	return retry, nil
}
