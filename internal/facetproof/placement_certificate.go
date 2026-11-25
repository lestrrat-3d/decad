package facetproof

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// placementCertificateShadow makes a certified placement also run the held
// embedding scan and refuse when the scan finds anything to move. Tests set it;
// nothing else does.
var placementCertificateShadow atomic.Bool

// certifiedCopy reports whether the placed copy next, a rigid copy of the
// source (verts, tris) by the basis, is certified embedded by the certificate
// cache the context carries (meshbool.PlacementCertificate), so the held
// embedding scan can be skipped. maxAllow is the largest per-vertex rounding
// allowance the placement charged, the bound Lemma 1 needs on |W_i − X_i|.
// False leaves the scan to run as it always did. The caller has excluded
// bounded replays and placements whose images come from an exact rotation.
func certifiedCopy(ctx context.Context, exact []proof.Xpt, verts []r3.Vec, tris [][3]int, next PlacedMesh,
	basis r3.Basis, vertexAllow []float64, denotes []bool, axes [3]r3.Vec) (bool, error) {
	if len(tris) < meshbool.CertificateMinFacets {
		return false, nil
	}
	cache := meshbool.PlacementCertificatesFrom(ctx)
	if cache == nil || !slices.Contains(denotes, false) {
		return false, nil
	}
	cert, err := cache.Certificate(ctx, verts, tris)
	if errors.Is(err, meshbool.ErrNoCertificate) {
		return false, nil
	}
	if err != nil {
		return false, err
	}
	maxAllow := 0.0
	for _, a := range vertexAllow {
		maxAllow = max(maxAllow, a)
	}
	ok, err := cert.Check(ctx, basis, maxAllow, next.Verts, next.Tris)
	if err != nil || !ok || !placementCertificateShadow.Load() {
		return ok, err
	}
	scanned := slices.Clone(next.Verts)
	moved, repair, err := keepPlacedEmbeddedWithAxes(ctx, exact, scanned, next.Tris, axes, denotes)
	if err != nil {
		return false, fmt.Errorf("%w: a certified copy failed the held embedding scan: %w", decaderr.ErrUnsupported, err)
	}
	if moved != nil || repair != nil || !slices.Equal(scanned, next.Verts) {
		return false, fmt.Errorf("%w: a certified copy moved in the held embedding scan", decaderr.ErrUnsupported)
	}
	return true, nil
}
