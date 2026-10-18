package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/patchmesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tessellateBodyPatch binds source handles and public refusal names to the
// PatchEdges node builder. The circular source mesh is certified first at
// VerifyBoundary even when the requested result level is VerifyNone.
func tessellateBodyPatch(ctx context.Context, b *Body, pp bodyPatchPayload,
	chord float64, verify Verification,
) (*Mesh, error) {
	var node *topo.Mesh
	var err error
	if prism, ok := patchchain.CircularPrismSource(pp); ok {
		source, sourceErr := tessellatePrismWithCountFloor(ctx, bodyHandle(pp.Faces[0].Body), prism,
			prismWallRole, chord, VerifyBoundary, nil, 0)
		if sourceErr != nil {
			return nil, sourceErr
		}
		node, err = patchmesh.CircularPrism(ctx, &b.n, pp, prism, &source.n, chord, patchmesh.Level(verify))
	} else {
		node, err = patchmesh.Planar(ctx, &b.n, pp, patchmesh.Level(verify),
			func(s surfacegeom.Surface) any { return publicSurface(s) },
			func(c surfacegeom.Curve) any { return publicCurve(c) })
	}
	if err != nil {
		return nil, err
	}
	return bornMesh(&Mesh{n: *node}), nil
}
