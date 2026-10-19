package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/capblend"
)

// tessellateCapBlend adapts the node build (capblend.TessellateMesh) to
// public handles and runs the Loft occupied-volume proof, which reads the
// root Body and Mesh.
func tessellateCapBlend(ctx context.Context, b *Body, cbp capBlendPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	mesh := bornMesh(new(Mesh))
	lms, motion, err := capblend.TessellateMesh(ctx, &b.n, cbp, chord, verify >= VerifyAll, &mesh.n)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyAll && cbp.LoftSource != nil {
		if err := proveLoftCapBandVolume(ctx, mesh, b, cbp, lms, motion); err != nil {
			return nil, err
		}
	}
	return mesh, nil
}
