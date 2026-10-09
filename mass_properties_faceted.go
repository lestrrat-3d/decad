package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
)

// facetedMassProperties integrates an audited Boolean mesh and widens its
// volume moments by the payload's certified occupied-volume difference
// (mass_properties_mesh.go). It reads the payload's own held triangles and
// certificate, and admits them only when the VerifyAll mesh restating them
// publishes the same certificate.
func facetedMassProperties(ctx context.Context, b *Body, pp facetedPayload, density units.Value) (MassProperties, error) {
	if len(pp.verts) == 0 || len(pp.tris) == 0 || proofbound.IsNonFinite(pp.meshBound) || pp.meshBound < 0 ||
		proofbound.IsNonFinite(pp.volSymDiff) || pp.volSymDiff < 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no finite occupied-volume certificate", ErrUnsupported)
	}
	mesh, err := b.Tessellate(ctx, units.Millimeters(math.Max(1, pp.meshBound)), WithVerification(VerifyAll))
	if err != nil {
		return MassProperties{}, err
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() ||
		mesh.volSymDiff != pp.volSymDiff || mesh.bound != pp.meshBound {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no matching verified mesh certificate", ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, pp.verts, pp.tris, false); err != nil {
		return MassProperties{}, err
	}
	// Anchor at a held corner before summing tetrahedra.
	return massmoment.HeldMeshMassProperties(ctx, b.bounds, pp.verts[0], pp.verts, pp.tris, pp.volSymDiff, density)
}
