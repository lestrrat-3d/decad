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
	if len(pp.Verts) == 0 || len(pp.Tris) == 0 || proofbound.IsNonFinite(pp.MeshBound) || pp.MeshBound < 0 ||
		proofbound.IsNonFinite(pp.VolSymDiff) || pp.VolSymDiff < 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no finite occupied-volume certificate", ErrUnsupported)
	}
	mesh, err := b.Tessellate(ctx, units.Millimeters(math.Max(1, pp.MeshBound)), WithVerification(VerifyAll))
	if err != nil {
		return MassProperties{}, err
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() ||
		mesh.n.VolSymDiff != pp.VolSymDiff || mesh.n.Bound != pp.MeshBound {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no matching verified mesh certificate", ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, pp.Verts, pp.Tris, pp.contactAudited); err != nil {
		return MassProperties{}, err
	}
	// Anchor at a held corner before summing tetrahedra.
	return massPropertiesFromReadings(
		massmoment.HeldMeshMassProperties(ctx, b.n.Bounds, pp.Verts[0], pp.Verts, pp.Tris, pp.VolSymDiff, density))
}
