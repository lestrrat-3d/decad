package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// This file owns the mass properties read off an audited triangle set whose
// occupied volume differs from the denoted solid by a certified E
// (docs/dynamic-mass-design.md §2.2, docs/multibody-dynamics-design.md §8.3-§8.5):
//
//   - internal/massmoment integrates exact signed tetrahedra and widens their
//     moments by E, R_i·E and R_i·R_j·E;
//   - internal/massmoment rounds those intervals into public readings;
//   - verifiedMeshMassProperties, the VerifyAll tolerance ladder. It is the
//     path every solid without an analytic arm takes: a loft or an exact
//     stitched solid restates its own held triangles at any tolerance, so it
//     settles at the first step, and a curved payload refines until its tensor
//     interval proves positive or its own tessellation refuses the next step.
//     A revolve mesh reaches k = 10 at most: its facet-contact audit refuses a
//     finer mesh at the fixed facet-pair ceiling.

// meshLadderFirst and meshLadderLast are §8.5's ladder: tol_k = diameter·2^-k
// for k = meshLadderFirst … meshLadderLast.
const (
	meshLadderFirst = 8
	meshLadderLast  = 14
)

// errMassIntervalUnproved marks a held-mesh reading whose volume or tensor
// interval does not prove positive. A finer mesh may prove it, so the ladder
// continues on it and on nothing else.
var errMassIntervalUnproved = massmoment.ErrMeshIntervalUnproved

// verifiedMeshMassProperties is docs/multibody-dynamics-design.md §8.5's
// ladder for a solid with no analytic arm: tessellate at VerifyAll with
// tol_k = diameter·2^-k, diameter the body box's diagonal, and return the
// first reading whose volume and tensor intervals prove positive. A
// tessellation refusal, a mesh that carries no occupied-volume proof, or a
// failed embedding audit ends the ladder at once: a finer mesh of the same
// payload is refused, or carries no proof, the same way.
func verifiedMeshMassProperties(ctx context.Context, b *Body, density units.Value) (MassProperties, error) {
	diameter := b.n.Bounds.Max.Sub(b.n.Bounds.Min).Len()
	if proofbound.IsNonFinite(diameter) || diameter <= 0 {
		return MassProperties{}, fmt.Errorf("%w: mesh mass has no finite positive diameter", ErrUnsupported)
	}
	for k := meshLadderFirst; k <= meshLadderLast; k++ {
		result, err := meshMassPropertiesAt(ctx, b, math.Ldexp(diameter, -k), density)
		if err == nil {
			return result, nil
		}
		if !errors.Is(err, errMassIntervalUnproved) {
			return MassProperties{}, err
		}
	}
	return MassProperties{}, fmt.Errorf("%w: tolerance ladder exhausted at diameter·2^-%d", errMassIntervalUnproved, meshLadderLast)
}

// meshMassPropertiesAt is one step of the ladder at chord tolerance tol
// (millimetres). The VerifyAll mesh's closure, orientation and vertex-link
// audits run again on its triangles here, as does the exact facet-crossing
// audit unless the tessellation itself ran its payload's facet-contact audit
// and reported the boundary verified.
func meshMassPropertiesAt(ctx context.Context, b *Body, tol float64, density units.Value) (MassProperties, error) {
	mesh, err := b.Tessellate(ctx, units.Millimeters(tol), WithVerification(VerifyAll))
	if err != nil {
		return MassProperties{}, err
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
		return MassProperties{}, fmt.Errorf("%w: the body's mesh carries no occupied-volume proof", ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, mesh.n.Vertices, mesh.n.Triangles, payloadAuditsFacetContact(b.n.Payload)); err != nil {
		return MassProperties{}, err
	}
	anchor := b.n.Bounds.Min.Add(b.n.Bounds.Max).Scale(.5)
	return massPropertiesFromReadings(
		massmoment.HeldMeshMassProperties(ctx, b.n.Bounds, anchor, mesh.n.Vertices, mesh.n.Triangles, mesh.n.VolSymDiff, density))
}

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
	if err := massmoment.AuditMesh(ctx, pp.Verts, pp.Tris, pp.ContactAudited); err != nil {
		return MassProperties{}, err
	}
	// Anchor at a held corner before summing tetrahedra.
	return massPropertiesFromReadings(
		massmoment.HeldMeshMassProperties(ctx, b.n.Bounds, pp.Verts[0], pp.Verts, pp.Tris, pp.VolSymDiff, density))
}
