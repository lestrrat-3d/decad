package massdispatch

import (
	"context"
	"errors"
	"fmt"
	"math"

	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessaudit"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/units"
)

func Properties(ctx context.Context, mesher verifiedmesh.Mesher, b *topo.Body, density units.Value) (massmoment.MassReadings, error) {
	if b == nil {
		return massmoment.MassReadings{}, fmt.Errorf("%w: nil body", decaderr.ErrDegenerate)
	}
	if density.Kind() != units.Density {
		return massmoment.MassReadings{}, fmt.Errorf("%w: mass density is required", decaderr.ErrUnitKind)
	}
	if proofbound.IsNonFinite(density.Mag()) || proofbound.IsNonFinite(density.Unit().Factor()) {
		return massmoment.MassReadings{}, fmt.Errorf("%w: density is not finite", decaderr.ErrNotFinite)
	}
	if density.Mag() < 0 {
		return massmoment.MassReadings{}, fmt.Errorf("%w: density is negative", decaderr.ErrNegativeMagnitude)
	}
	if density.Mag() == 0 {
		return massmoment.MassReadings{}, fmt.Errorf("%w: density is zero", decaderr.ErrDegenerate)
	}
	if !b.Solid || b.Kind != topo.BodySolid {
		return massmoment.MassReadings{}, decaderr.ErrNotSolid
	}
	if b.Payload == nil {
		return massmoment.MassReadings{}, fmt.Errorf("%w: body has no evaluator payload", decaderr.ErrUnsupported)
	}
	if sphere, ok := contactround.SphereRecord(b); ok {
		if err := ctx.Err(); err != nil {
			return massmoment.MassReadings{}, err
		}
		return massmoment.SourceSphere(ctx, sphere.Radius, b.Centroid, density)
	}
	if cylinder, ok := contactround.RevolvedCylinderAtPose(b, r3.Identity()); ok {
		if err := ctx.Err(); err != nil {
			return massmoment.MassReadings{}, err
		}
		return massmoment.SourceCylinder(ctx, cylinder.Axis, cylinder.Box.Lo, cylinder.Box.Hi,
			b.Centroid, density)
	}
	if revolve, ok := b.Payload.(revolveplan.Payload); ok {
		result, err := massmoment.RevolveProperties(ctx, revolve.MassRecord(), b.Centroid, density)
		if err == nil || !errors.Is(err, decaderr.ErrUnsupported) || ctx.Err() != nil {
			return result, err
		}
		// A term the analytic path does not charge leaves the revolve to its
		// verified mesh (agent-docs/multibody-dynamics-design.md §8.5).
		return verifiedMeshMassProperties(ctx, mesher, b, density)
	}
	if faceted, ok := b.Payload.(facetedrecord.Payload); ok {
		return facetedMassProperties(ctx, mesher, b, faceted, density)
	}
	if sweep, ok := b.Payload.(sweeprecord.Payload); ok {
		result, err := massmoment.SweepProperties(ctx, sweep.MassRecord(), b.Centroid, density)
		return analyticOrMeshMassProperties(ctx, mesher, b, density, result, err)
	}
	if cup, ok := b.Payload.(cupwall.Payload); ok {
		outer, cavity := cup.View().MassRecords()
		result, err := massmoment.CupProperties(ctx, outer, cavity, b.Centroid, density)
		return analyticOrMeshMassProperties(ctx, mesher, b, density, result, err)
	}
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok {
		return verifiedMeshMassProperties(ctx, mesher, b, density)
	}
	if pp.SurfaceResult {
		return massmoment.MassReadings{}, fmt.Errorf("%w: certified volume moments are unavailable for this solid", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return massmoment.MassReadings{}, err
	}
	basis := pp.Xform.Basis()
	if pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!pairbox.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
		!pairbox.CardinalBasis(basis.EX, basis.EY, basis.EZ) {
		return massmoment.GeneralPrismProperties(ctx, pp.MassRecord(), b.Centroid, density)
	}
	if !pairbox.RectangularProfile(pp.Profile) {
		return massmoment.CardinalPrismProperties(ctx, pp.MassRecord(), b.Centroid, density)
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	return massmoment.CardinalBoxProperties(ctx, pp.MassRecord(), b.Centroid, density)
}

// analyticOrMeshMassProperties keeps an analytic arm's result unless that arm
// refused with decaderr.ErrUnsupported, in which case the body takes its verified mesh
// (agent-docs/multibody-dynamics-design.md §8.5).
func analyticOrMeshMassProperties(ctx context.Context, mesher verifiedmesh.Mesher, b *topo.Body, density units.Value, result massmoment.MassReadings, err error) (massmoment.MassReadings, error) {
	if err == nil || !errors.Is(err, decaderr.ErrUnsupported) || ctx.Err() != nil {
		return result, err
	}
	return verifiedMeshMassProperties(ctx, mesher, b, density)
}

// This file owns the mass properties read off an audited triangle set whose
// occupied volume differs from the denoted solid by a certified E
// (agent-docs/dynamic-mass-design.md §2.2, agent-docs/multibody-dynamics-design.md §8.3-§8.5):
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

// verifiedMeshMassProperties is agent-docs/multibody-dynamics-design.md §8.5's
// ladder for a solid with no analytic arm: tessellate at VerifyAll with
// tol_k = diameter·2^-k, diameter the body box's diagonal, and return the
// first reading whose volume and tensor intervals prove positive. A
// tessellation refusal, a mesh that carries no occupied-volume proof, or a
// failed embedding audit ends the ladder at once: a finer mesh of the same
// payload is refused, or carries no proof, the same way.
func verifiedMeshMassProperties(ctx context.Context, mesher verifiedmesh.Mesher, b *topo.Body, density units.Value) (massmoment.MassReadings, error) {
	diameter := b.Bounds.Max.Sub(b.Bounds.Min).Len()
	if proofbound.IsNonFinite(diameter) || diameter <= 0 {
		return massmoment.MassReadings{}, fmt.Errorf("%w: mesh mass has no finite positive diameter", decaderr.ErrUnsupported)
	}
	for k := meshLadderFirst; k <= meshLadderLast; k++ {
		result, err := meshMassPropertiesAt(ctx, mesher, b, math.Ldexp(diameter, -k), density)
		if err == nil {
			return result, nil
		}
		if !errors.Is(err, errMassIntervalUnproved) {
			return massmoment.MassReadings{}, err
		}
	}
	return massmoment.MassReadings{}, fmt.Errorf("%w: tolerance ladder exhausted at diameter·2^-%d", errMassIntervalUnproved, meshLadderLast)
}

// meshMassPropertiesAt is one step of the ladder at chord tolerance tol
// (millimetres). The VerifyAll mesh's closure, orientation and vertex-link
// audits run again on its triangles here, as does the exact facet-crossing
// audit unless the tessellation itself ran its payload's facet-contact audit
// and reported the boundary verified.
func meshMassPropertiesAt(ctx context.Context, mesher verifiedmesh.Mesher, b *topo.Body, tol float64, density units.Value) (massmoment.MassReadings, error) {
	mesh, err := mesher.VerifyAll(ctx, b, tol)
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	if !mesh.BoundaryOK || !mesh.SymDiffOK {
		return massmoment.MassReadings{}, fmt.Errorf("%w: the body's mesh carries no occupied-volume proof", decaderr.ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, mesh.Vertices, mesh.Triangles, tessaudit.AuditsFacetContact(b.Payload)); err != nil {
		return massmoment.MassReadings{}, err
	}
	anchor := b.Bounds.Min.Add(b.Bounds.Max).Scale(.5)
	return massmoment.HeldMeshMassProperties(ctx, b.Bounds, anchor, mesh.Vertices, mesh.Triangles, mesh.VolSymDiff, density)
}

// facetedMassProperties integrates an audited Boolean mesh and widens its
// volume moments by the payload's certified occupied-volume difference
// (mass_properties.go). It reads the payload's own held triangles and
// certificate, and admits them only when the VerifyAll mesh restating them
// publishes the same certificate.
func facetedMassProperties(ctx context.Context, mesher verifiedmesh.Mesher, b *topo.Body, pp facetedrecord.Payload, density units.Value) (massmoment.MassReadings, error) {
	if len(pp.Verts) == 0 || len(pp.Tris) == 0 || proofbound.IsNonFinite(pp.MeshBound) || pp.MeshBound < 0 ||
		proofbound.IsNonFinite(pp.VolSymDiff) || pp.VolSymDiff < 0 {
		return massmoment.MassReadings{}, fmt.Errorf("%w: faceted mass has no finite occupied-volume certificate", decaderr.ErrUnsupported)
	}
	mesh, err := mesher.VerifyAll(ctx, b, math.Max(1, pp.MeshBound))
	if err != nil {
		return massmoment.MassReadings{}, err
	}
	if !mesh.BoundaryOK || !mesh.SymDiffOK ||
		mesh.VolSymDiff != pp.VolSymDiff || mesh.Bound != pp.MeshBound {
		return massmoment.MassReadings{}, fmt.Errorf("%w: faceted mass has no matching verified mesh certificate", decaderr.ErrUnsupported)
	}
	if err := massmoment.AuditMesh(ctx, pp.Verts, pp.Tris, pp.ContactAudited); err != nil {
		return massmoment.MassReadings{}, err
	}
	// Anchor at a held corner before summing tetrahedra.
	return massmoment.HeldMeshMassProperties(ctx, b.Bounds, pp.Verts[0], pp.Verts, pp.Tris, pp.VolSymDiff, density)
}
