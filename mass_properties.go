package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MassProperties contains the uniform-density mass, world-space center, and
// centroidal inertia of a solid. Each scalar is accompanied by an absolute
// bound on its numerical error.
type MassProperties struct {
	Mass    Measurement
	Center  VecMeasurement
	Inertia InertiaReading
}

// InertiaReading is the symmetric inertia tensor about the mass center in
// world axes. Mixed entries include the physical minus sign.
type InertiaReading struct {
	XX, YY, ZZ Measurement
	XY, XZ, YZ Measurement
}

// massPropertiesFromReadings assembles the public record from integrator data.
func massPropertiesFromReadings(readings massmoment.MassReadings, err error) (MassProperties, error) {
	if err != nil {
		return MassProperties{}, err
	}
	tensor := readings.Tensor
	return MassProperties{
		Mass: measurementFromInternal(readings.Mass), Center: vecMeasurementFromInternal(readings.Center),
		Inertia: InertiaReading{
			XX: measurementFromInternal(tensor[0]), YY: measurementFromInternal(tensor[1]),
			ZZ: measurementFromInternal(tensor[2]), XY: measurementFromInternal(tensor[3]),
			XZ: measurementFromInternal(tensor[4]), YZ: measurementFromInternal(tensor[5]),
		},
	}, nil
}

// MassProperties computes the properties of b for a stated positive, uniform
// density. The density must have kind units.Density. The current evaluator
// admits untapered solid prisms under any frame and rigid placement, including
// those whose recorded section or levels carry a proven displacement, full
// source spheres under rigid placements, cardinal full source cylinders made
// by revolving an axis-incident rectangle, full and partial revolves of any
// integrated section about an exact in-plane axis under any frame and rigid
// placement, sweeps whose spans those prism and revolve paths admit, cups
// whose outer and cavity prisms they admit, and faceted Boolean solids with a
// certified occupied-volume bound. Every other solid, and a sweep or cup the
// analytic arms refuse, is integrated over its VerifyAll mesh when that mesh
// carries an occupied-volume proof, refining the mesh until the tensor
// interval proves positive: lofts, tapered extrudes, exact stitched solids
// and curved payloads the analytic arms refuse.
// It returns ErrUnsupported for other solids rather than estimating their inertia.
// The receiver and context must not be nil.
func (b *Body) MassProperties(ctx context.Context, density units.Value) (MassProperties, error) {
	if b == nil {
		return MassProperties{}, fmt.Errorf("%w: nil body", ErrDegenerate)
	}
	if density.Kind() != units.Density {
		return MassProperties{}, fmt.Errorf("%w: mass density is required", ErrUnitKind)
	}
	if proofbound.IsNonFinite(density.Mag()) || proofbound.IsNonFinite(density.Unit().Factor()) {
		return MassProperties{}, fmt.Errorf("%w: density is not finite", ErrNotFinite)
	}
	if density.Mag() < 0 {
		return MassProperties{}, fmt.Errorf("%w: density is negative", ErrNegativeMagnitude)
	}
	if density.Mag() == 0 {
		return MassProperties{}, fmt.Errorf("%w: density is zero", ErrDegenerate)
	}
	if !b.n.Solid || b.n.Kind != topo.BodySolid {
		return MassProperties{}, ErrNotSolid
	}
	if b.n.Payload == nil {
		return MassProperties{}, fmt.Errorf("%w: body has no evaluator payload", ErrUnsupported)
	}
	if sphere, ok := sourceSphereRecord(b); ok {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		return massPropertiesFromReadings(massmoment.SourceSphere(ctx, sphere.radius, b.n.Centroid, density))
	}
	if cylinder, ok := sourceRevolvedCylinderAtPose(b, r3.Identity()); ok {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		return massPropertiesFromReadings(massmoment.SourceCylinder(ctx, cylinder.axis, cylinder.box.lo, cylinder.box.hi,
			b.n.Centroid, density))
	}
	if revolve, ok := b.n.Payload.(revolvePayload); ok {
		result, err := massPropertiesFromReadings(
			massmoment.RevolveProperties(ctx, massRevolveRecord(revolve), b.n.Centroid, density))
		if err == nil || !errors.Is(err, ErrUnsupported) || ctx.Err() != nil {
			return result, err
		}
		// A term the analytic path does not charge leaves the revolve to its
		// verified mesh (docs/multibody-dynamics-design.md §8.5).
		return verifiedMeshMassProperties(ctx, b, density)
	}
	if faceted, ok := b.n.Payload.(facetedPayload); ok {
		return facetedMassProperties(ctx, b, faceted, density)
	}
	if sweep, ok := b.n.Payload.(sweepPayload); ok {
		result, err := massPropertiesFromReadings(massmoment.SweepProperties(ctx, massSweepRecord(sweep), b.n.Centroid, density))
		return analyticOrMeshMassProperties(ctx, b, density, result, err)
	}
	if cup, ok := b.n.Payload.(cupPayload); ok {
		outer, cavity := massCupRecords(cup.View())
		result, err := massPropertiesFromReadings(massmoment.CupProperties(ctx, outer, cavity, b.n.Centroid, density))
		return analyticOrMeshMassProperties(ctx, b, density, result, err)
	}
	pp, ok := b.n.Payload.(prismPayload)
	if !ok {
		return verifiedMeshMassProperties(ctx, b, density)
	}
	if pp.SurfaceResult {
		return MassProperties{}, fmt.Errorf("%w: certified volume moments are unavailable for this solid", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	basis := pp.Xform.Basis()
	if pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!pairbox.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
		!pairbox.CardinalBasis(basis.EX, basis.EY, basis.EZ) {
		return massPropertiesFromReadings(massmoment.GeneralPrismProperties(ctx, pp.MassRecord(), b.n.Centroid, density))
	}
	if !pairbox.RectangularProfile(pp.Profile) {
		return massPropertiesFromReadings(massmoment.CardinalPrismProperties(ctx, pp.MassRecord(), b.n.Centroid, density))
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	return massPropertiesFromReadings(massmoment.CardinalBoxProperties(ctx, pp.MassRecord(), b.n.Centroid, density))
}

// massCupRecords reads the two prism records whose difference forms a cup.
func massCupRecords(cp cupView) (massmoment.PrismRecord, massmoment.PrismRecord) {
	outer := cp.OuterPrism()
	outer.Profile = cp.Outer
	cavity := cp.CavityPrism()
	cavity.Profile = cp.Cavity
	return outer.MassRecord(), cavity.MassRecord()
}

// massRevolveRecord presents a root revolve payload to the mass integrator.
func massRevolveRecord(rp revolvePayload) massmoment.RevolveRecord {
	return massmoment.RevolveRecord{
		Profile: rp.Profile, Frame: rp.Frame, Transform: rp.Xform,
		Den: rp.Den, Phi0: rp.Phi0, Phi1: rp.Phi1,
		AU: rp.Ax.AU, AV: rp.Ax.AV, DU: rp.Ax.DU, DV: rp.Ax.DV,
		AUBound: rp.Ax.AUBound, AVBound: rp.Ax.AVBound,
		DUBound: rp.Ax.DUBound, DVBound: rp.Ax.DVBound,
		SectionDelta: rp.SectionDelta, RadialAdmitAllow: rp.Ax.RadialAdmitAllow,
		AxisSnap: rp.Ax.Snap != (regionSnapAllow{}),
	}
}

// massSweepRecord presents the sweep's certified span reductions to the mass integrator.
func massSweepRecord(sp sweepPayload) massmoment.SweepRecord {
	record := massmoment.SweepRecord{Arc: sp.Arc}
	if len(sp.Spans) == 0 {
		if sp.Arc {
			record.Revolve = massRevolveRecord(sp.Revolve)
		} else {
			record.Prism = sp.Prism.MassRecord()
		}
		return record
	}
	record.Spans = make([]massmoment.SweepSpanRecord, len(sp.Spans))
	for i, span := range sp.Spans {
		record.Spans[i] = massmoment.SweepSpanRecord{Arc: span.Arc}
		if span.Arc {
			record.Spans[i].Revolve = massRevolveRecord(span.Revolve)
		} else {
			record.Spans[i].Prism = span.Prism.MassRecord()
		}
	}
	return record
}

// analyticOrMeshMassProperties keeps an analytic arm's result unless that arm
// refused with ErrUnsupported, in which case the body takes its verified mesh
// (docs/multibody-dynamics-design.md §8.5).
func analyticOrMeshMassProperties(ctx context.Context, b *Body, density units.Value, result MassProperties, err error) (MassProperties, error) {
	if err == nil || !errors.Is(err, ErrUnsupported) || ctx.Err() != nil {
		return result, err
	}
	return verifiedMeshMassProperties(ctx, b, density)
}

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
// (mass_properties.go). It reads the payload's own held triangles and
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
