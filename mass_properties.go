package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"

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
	if !b.solid || b.kind != BodySolid {
		return MassProperties{}, ErrNotSolid
	}
	if b.payload == nil {
		return MassProperties{}, fmt.Errorf("%w: body has no evaluator payload", ErrUnsupported)
	}
	if sphere, ok := sourceSphereRecord(b); ok {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		return massPropertiesFromReadings(massmoment.SourceSphere(ctx, sphere.radius, vecMeasurementToInternal(b.centroid), density))
	}
	if cylinder, ok := sourceRevolvedCylinderAtPose(b, r3.Identity()); ok {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		return massPropertiesFromReadings(massmoment.SourceCylinder(ctx, cylinder.axis, cylinder.box.lo, cylinder.box.hi,
			vecMeasurementToInternal(b.centroid), density))
	}
	if revolve, ok := b.payload.(revolvePayload); ok {
		result, err := massPropertiesFromReadings(
			massmoment.RevolveProperties(ctx, massRevolveRecord(revolve), vecMeasurementToInternal(b.centroid), density))
		if err == nil || !errors.Is(err, ErrUnsupported) || ctx.Err() != nil {
			return result, err
		}
		// A term the analytic path does not charge leaves the revolve to its
		// verified mesh (docs/multibody-dynamics-design.md §8.5).
		return verifiedMeshMassProperties(ctx, b, density)
	}
	if faceted, ok := b.payload.(facetedPayload); ok {
		return facetedMassProperties(ctx, b, faceted, density)
	}
	if sweep, ok := b.payload.(sweepPayload); ok {
		result, err := massPropertiesFromReadings(massmoment.SweepProperties(ctx, massSweepRecord(sweep), vecMeasurementToInternal(b.centroid), density))
		return analyticOrMeshMassProperties(ctx, b, density, result, err)
	}
	if cup, ok := b.payload.(cupPayload); ok {
		outer, cavity := massCupRecords(cup.view())
		result, err := massPropertiesFromReadings(massmoment.CupProperties(ctx, outer, cavity, vecMeasurementToInternal(b.centroid), density))
		return analyticOrMeshMassProperties(ctx, b, density, result, err)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return verifiedMeshMassProperties(ctx, b, density)
	}
	if pp.surfaceResult {
		return MassProperties{}, fmt.Errorf("%w: certified volume moments are unavailable for this solid", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	basis := pp.xform.Basis()
	if pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!pairbox.CardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!pairbox.CardinalBasis(basis.EX, basis.EY, basis.EZ) {
		return massPropertiesFromReadings(massmoment.GeneralPrismProperties(ctx, massPrismRecord(pp), vecMeasurementToInternal(b.centroid), density))
	}
	if !pairbox.RectangularProfile(pp.profile) {
		return massPropertiesFromReadings(massmoment.CardinalPrismProperties(ctx, massPrismRecord(pp), vecMeasurementToInternal(b.centroid), density))
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	return massPropertiesFromReadings(massmoment.CardinalBoxProperties(ctx, massPrismRecord(pp), vecMeasurementToInternal(b.centroid), density))
}

// massPrismRecord presents a root prism payload to the mass integrator.
func massPrismRecord(pp prismPayload) massmoment.PrismRecord {
	return massmoment.PrismRecord{
		Profile: pp.profile, Frame: pp.frame, Transform: pp.xform,
		Z0: pp.z0, Z1: pp.z1, SectionDelta: pp.sectionDelta,
		Z0Delta: pp.z0Delta, Z1Delta: pp.z1Delta,
	}
}

// massCupRecords reads the two prism records whose difference forms a cup.
func massCupRecords(cp cupView) (massmoment.PrismRecord, massmoment.PrismRecord) {
	outer := cp.outerPrism()
	outer.profile = cp.outer
	cavity := cp.cavityPrism()
	cavity.profile = cp.cavity
	return massPrismRecord(outer), massPrismRecord(cavity)
}

// massRevolveRecord presents a root revolve payload to the mass integrator.
func massRevolveRecord(rp revolvePayload) massmoment.RevolveRecord {
	return massmoment.RevolveRecord{
		Profile: rp.profile, Frame: rp.frame, Transform: rp.xform,
		Den: rp.den, Phi0: rp.phi0, Phi1: rp.phi1,
		AU: rp.ax.AU, AV: rp.ax.AV, DU: rp.ax.DU, DV: rp.ax.DV,
		AUBound: rp.ax.AUBound, AVBound: rp.ax.AVBound,
		DUBound: rp.ax.DUBound, DVBound: rp.ax.DVBound,
		SectionDelta: rp.sectionDelta, RadialAdmitAllow: rp.ax.RadialAdmitAllow,
		AxisSnap: rp.ax.Snap != (regionSnapAllow{}),
	}
}

// massSweepRecord presents the sweep's certified span reductions to the mass integrator.
func massSweepRecord(sp sweepPayload) massmoment.SweepRecord {
	record := massmoment.SweepRecord{Arc: sp.arc}
	if len(sp.spans) == 0 {
		if sp.arc {
			record.Revolve = massRevolveRecord(sp.revolve)
		} else {
			record.Prism = massPrismRecord(sp.prism)
		}
		return record
	}
	record.Spans = make([]massmoment.SweepSpanRecord, len(sp.spans))
	for i, span := range sp.spans {
		record.Spans[i] = massmoment.SweepSpanRecord{Arc: span.arc}
		if span.arc {
			record.Spans[i].Revolve = massRevolveRecord(span.revolve)
		} else {
			record.Spans[i].Prism = massPrismRecord(span.prism)
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
