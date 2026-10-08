package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// interval proves positive: lofts, exact stitched solids and curved payloads
// the analytic arms refuse.
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
		return sourceSphereMassProperties(ctx, b, sphere, density)
	}
	if cylinder, ok := sourceRevolvedCylinderAtPose(b, r3.Identity()); ok {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		return sourceRevolvedCylinderMassProperties(ctx, b, cylinder, density)
	}
	if revolve, ok := b.payload.(revolvePayload); ok {
		result, err := revolveMassProperties(ctx, b, revolve, density)
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
		result, err := sweepMassProperties(ctx, b, sweep, density)
		return analyticOrMeshMassProperties(ctx, b, density, result, err)
	}
	if cup, ok := b.payload.(cupPayload); ok {
		result, err := cupMassProperties(ctx, b, cup.view(), density)
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
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!cardinalBasis(basis.EX, basis.EY, basis.EZ) {
		return rotatedPrismMassProperties(ctx, pp, b.centroid, density)
	}
	if !rectangularProfile(pp.profile) {
		return prismMassProperties(ctx, b, pp, density)
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	mass, readings, err := massmoment.CardinalBoxMoments(ctx, pp.profile.Outer.Segments, pp.z0, pp.z1,
		pp.frame, pp.xform, density)
	if err != nil {
		return MassProperties{}, err
	}
	result := MassProperties{Center: b.centroid}
	result.Mass, err = massReading(mass, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Mag() >= result.Mass.Value.Mag() {
		return MassProperties{}, fmt.Errorf("%w: mass interval does not prove positive mass", ErrUnsupported)
	}
	diagonal := [3]*Measurement{&result.Inertia.XX, &result.Inertia.YY, &result.Inertia.ZZ}
	for i, exact := range readings {
		*diagonal[i], err = massReading(exact, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
		if diagonal[i].Bound.Mag() >= diagonal[i].Value.Mag() {
			return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
		}
	}
	zero := Measurement{
		Value:     units.KilogramSquareMillimeters(0),
		Bound:     units.KilogramSquareMillimeters(0),
		Exactness: Exact,
	}
	result.Inertia.XY, result.Inertia.XZ, result.Inertia.YZ = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
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

// prismMassProperties integrates the evaluator's admitted section moments,
// then its recorded axial interval. Every interval is rational, including the
// enclosure of a curved section's published moments. It takes exact
// signed-permutation axes and undisplaced records only, so mapping the local
// tensor to world axes reorders and negates entries without new rounding;
// every other prism takes rotatedPrismMassProperties.
func prismMassProperties(ctx context.Context, b *Body, pp prismPayload, density units.Value) (MassProperties, error) {
	section, err := prismSectionMoments(ctx, pp)
	if err != nil {
		return MassProperties{}, err
	}
	massIv, world, err := massmoment.CardinalPrismInertia(section, pp.z0, pp.z1, pp.frame, pp.xform, density)
	if err != nil {
		return MassProperties{}, err
	}
	result := MassProperties{Center: b.centroid}
	result.Mass, err = massIntervalReading(massIv, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	entries := []struct {
		iv      proofbound.RatInterval
		reading *Measurement
	}{
		{world[0][0], &result.Inertia.XX}, {world[1][1], &result.Inertia.YY},
		{world[2][2], &result.Inertia.ZZ}, {world[0][1], &result.Inertia.XY},
		{world[0][2], &result.Inertia.XZ}, {world[1][2], &result.Inertia.YZ},
	}
	for _, entry := range entries {
		*entry.reading, err = massIntervalReading(entry.iv, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

func massIntervalReading(iv proofbound.RatInterval, unit units.Unit) (Measurement, error) {
	if iv.Lo.Cmp(iv.Hi) == 0 {
		return massReading(iv.Lo, unit)
	}
	held, _ := intervalMid(iv).Float64()
	bound := proofbound.IntervalFloatError(iv, held)
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(bound) {
		return Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", ErrNotFinite)
	}
	return Measurement{Value: units.New(held, unit), Bound: units.New(bound, unit), Exactness: exactnessOf(bound)}, nil
}

func massReading(exact *big.Rat, unit units.Unit) (Measurement, error) {
	value, _ := exact.Float64()
	bound := proofarith.RationalFloatError(exact, value)
	if proofbound.IsNonFinite(value) || proofbound.IsNonFinite(bound) {
		return Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", ErrNotFinite)
	}
	return Measurement{
		Value:     units.New(value, unit),
		Exactness: exactnessOf(bound),
		Bound:     units.New(bound, unit),
	}, nil
}
