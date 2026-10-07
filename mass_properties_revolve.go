package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// This file is the general revolve's mass path (docs/dynamic-mass-design.md
// §2.1, docs/multibody-dynamics-design.md §8.6): a full or partial revolve of
// any section the moment engine integrates, from the recorded section's
// moments through third order and the sweep's certified angular factors.
//
// In the axis frame a section point (z, ρ) at sweep angle φ sits at
// a3 + z·w + ρ·(cos φ·e0 + sin φ·e1) (revolvePayload.point), and the volume
// element is ρ dA dφ. About the axis anchor a3, in the local basis (w, e0, e1):
//
//	V     = Δφ·∫ρ
//	P     = (Δφ·∫zρ,  Sφ·∫ρ²,  Cφ·∫ρ²)
//	Q_ww  = Δφ·∫z²ρ   Q_w0 = Sφ·∫zρ²   Q_w1 = Cφ·∫zρ²
//	Q_00  = ∫cos²φ·∫ρ³   Q_01 = ∫sinφcosφ·∫ρ³   Q_11 = ∫sin²φ·∫ρ³
//
// with Sφ = ∫cos φ dφ and Cφ = ∫sin φ dφ over [φ0, φ1], every section
// integral over dA. The ∫ρ³, ∫zρ² and ∫z²ρ terms are the third-order ones
// freeform.MomentThirdOrder supplies. A full turn's Sφ, Cφ and ∫sinφcosφ are the exact
// zero and its ∫cos² and ∫sin² are π, through the same formulas, because the
// turn-stated endpoints have exact sine and cosine (quarterTurnSinCos).
//
// Every quantity is a rational interval: section moments exactly or from
// their published bounds, the axis frame exactly, the angular factors from
// the payload's own sweep denotation, density exactly. The centroidal tensor
// is S = Q − P·Pᵀ/V over one common V, P, Q enclosure, and the inertia
// I = ρ_m·(trace(S)·1 − S).

// revolveMassProperties integrates the general revolve. Every gate below only
// refuses: a payload whose readings carry a term this path does not charge —
// an axis-snap or admitted-band allowance, an uncertain axis, a sweep end
// without a denotation, a section displacement — returns ErrUnsupported
// rather than a tensor missing that term. The local tensor reaches world axes
// through docs/multibody-dynamics-design.md §8.1's rotation: the exact
// rational product of the placement basis and the local basis, widened by
// its orthonormality defect so the reading refers to the rigid rotation
// nearest it.
func revolveMassProperties(ctx context.Context, b *Body, rp revolvePayload, density units.Value) (MassProperties, error) {
	moments, err := revolveVolumeMoments(ctx, rp)
	if err != nil {
		return MassProperties{}, err
	}
	rotation, err := massmoment.RevolveRotation(rp.frame, rp.ax.dU, rp.ax.dV, rp.xform)
	if err != nil {
		return MassProperties{}, err
	}
	return rigidMassProperties(ctx, b.centroid, moments, rotation, density)
}

// revolveVolumeMoments integrates the revolve's V, P and Q about the axis
// anchor a3 in the local basis (w, e0, e1), refusing every term it does not
// charge.
func revolveVolumeMoments(ctx context.Context, rp revolvePayload) (massmoment.Moments, error) {
	if rp.sectionDelta != 0 {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve section carries a displacement the mass path does not charge", ErrUnsupported)
	}
	ax := rp.ax
	if ax.aUBound != 0 || ax.aVBound != 0 || ax.dUBound != 0 || ax.dVBound != 0 {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve axis is not exact", ErrUnsupported)
	}
	if ax.radialAdmitAllow != 0 || ax.snap != (regionSnapAllow{}) {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve axis snap is not charged by the mass path", ErrUnsupported)
	}
	if !rp.den.phi0.valid() || !rp.den.phi1.valid() {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve sweep has no exact denotation", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return massmoment.Moments{}, err
	}

	ig, err := rp.profile.evaluatorIntegralsContext(ctx, freeform.MomentThirdOrder, nil)
	if err != nil {
		return massmoment.Moments{}, err
	}
	plane, err := revolveSectionMoments(ig)
	if err != nil {
		return massmoment.Moments{}, err
	}
	angular, ok := revolveAngularFactors(rp)
	if !ok {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve sweep has no certified angular factors", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return massmoment.Moments{}, err
	}
	return massmoment.RevolveMoments(plane, rp.ax.aU, rp.ax.aV, rp.ax.dU, rp.ax.dV, angular)
}

// rigidMassProperties publishes the mass properties of a solid whose local
// moments are m and whose local axes reach world axes through the rigid
// rotation nearest the exact rational matrix rotation (its column k the
// world image of local axis k). It forms the centroidal tensor from the one
// V, P, Q enclosure, rotates it with docs/multibody-dynamics-design.md §8.1's
// orthonormality-defect widening, and proves the PUBLISHED tensor positive
// definite by its leading principal minors. center is the evaluator's own
// bounded world centroid of the same solid.
func rigidMassProperties(ctx context.Context, center VecMeasurement, m massmoment.Moments, rotation [3][3]*big.Rat, density units.Value) (MassProperties, error) {
	world, rho, err := massmoment.RigidInertia(m, rotation, density)
	if err != nil {
		return MassProperties{}, err
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	result := MassProperties{Center: center}
	result.Mass, err = massIntervalReading(proofbound.IntervalScale(m.Volume, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: mass reading is not positive", ErrUnsupported)
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
	// The proof runs on the PUBLISHED readings, so every tensor a caller can
	// read inside the six bounds is positive definite, not only the rational
	// box they were rounded from.
	if !massmoment.PositiveDefinite(publishedTensor(result.Inertia)) {
		return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// publishedTensor is the interval tensor the six readings state: each held
// value widened by its own bound, both read as exact rationals.
func publishedTensor(reading InertiaReading) [3][3]proofbound.RatInterval {
	entry := func(m Measurement) proofbound.RatInterval {
		return proofbound.IntervalWiden(proofbound.PointInterval(proofarith.FloatRat(m.Value.Base())), proofarith.FloatRat(m.Bound.Base()))
	}
	xy, xz, yz := entry(reading.XY), entry(reading.XZ), entry(reading.YZ)
	return [3][3]proofbound.RatInterval{
		{entry(reading.XX), xy, xz},
		{xy, entry(reading.YY), yz},
		{xz, yz, entry(reading.ZZ)},
	}
}

// revolveSectionMoments returns the recorded section's plane-origin moments
// m[p][q] = ∫u^p·v^q dA for p + q ≤ 3 as rational enclosures. Orders up to
// two are the region's exact rationals where it has them and otherwise its
// published values widened by their proven bounds, as prismMassProperties
// reads them; the third order is the engine's own enclosure.
func revolveSectionMoments(ig regionIntegrals) ([4][4]proofbound.RatInterval, error) {
	var m [4][4]proofbound.RatInterval
	slots := [6]*proofbound.RatInterval{&m[0][0], &m[1][0], &m[0][1], &m[2][0], &m[1][1], &m[0][2]}
	if !ig.exactDead && ig.exact.Complete() {
		for i, value := range ig.exact.Fields() {
			*slots[i] = proofbound.PointInterval(value)
		}
	} else {
		values := [6]proofbound.BoundedScalar{
			{Value: ig.area, Bound: ig.areaBound}, {Value: ig.mu, Bound: ig.muBound}, {Value: ig.mv, Bound: ig.mvBound},
			{Value: ig.muu, Bound: ig.muuBound}, {Value: ig.muv, Bound: ig.muvBound}, {Value: ig.mvv, Bound: ig.mvvBound},
		}
		for i, value := range values {
			iv, err := massMomentInterval(value)
			if err != nil {
				return m, err
			}
			*slots[i] = iv
		}
	}
	third, ok := ig.thirdMoments()
	if !ok {
		return m, fmt.Errorf("%w: revolve section has no third-order moment enclosure", ErrUnsupported)
	}
	m[3][0], m[2][1], m[1][2], m[0][3] = third[0], third[1], third[2], third[3]
	return m, nil
}

// revolveAngularFactors reads the sweep's certified width and endpoint values.
func revolveAngularFactors(rp revolvePayload) (massmoment.RevolveAngular, bool) {
	width, ok := rp.den.widthInterval()
	if !ok {
		return massmoment.RevolveAngular{}, false
	}
	s0, c0, ok0 := rp.den.phi0.sinCosFor(rp.phi0)
	s1, c1, ok1 := rp.den.phi1.sinCosFor(rp.phi1)
	if !ok0 || !ok1 {
		return massmoment.RevolveAngular{}, false
	}
	return massmoment.AngularFactors(width, s0, c0, s1, c1), true
}
