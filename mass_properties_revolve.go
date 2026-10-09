package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/massmoment"
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
// rather than a tensor missing that term. The local moments reach world axes
// through the exact rational product of the placement basis and the local
// basis, the map the revolve's volume, area and vertices denote through
// (docs/evaluator-design.md §6): massmoment.AffineInertia takes that map's
// exact image, so the mass carries |det L| as Volume does and no
// orthonormality widening is needed.
func revolveMassProperties(ctx context.Context, b *Body, rp revolvePayload, density units.Value) (MassProperties, error) {
	moments, err := revolveVolumeMoments(ctx, rp)
	if err != nil {
		return MassProperties{}, err
	}
	// The axis is exact here (revolveVolumeMoments refuses any other), so the
	// local basis (w, e0, e1) is orthonormal in plane coordinates and this
	// matrix is exactly the map the record denotes through, L·[d e0 e1]
	// (docs/evaluator-design.md §6). The tensor is that map's exact image.
	linear, err := massmoment.RevolveRotation(rp.frame, rp.ax.dU, rp.ax.dV, rp.xform)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := massmoment.AffineInertia(moments, linear, density)
	if err != nil {
		return MassProperties{}, err
	}
	return massmoment.Publish(ctx, b.centroid, massIv, world)
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
	if !rp.den.Phi0.Valid() || !rp.den.Phi1.Valid() {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve sweep has no exact denotation", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return massmoment.Moments{}, err
	}

	plane, err := massmoment.RevolveSectionMoments(ctx, rp.profile)
	if err != nil {
		return massmoment.Moments{}, err
	}
	angular, ok := massmoment.RevolveSweepFactors(rp.den, rp.phi0, rp.phi1)
	if !ok {
		return massmoment.Moments{}, fmt.Errorf("%w: revolve sweep has no certified angular factors", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return massmoment.Moments{}, err
	}
	return massmoment.RevolveMoments(plane, rp.ax.aU, rp.ax.aV, rp.ax.dU, rp.ax.dV, angular)
}
