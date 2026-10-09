package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/units"
)

// This file owns the mass properties of an untapered prism whose frame or
// placement basis is not a signed permutation, or whose recorded section or
// sweep levels carry a proven displacement (docs/multibody-dynamics-design.md
// §8.1 and §8.2, docs/dynamic-mass-design.md §2.1 and §3).
//
// The volume moments are integrated in the frame-local coordinates
// q = (u, v, z - zm), zm the recorded mid level, then rotated into world axes
// by the exact rational product M of the placement basis and the frame axes.
// Two certificates join the analytic integrals:
//
//   - The occupied-volume error E between the recorded prism and the prism its
//     construction denotes widens V by E, each P_i by R·E and each Q_ij by
//     R²·E, R bounding every |q_i| over both prisms (dynamic-mass §2.2).
//   - The held basis M is orthonormal only to rounding. The prism's volume,
//     areas and vertices denote the image of the frame-local solid under M
//     read exactly (docs/evaluator-design.md §5.1), so the reading is that
//     image's (massmoment.AffineInertia): mass ρ·|det M|·V, and the inertia
//     of the second moment |det M|·M·S·Mᵀ. Positivity is proved on the
//     published tensor by its leading principal minors.
//
// massmoment.PrismProfileMoments and the internal/massmoment transforms are shared
// with the sweep and cup paths, which
// combine several solids' V, P and Q about one anchor before forming a
// tensor.

// rotatedPrismMassProperties integrates pp's admitted section moments over
// its axial interval, charges any recorded displacement, and rotates the
// centroidal tensor into world axes. The caller has already admitted pp as a
// solid prism; this path takes every frame and placement basis, cardinal ones
// included.
func rotatedPrismMassProperties(ctx context.Context, pp prismPayload, center VecMeasurement, density units.Value) (MassProperties, error) {
	moments, err := massmoment.PrismProfileMoments(ctx, pp.profile, pp.z0, pp.z1,
		pp.sectionDelta, pp.z0Delta, pp.z1Delta)
	if err != nil {
		return MassProperties{}, err
	}
	basis, err := massmoment.PrismRotation(pp.frame, pp.xform)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := massmoment.AffineInertia(moments, basis, density)
	if err != nil {
		return MassProperties{}, err
	}
	return massmoment.Publish(ctx, center, massIv, world)
}
