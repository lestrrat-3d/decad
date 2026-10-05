package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/units"
)

// This file is the cup's mass path (docs/multibody-dynamics-design.md §8.8,
// docs/dynamic-mass-design.md §2 and §3). A cup is its outer prism minus its
// cavity prism: the two share the plane frame and the placement, and the
// cavity lies inside the outer solid. Each prism is integrated on its own
// interval by prismVolumeMoments, which charges its own level displacements
// as an occupied-volume error, so each contribution carries its own outward
// interval before the subtraction. The cavity's moments are re-anchored
// exactly onto the outer prism's mid level, subtracted at the V, P, Q level,
// and the difference reaches world axes through the shared frame and
// placement as one rotated solid.

// cupMassProperties integrates the outer prism and the cavity prism and
// publishes their difference.
func cupMassProperties(ctx context.Context, b *Body, cp cupPayload, density units.Value) (MassProperties, error) {
	outer := cp.outerPrism()
	outer.profile = cp.outer
	cavity := cp.cavityPrism()
	cavity.profile = cp.cavity
	// One region is the other offset by the thickness as held in
	// millimetres. The denoted offset differs by at most the thickness's own
	// conversion displacement, so that is the offset section's displacement.
	if cp.sense == Outward {
		outer.sectionDelta = cp.thicknessDelta
	} else {
		cavity.sectionDelta = cp.thicknessDelta
	}
	outerMid, err := prismMidLevel(outer)
	if err != nil {
		return MassProperties{}, err
	}
	cavityMid, err := prismMidLevel(cavity)
	if err != nil {
		return MassProperties{}, err
	}
	solid, err := prismVolumeMoments(ctx, outer)
	if err != nil {
		return MassProperties{}, err
	}
	void, err := prismVolumeMoments(ctx, cavity)
	if err != nil {
		return MassProperties{}, err
	}
	// Both are frame-local about their own (0, 0, zm). A cavity coordinate
	// about the outer mid level is its own plus (0, 0, zm_cavity − zm_outer).
	void = shiftVolumeMoments(void, [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat).Sub(cavityMid, outerMid)})
	rotation, err := prismRotation(outer)
	if err != nil {
		return MassProperties{}, err
	}
	return rigidMassProperties(ctx, b.centroid, subVolumeMoments(solid, void), rotation, density)
}
