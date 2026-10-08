package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/units"
)

// This file is the cup's mass path (docs/multibody-dynamics-design.md §8.8,
// docs/dynamic-mass-design.md §2 and §3). A cup is its outer prism minus its
// cavity prism: the two share the plane frame and the placement, and the
// cavity lies inside the outer solid. Each prism is integrated on its own
// interval by prismVolumeMoments, which charges its own level displacements
// as an occupied-volume error, so each contribution carries its own outward
// interval before the subtraction; the offset region's prism charges the
// cup's offsetDelta that way too. The cavity's moments are re-anchored
// exactly onto the outer prism's mid level, subtracted at the V, P, Q level,
// and the difference reaches world axes as its exact image under the shared
// frame and placement basis (massmoment.AffineInertia), the map the cup's
// volume and vertices denote through.

// cupMassProperties integrates the outer prism and the cavity prism and
// publishes their difference.
func cupMassProperties(ctx context.Context, b *Body, cp cupView, density units.Value) (MassProperties, error) {
	outer := cp.outerPrism()
	outer.profile = cp.outer
	cavity := cp.cavityPrism()
	cavity.profile = cp.cavity
	// The offset region's prism already carries the cup's offsetDelta as its
	// sectionDelta (outerPrism, cavityPrism): the recorded offset section is
	// within it of the offset the denoted thickness names.
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
	void = massmoment.Shift(void, [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat).Sub(cavityMid, outerMid)})
	rotation, err := prismRotation(outer)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := massmoment.AffineInertia(massmoment.Sub(solid, void), rotation, density)
	if err != nil {
		return MassProperties{}, err
	}
	return publishMassProperties(ctx, b.centroid, massIv, world)
}
