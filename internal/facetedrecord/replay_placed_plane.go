package facetedrecord

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/r3"
)

// replayTranslatedPlane matches ordinary retained-plane departure for exact
// translations, including rounded origins.
func replayTranslatedPlane(ctx context.Context, source surfacegeom.Plane, delta r3.Transform, b *proofbound.ReplayBudget) (surfacegeom.Plane, float64, error) {
	if !facetproof.TranslationOnly(delta) {
		return surfacegeom.Plane{}, 0, errBrakeReplayBoundedPlanePlacementRequiresTranslation
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return surfacegeom.Plane{}, 0, err
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return surfacegeom.Plane{}, 0, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(surfacegeom.Plane{}))); err != nil {
		return surfacegeom.Plane{}, 0, err
	}
	movedAny, err := surfaceplace.TransformSurface(source, delta)
	if err != nil {
		return surfacegeom.Plane{}, 0, err
	}
	moved, ok := movedAny.(surfacegeom.Plane)
	if !ok {
		return surfacegeom.Plane{}, 0, errBrakeReplayTranslatedPlaneFrameDiffers
	}
	departure, err := ReplayPlacedBuiltPlaneDeparture(ctx, source, moved, delta, b)
	return moved, departure, err
}

// ReplayPlacedBuiltPlaneDeparture matches the ordinary departure for a
// translated built plane whose held directions are the exact source directions.
func ReplayPlacedBuiltPlaneDeparture(ctx context.Context, source, moved surfacegeom.Plane, delta r3.Transform, b *proofbound.ReplayBudget) (float64, error) {
	if b == nil || !facetproof.TranslationOnly(delta) || moved.Frame.U() != source.Frame.U() || moved.Frame.V() != source.Frame.V() {
		return 0, errBrakeReplayTranslatedPlaneDirectionsDiffer
	}
	basis := delta.Basis()
	p, err := proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, delta.Translation(), source.Frame.Origin(), b)
	if err != nil {
		return 0, err
	}
	origin := moved.Frame.Origin()
	var gaps [3]float64
	var ideal [3]*big.Rat
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		q, err := proof.ReplayReducedFraction(ctx, num, p.W, b)
		if err != nil {
			return 0, err
		}
		ideal[i] = q
		gaps[i], err = proof.ReplayRationalFloatError(ctx, q, [3]float64{origin.X, origin.Y, origin.Z}[i], b)
		if err != nil {
			return 0, err
		}
	}
	maxGap := max(gaps[0], gaps[1], gaps[2])
	if maxGap == 0 {
		return 0, nil
	}
	along, err := replayPlaneNormalGap(ctx, ideal, origin, source.Frame.U(), source.Frame.V(), b)
	if err != nil {
		return 0, err
	}
	pGap := min(proofbound.Radius3D(maxGap), along)
	return proofbound.AbsSumUpper(0, 0, pGap), nil
}
