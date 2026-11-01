package facetedrecord

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ReplayPlacedBuiltCylinderDeparture proves the exact-image, zero-departure case.
func ReplayPlacedBuiltCylinderDeparture(ctx context.Context, source, moved surfacegeom.Cylinder,
	delta r3.Transform, b *proofbound.ReplayBudget) (float64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if b == nil || !b.IsBrakeReplay() || !facetproof.TranslationOnly(delta) ||
		source.Axis != (r3.Vec{Z: 1}) || source.Axis != moved.Axis || source.Radius != moved.Radius {
		return 0, errBrakeReplayBoundedCylinderImageDiffers
	}
	radius, err := source.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
		return 0, errBrakeReplayBoundedCylinderRadiusIsInvalid
	}
	basis := delta.Basis()
	p, err := proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, delta.Translation(), source.Origin, b)
	if err != nil {
		return 0, err
	}
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		q, err := proof.ReplayReducedFraction(ctx, num, p.W, b)
		if err != nil {
			return 0, err
		}
		gap, err := proof.ReplayRationalFloatError(ctx, q, [3]float64{moved.Origin.X, moved.Origin.Y, moved.Origin.Z}[i], b)
		if err != nil {
			return 0, err
		}
		if gap != 0 {
			return 0, errBrakeReplayBoundedCylinderOriginIsInexact
		}
	}
	return 0, nil
}

// ReplayTranslatedCylinderDeparture matches a translated retained cylinder's normal charge.
func ReplayTranslatedCylinderDeparture(ctx context.Context, source, moved surfacegeom.Cylinder,
	delta r3.Transform, verts []r3.Vec, reach float64, b *proofbound.ReplayBudget) (float64, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if b == nil || !b.IsBrakeReplay() || !facetproof.TranslationOnly(delta) || source.Axis != (r3.Vec{Z: 1}) ||
		moved.Axis != source.Axis || source.Radius != moved.Radius || len(verts) > 65536 ||
		!(reach >= 0) || reach > 1e100 {
		return 0, errBrakeReplayUnsupportedBoundedCylinderPlacement
	}
	radius, err := source.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
		return 0, errBrakeReplayBoundedCylinderRadiusIsInvalid
	}
	// This range proves the ordinary sum of vertex distances stays finite,
	// so its product with the exactly zero axis/rotation error is exactly zero.
	if err := b.SpendWork(ctx, 3*uint64(len(verts))+3); err != nil {
		return 0, err
	}
	valid := func(v r3.Vec) bool {
		if math.Abs(v.X) > 1e100 || math.Abs(v.Y) > 1e100 || math.Abs(v.Z) > 1e100 ||
			proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return false
		}
		return true
	}
	if !valid(source.Origin) {
		return 0, errBrakeReplayBoundedCylinderOriginRange
	}
	for _, v := range verts {
		if !valid(v) {
			return 0, errBrakeReplayBoundedCylinderVertexRange
		}
	}
	basis := delta.Basis()
	p, err := proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, delta.Translation(), source.Origin, b)
	if err != nil {
		return 0, err
	}
	var ideal [3]*big.Rat
	var gaps [3]float64
	held := [3]float64{moved.Origin.X, moved.Origin.Y, moved.Origin.Z}
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		ideal[i], err = proof.ReplayReducedFraction(ctx, num, p.W, b)
		if err != nil {
			return 0, err
		}
		gaps[i], err = proof.ReplayRationalFloatError(ctx, ideal[i], held[i], b)
		if err != nil {
			return 0, err
		}
	}
	pointGap := proofbound.Radius3D(max(gaps[0], gaps[1], gaps[2]))
	var square [2]*big.Rat
	for i := range 2 {
		q, err := proof.ReplayFloatRat(ctx, held[i], b)
		if err != nil {
			return 0, err
		}
		d, err := proof.ReplayRatArithmetic(ctx, ideal[i], q, '-', b)
		if err != nil {
			return 0, err
		}
		square[i], err = proof.ReplayRatArithmetic(ctx, d, d, '*', b)
		if err != nil {
			return 0, err
		}
	}
	sum, err := proof.ReplayRatArithmetic(ctx, square[0], square[1], '+', b)
	if err != nil {
		return 0, err
	}
	across := 0.0
	if sum.Sign() != 0 {
		up, err := replayPlaneFloatBound(ctx, sum, true, b)
		if err != nil {
			return 0, err
		}
		num := math.Nextafter(math.Sqrt(up), math.Inf(1))
		across = proofbound.DivUpper(num, math.Nextafter(1, 0))
	}
	pGap := min(pointGap, across)
	shift := proofbound.AbsSumUpper(pGap, 0)
	term, ok := HeldNormalTerm(source, proofbound.ProductUpper(2, shift), 0)
	if !ok {
		return 0, errBrakeReplayTranslatedCylinderNormalBoundUnavailable
	}
	return proofbound.AbsSumUpper(0, term), nil
}
