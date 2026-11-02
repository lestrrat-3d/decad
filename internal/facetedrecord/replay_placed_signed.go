package facetedrecord

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func replayExactSignedImage(ctx context.Context, delta r3.Transform, source, moved r3.Vec, direction bool,
	b *proofbound.ReplayBudget) error {
	if b == nil || !b.IsBrakeReplay() {
		return errBrakeReplaySignedPlacementUnavailable
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return err
	}
	if !replaySignedAxes(delta) {
		return errBrakeReplaySignedPlacementUnavailable
	}
	basis := delta.Basis()
	translation := delta.Translation()
	if direction {
		translation = r3.Vec{}
	}
	p, err := proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, translation, source, b)
	if err != nil {
		return err
	}
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		q, err := proof.ReplayReducedFraction(ctx, num, p.W, b)
		if err != nil {
			return err
		}
		gap, err := proof.ReplayRationalFloatError(ctx, q, [3]float64{moved.X, moved.Y, moved.Z}[i], b)
		if err != nil {
			return err
		}
		if gap != 0 {
			return errBrakeReplaySignedPlacementImageIsRounded
		}
	}
	return nil
}

func replaySignedAxes(delta r3.Transform) bool {
	if !delta.IsValid() || !proofbound.FiniteVec(delta.Translation()) {
		return false
	}
	basis := delta.Basis()
	seen := [3]bool{}
	for _, v := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		axis := -1
		for i, x := range [3]float64{v.X, v.Y, v.Z} {
			if x == 1 || x == -1 {
				if axis != -1 {
					return false
				}
				axis = i
			} else if x != 0 {
				return false
			}
		}
		if axis < 0 || seen[axis] {
			return false
		}
		seen[axis] = true
	}
	return true
}

// ReplaySignedBuiltPlaneDeparture admits only an exact signed-axis image.
func ReplaySignedBuiltPlaneDeparture(ctx context.Context, source, moved surfacegeom.Plane, delta r3.Transform,
	b *proofbound.ReplayBudget) (float64, error) {
	if err := replayExactSignedImage(ctx, delta, source.Frame.Origin(), moved.Frame.Origin(), false, b); err != nil {
		return 0, err
	}
	if err := replayExactSignedImage(ctx, delta, source.Frame.U(), moved.Frame.U(), true, b); err != nil {
		return 0, err
	}
	if err := replayExactSignedImage(ctx, delta, source.Frame.V(), moved.Frame.V(), true, b); err != nil {
		return 0, err
	}
	return 0, nil
}

// ReplaySignedBuiltCylinderDeparture admits only an exact signed-axis image.
func ReplaySignedBuiltCylinderDeparture(ctx context.Context, source, moved surfacegeom.Cylinder, delta r3.Transform,
	b *proofbound.ReplayBudget) (float64, error) {
	radius, err := source.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) || source.Radius != moved.Radius {
		return 0, errBrakeReplaySignedCylinderRadiusDiffers
	}
	if err := replayExactSignedImage(ctx, delta, source.Origin, moved.Origin, false, b); err != nil {
		return 0, err
	}
	if err := replayExactSignedImage(ctx, delta, source.Axis, moved.Axis, true, b); err != nil {
		return 0, err
	}
	return 0, nil
}
