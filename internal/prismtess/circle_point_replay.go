package prismtess

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func circlePointRoundReplay(ctx context.Context, pp prismrecord.Payload, u, v, z float64, held r3.Vec,
	b *proofbound.ReplayBudget) (float64, error) {
	if b == nil || !b.IsBrakeReplay() || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Frame.N() != (r3.Vec{Z: 1}) {
		return 0, fmt.Errorf("%w: bounded circle requires an axis-aligned frame", decaderr.ErrUnsupported)
	}
	basis := pp.Xform.Basis()
	if basis.EX != (r3.Vec{X: 1}) || basis.EY != (r3.Vec{Y: 1}) || basis.EZ != (r3.Vec{Z: 1}) {
		return 0, fmt.Errorf("%w: bounded circle requires a translation", decaderr.ErrUnsupported)
	}
	o, tr := pp.Frame.Origin(), pp.Xform.Translation()
	origin, move := [3]float64{o.X, o.Y, o.Z}, [3]float64{tr.X, tr.Y, tr.Z}
	local, actual := [3]float64{u, v, z}, [3]float64{held.X, held.Y, held.Z}
	maxGap := 0.0
	for i := range 3 {
		a, err := proof.ReplayFloatRat(ctx, origin[i], b)
		if err != nil {
			return 0, err
		}
		p, err := proof.ReplayFloatRat(ctx, local[i], b)
		if err != nil {
			return 0, err
		}
		a, err = proof.ReplayRatArithmetic(ctx, a, p, '+', b)
		if err != nil {
			return 0, err
		}
		m, err := proof.ReplayFloatRat(ctx, move[i], b)
		if err != nil {
			return 0, err
		}
		a, err = proof.ReplayRatArithmetic(ctx, a, m, '+', b)
		if err != nil {
			return 0, err
		}
		d, err := proof.ReplayRationalFloatError(ctx, a, actual[i], b)
		if err != nil {
			return 0, err
		}
		maxGap = max(maxGap, d)
	}
	return proofbound.Radius3D(maxGap), nil
}
