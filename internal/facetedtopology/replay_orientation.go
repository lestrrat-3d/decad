package facetedtopology

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func replayOrientation(ctx context.Context, budget *proofbound.WorkBudget, a, b, c, d r3.Vec) (int, error) {
	meter := budget.ReplayID
	if meter == nil {
		return 0, fmt.Errorf("%w: missing replay orientation meter", decaderr.ErrUnsupported)
	}
	if err := meter.SpendWork(ctx, 256); err != nil {
		return 0, err
	}
	if sign, ok := proof.OrientSignFloat(a, b, c, d); ok {
		return sign, nil
	}
	reserve := func(n uint64) error {
		if n > ^uint64(0)/64 {
			return fmt.Errorf("%w: replay orientation size", decaderr.ErrUnsupported)
		}
		if err := meter.ReserveBytes(ctx, n*64); err != nil {
			return err
		}
		return meter.ReserveRecords(ctx, n)
	}
	n, err := proof.TriangleNormalReplay(a, b, c, reserve, func(n uint64) error { return meter.SpendWork(ctx, n) })
	if err != nil {
		return 0, err
	}
	xa, err := proof.XptOfBounded(a, reserve)
	if err != nil {
		return 0, err
	}
	xd, err := proof.XptOfBounded(d, reserve)
	if err != nil {
		return 0, err
	}
	return proof.ReplayXdotDiffSign(ctx, n, xd, xa, meter)
}
