package diameter

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type replayPairMeter struct{ b *proofbound.WorkBudget }

func (m replayPairMeter) ReserveRecords(_ context.Context, n uint64) error { return m.b.Reserve(n) }
func (m replayPairMeter) ReserveBytes(_ context.Context, n uint64) error   { return m.b.ReserveBytes(n) }
func (m replayPairMeter) SpendWork(_ context.Context, n uint64) error {
	_, err := m.b.StepN(n)
	return err
}
func replayPairDistanceDown(ctx context.Context, b *proofbound.WorkBudget, a, c r3.Vec) (float64, error) {
	meter := replayPairMeter{b}
	if err := b.ReserveBytesMul(1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
		return 0, err
	}
	total := new(big.Rat)
	for _, axis := range [3][2]float64{{a.X, c.X}, {a.Y, c.Y}, {a.Z, c.Z}} {
		x, err := proof.ReplayFloatRat(ctx, axis[0], meter)
		if err != nil {
			return 0, err
		}
		y, err := proof.ReplayFloatRat(ctx, axis[1], meter)
		if err != nil {
			return 0, err
		}
		d, err := proof.ReplayRatArithmetic(ctx, x, y, '-', meter)
		if err != nil {
			return 0, err
		}
		square, err := proof.ReplayRatArithmetic(ctx, d, d, '*', meter)
		if err != nil {
			return 0, err
		}
		total, err = proof.ReplayRatArithmetic(ctx, total, square, '+', meter)
		if err != nil {
			return 0, err
		}
	}
	return proof.ReplaySqrtDownRat(ctx, total, meter)
}
