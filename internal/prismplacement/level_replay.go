package prismplacement

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

var errReplayLevel = fmt.Errorf("%w: unsupported zero-displacement level", decaderr.ErrUnsupported)

// ValidateZeroLevelReplay checks a retained zero-displacement denotation. Its
// capped rational conversion is charged to the caller's shared replay meter.
func ValidateZeroLevelReplay(ctx context.Context, held float64, d LevelDenotation, b *proofbound.ReplayBudget) error {
	if b == nil || math.IsNaN(held) || math.IsInf(held, 0) {
		return errReplayLevel
	}
	if err := b.SpendWork(ctx, 1); err != nil {
		return err
	}
	if d.exact == nil {
		if d != (LevelDenotation{}) {
			return errReplayLevel
		}
		return nil
	}
	if math.Float64bits(d.held) != math.Float64bits(held) || math.Float64bits(d.delta) != 0 {
		return errReplayLevel
	}
	return ValidateHeldRationalReplay(ctx, held, d.exact, b)
}

// ValidateHeldRationalReplay checks an exact rational against a held float.
func ValidateHeldRationalReplay(ctx context.Context, held float64, exactValue *big.Rat, b *proofbound.ReplayBudget) error {
	if b == nil || exactValue == nil || math.IsNaN(held) || math.IsInf(held, 0) {
		return errReplayLevel
	}
	// Each input has at most 34 limbs even on a 32-bit target. The fixed work
	// and storage envelope covers their division and its temporary limbs.
	if err := b.SpendWork(ctx, 65536); err != nil {
		return err
	}
	if err := b.ReserveRecords(ctx, 8); err != nil {
		return err
	}
	if err := b.ReserveBytes(ctx, 32768); err != nil {
		return err
	}
	if exactValue.Num().BitLen() > 1076 || exactValue.Denom().BitLen() > 1076 {
		return errReplayLevel
	}
	value, exact := exactValue.Float64()
	if !exact || value != held {
		return errReplayLevel
	}
	return nil
}
