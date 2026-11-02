package prismplacement

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ValidateZeroLevelReplay checks a retained zero-displacement denotation.
func ValidateZeroLevelReplay(ctx context.Context, held float64, d LevelDenotation, b *proofbound.ReplayBudget) error {
	return denotation.ValidateZeroLevelReplay(ctx, held, d, b)
}

// ValidateHeldRationalReplay checks an exact rational against a held float.
func ValidateHeldRationalReplay(ctx context.Context, held float64, q *big.Rat, b *proofbound.ReplayBudget) error {
	return denotation.ValidateHeldRationalReplay(ctx, held, q, b)
}
