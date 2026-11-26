package capblend

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DisableTransfer returns a context under which the named transfer checks
// ("T1", "T2", "T3", "ContourDelta") are switched off. Only builds run with
// that context see the switch.
func DisableTransfer(ctx context.Context, checks ...string) context.Context {
	return context.WithValue(ctx, disabledTransferKey{}, checks)
}

// RequireTransfer runs agent-docs/prism-boolean-design.md Part K §8.3's transfer
// condition on a mixed profile.
func RequireTransfer(ctx context.Context, cbp Payload, mixed momentinput.Profile, loopDelta []float64) error {
	return requireTransfer(ctx, proofbound.NewWorkBudget(ctx), cbp, mixed, loopDelta)
}

// SameWalkSegments exposes sameWalkSegments to the external test package.
var SameWalkSegments = sameWalkSegments
