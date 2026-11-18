package capblend

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DisableTransfer switches transfer checks ("T1", "T2", "T3", "ContourDelta") off
// and returns the function that restores them.
func DisableTransfer(checks ...string) func() {
	disabledTransfer.Store(&checks)
	return func() { disabledTransfer.Store(nil) }
}

// RequireTransfer runs agent-docs/prism-boolean-design.md Part K §8.3's transfer
// condition on a mixed profile.
func RequireTransfer(ctx context.Context, cbp Payload, mixed momentinput.Profile, loopDelta []float64) error {
	return requireTransfer(ctx, proofbound.NewWorkBudget(ctx), cbp, mixed, loopDelta)
}

// SameWalkSegments exposes sameWalkSegments to the external test package.
var SameWalkSegments = sameWalkSegments
