package prismplacement

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// ReplayLevelIdentity binds a level to its original pointer and value.
type ReplayLevelIdentity = denotation.ReplayLevelIdentity

// CaptureReplayLevelIdentity copies a bounded level before publication.
func CaptureReplayLevelIdentity(ctx context.Context, d LevelDenotation, b proof.ReplayKeyBudget) (ReplayLevelIdentity, bool, error) {
	return denotation.CaptureReplayLevelIdentity(ctx, d, b)
}
