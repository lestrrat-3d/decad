package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func replayBuiltFace(ctx context.Context, body *topo.Body, face *topo.Face, budget proof.ReplayKeyBudget) (bool, error) {
	if body == nil {
		return false, nil
	}
	return body.ReplayPrismBuiltFace(ctx, face, budget)
}
