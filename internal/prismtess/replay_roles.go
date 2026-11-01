package prismtess

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

func rectangleRolesReplay(ctx context.Context, b *topo.Body, budget *proofbound.ReplayBudget) (*topobuild.ReplayRoles, error) {
	return topobuild.NewReplayRoles(ctx, b, budget)
}
