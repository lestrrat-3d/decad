package tessdispatch

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// FacetedReplay restates a faceted solid and runs its one-shell VerifyAll gate
// using the caller's budget. Other payloads and shell counts refuse.
func FacetedReplay(ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	return boundedReplay(ctx, body, chord, budget,
		errBrakeReplayBoundedTessellationRequiresAFacetedSolid,
		facetedrecord.RestateReplayTable)
}

// RectanglePrismReplay tessellates and verifies one recorded rectangular solid.
func RectanglePrismReplay(ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	return boundedReplay(ctx, body, chord, budget,
		errBrakeReplayBoundedTessellationRequiresAPrismSolid,
		prismtess.BuildRectangleReplay)
}

func boundedReplay[T any](ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget, payloadError error,
	build func(context.Context, *topo.Body, T, float64, *proofbound.ReplayBudget) (*topo.Mesh, error),
) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if body == nil || body.Doc == nil || budget == nil {
		return nil, errBrakeReplayBoundedTessellationRequiresADocumentBodyAndBudget
	}
	if !(chord > 0) || math.IsInf(chord, 0) {
		return nil, errBrakeReplayInvalidBoundedTessellationChord
	}
	payload, ok := body.Payload.(T)
	if !ok || body.Kind != topo.BodySolid {
		return nil, payloadError
	}
	m, err := build(ctx, body, payload, chord, budget)
	if err != nil {
		return nil, err
	}
	if err := topobuild.RequireSingleShellWindingReplay(ctx, m, body.Lumps, budget); err != nil {
		return nil, err
	}
	m.BoundaryOK = true
	return m, nil
}
