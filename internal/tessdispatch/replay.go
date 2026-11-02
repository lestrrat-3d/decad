package tessdispatch

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/prismtess"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// FacetedReplay restates a faceted solid and runs its one-shell VerifyAll gate
// using the caller's budget. Other payloads and shell counts refuse.
func FacetedReplay(ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if body == nil || body.Doc == nil || budget == nil {
		return nil, errBrakeReplayBoundedTessellationRequiresADocumentBodyAndBudget
	}
	if !(chord > 0) || math.IsInf(chord, 0) {
		return nil, errBrakeReplayInvalidBoundedTessellationChord
	}
	payload, ok := body.Payload.(facetedrecord.Payload)
	if !ok || body.Kind != topo.BodySolid {
		return nil, errBrakeReplayBoundedTessellationRequiresAFacetedSolid
	}
	m, err := facetedrecord.RestateReplayTable(ctx, body, payload, chord, budget)
	if err != nil {
		return nil, err
	}
	if err := topobuild.RequireSingleShellWindingReplay(ctx, m, body.Lumps, budget); err != nil {
		return nil, err
	}
	m.BoundaryOK = true
	return m, nil
}

// RectanglePrismReplay tessellates and verifies one recorded rectangular solid.
func RectanglePrismReplay(ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if body == nil || body.Doc == nil || budget == nil {
		return nil, errBrakeReplayBoundedTessellationRequiresADocumentBodyAndBudget
	}
	if !(chord > 0) || math.IsInf(chord, 0) {
		return nil, errBrakeReplayInvalidBoundedTessellationChord
	}
	payload, ok := body.Payload.(prismrecord.Payload)
	if !ok || body.Kind != topo.BodySolid {
		return nil, errBrakeReplayBoundedTessellationRequiresAPrismSolid
	}
	m, err := prismtess.BuildRectangleReplay(ctx, body, payload, chord, budget)
	if err != nil {
		return nil, err
	}
	if err := topobuild.RequireSingleShellWindingReplay(ctx, m, body.Lumps, budget); err != nil {
		return nil, err
	}
	m.BoundaryOK = true
	return m, nil
}

// BrakeCirclePrismReplay tessellates a circular prism for the brake route.
func BrakeCirclePrismReplay(ctx context.Context, body *topo.Body, chord float64,
	budget *proofbound.ReplayBudget) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if body == nil || body.Doc == nil || budget == nil || !budget.IsBrakeReplay() {
		return nil, errBrakeReplayBoundedCircleRequiresABrakeRouteBodyAndBudget
	}
	if !(chord > 0) || math.IsInf(chord, 0) {
		return nil, errBrakeReplayInvalidBoundedCircleChord
	}
	payload, ok := body.Payload.(prismrecord.Payload)
	if !ok || body.Kind != topo.BodySolid {
		return nil, errBrakeReplayBoundedCircleRequiresAPrismSolid
	}
	m, err := prismtess.BuildCircleReplay(ctx, body, payload, chord, budget)
	if err != nil {
		return nil, err
	}
	if err := topobuild.RequireSingleShellWindingReplay(ctx, m, body.Lumps, budget); err != nil {
		return nil, err
	}
	m.BoundaryOK = true
	return m, nil
}
