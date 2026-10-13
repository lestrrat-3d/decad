package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// spiralCombineInputs authenticates every tool as a copy of the first trim.
// Distinct sector projections prove the complete source solids disjoint before
// their meshes enter one Boolean operand.
func spiralCombineInputs(ctx context.Context, blank *Body, tools []*Body) (*spiralCombineSource, error) {
	if len(tools) < 2 || len(tools) > 31 {
		return nil, fmt.Errorf("%w: this spiral-tooth Union class accepts two to 31 tools", ErrUnsupported)
	}
	first, ok, err := certifySpiralFirstJoin(ctx, blank, tools[0])
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, fmt.Errorf("%w: target and first tool have no spiral source certificate", ErrUnsupported)
	}
	moves := make([]r3.Transform, len(tools))
	moves[0] = r3.Identity()
	for i, tool := range tools[1:] {
		p, ok := tool.payload.(facetedPayload)
		if !ok || p.spiralTooth == nil || p.spiralTooth.source != first.tooth ||
			p.spiralTooth.motion != p.xform || !spiralNearAxisRotation(p.xform) {
			return nil, fmt.Errorf("%w: tool %d is not a rigid copy of the first spiral tooth", ErrUnsupported, i+2)
		}
		moves[i+1] = p.xform
	}
	shells, ok := spiralToolShells(tools)
	if !ok || !spiralsector.Separate(ctx, shells, moves, false) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, fmt.Errorf("%w: spiral tooth sectors are not proven disjoint", ErrUnsupported)
	}
	heldParts, err := spiralPlacedHeldTools(ctx, tools, moves)
	if err != nil {
		return nil, err
	}
	if !spiralsector.Separate(ctx, heldParts, moves, true) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		return nil, fmt.Errorf("%w: spiral held tooth shells are not proven disjoint", ErrUnsupported)
	}
	return &spiralCombineSource{first: first, tools: tools, moves: moves}, nil
}

// spiralToolShells reads each tool's held mesh for spiralsector.Separate. It
// reports false when a tool is not faceted.
func spiralToolShells(tools []*Body) ([]spiralsector.Shell, bool) {
	shells := make([]spiralsector.Shell, len(tools))
	for i, tool := range tools {
		p, ok := tool.payload.(facetedPayload)
		if !ok {
			return nil, false
		}
		shells[i] = spiralsector.Shell{Verts: p.verts, Tris: p.tris,
			VertexBound: p.vertexBound, MeshBound: p.meshBound}
	}
	return shells, true
}

// spiralPlacedHeldTools reads each tool's held mesh and source link for
// spiralsector.PlacedHeldShells. spiralCombineInputs passes non-nil tools and
// len(moves) == len(tools).
func spiralPlacedHeldTools(ctx context.Context, tools []*Body, moves []r3.Transform) ([]spiralsector.Shell, error) {
	copies := make([]spiralsector.HeldCopy, len(tools))
	var trim *loftFacesTrimRecord
	for i, tool := range tools {
		p, ok := tool.payload.(facetedPayload)
		if !ok {
			continue
		}
		copies[i] = spiralsector.HeldCopy{Verts: p.verts, Tris: p.tris}
		if i == 0 {
			trim = p.loftFacesTrim
			copies[i].Linked = trim != nil
			continue
		}
		copies[i].Linked = trim != nil && i < len(moves) && p.spiralTooth != nil &&
			p.spiralTooth.source == trim && p.spiralTooth.motion == moves[i] && p.xform == moves[i]
	}
	return spiralsector.PlacedHeldShells(ctx, copies, moves)
}
