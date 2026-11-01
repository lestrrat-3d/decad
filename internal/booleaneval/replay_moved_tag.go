package booleaneval

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/r3"
)

func replayMovedGroup(ctx context.Context, g facetedrecord.Group, piece, pieces int, sh pieceShift,
	verts []r3.Vec, reach float64, b *proofbound.ReplayBudget) (facetedrecord.Group, error) {
	if err := ctx.Err(); err != nil {
		return facetedrecord.Group{}, err
	}
	if b == nil || !b.IsBrakeReplay() || pieces != 1 || piece != 0 || sh.Grow != 0 || sh.Move == (r3.Vec{}) ||
		proofbound.IsNonFinite(sh.Move.X) || proofbound.IsNonFinite(sh.Move.Y) || proofbound.IsNonFinite(sh.Move.Z) ||
		g.Surface == nil || !g.Retained || g.Denoted != nil || g.Source == nil || proofbound.IsNonFinite(g.NormalBound) {
		return facetedrecord.Group{}, errBrakeEvalUnsupportedReplayMovedGroup
	}
	switch g.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder:
	default:
		return facetedrecord.Group{}, errBrakeEvalReplayMovedTagTypeUnavailable
	}
	if err := b.SpendWork(ctx, 128); err != nil {
		return facetedrecord.Group{}, err
	}
	if err := b.ReserveRecords(ctx, 4); err != nil {
		return facetedrecord.Group{}, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(facetedrecord.Group{}))+512); err != nil {
		return facetedrecord.Group{}, err
	}
	delta, err := r3.Translation(sh.Move)
	if err != nil {
		return facetedrecord.Group{}, err
	}
	image, err := surfaceplace.TransformSurface(g.Surface, delta)
	if err != nil {
		return facetedrecord.Group{}, err
	}
	var add float64
	switch source := g.Surface.(type) {
	case surfacegeom.Plane:
		moved, ok := image.(surfacegeom.Plane)
		if !ok {
			return facetedrecord.Group{}, errBrakeEvalReplayPlaneImageDiffers
		}
		add, err = facetedrecord.ReplayPlacedBuiltPlaneDeparture(ctx, source, moved, delta, b)
	case surfacegeom.Cylinder:
		moved, ok := image.(surfacegeom.Cylinder)
		if !ok {
			return facetedrecord.Group{}, errBrakeEvalReplayCylinderImageDiffers
		}
		add, err = facetedrecord.ReplayTranslatedCylinderDeparture(ctx, source, moved, delta, verts, reach, b)
	default:
		return facetedrecord.Group{}, errBrakeEvalReplayMovedTagTypeUnavailable
	}
	if err != nil {
		return facetedrecord.Group{}, err
	}
	g.Surface = image
	g.NormalBound = min(2, proofbound.AbsSumUpper(g.NormalBound, add))
	g.CapRecord = g.CapRecord.Under(delta)
	if g.CapRecord == nil {
		g.AxialDelta, g.HasAxialDelta = 0, false
	}
	return g, nil
}
