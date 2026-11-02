package facetproof

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// FaceProofsReplay accumulates the two per-face bounds in fixed tables.
func FaceProofsReplay(ctx context.Context, b *proofbound.ReplayBudget, verts []r3.Vec, tris [][3]int, faces []*topo.Face, deltas []float64) (*topo.FaceBoundTable, *topo.FaceBoundTable, error) {
	if b == nil || len(tris) != len(faces) || len(tris) != len(deltas) {
		return nil, nil, errBrakeReplayInvalidReplayFaceProofs
	}
	delta, err := topo.NewFaceBoundTable(ctx, len(faces), b)
	if err != nil {
		return nil, nil, err
	}
	area, err := topo.NewFaceBoundTable(ctx, len(faces), b)
	if err != nil {
		return nil, nil, err
	}
	for i, f := range faces {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, nil, err
		}
		t := tris[i]
		for _, v := range t {
			if v < 0 || v >= len(verts) {
				return nil, nil, errBrakeReplayInvalidReplayFaceIndex
			}
		}
		prior, _, err := delta.Lookup(ctx, f, b)
		if err != nil {
			return nil, nil, err
		}
		if err := delta.Set(ctx, f, max(prior, deltas[i]), b); err != nil {
			return nil, nil, err
		}
		prior, _, err = area.Lookup(ctx, f, b)
		if err != nil {
			return nil, nil, err
		}
		value := proofbound.AbsSumUpper(prior, proofbound.PerturbedTriangleAreaAllow(verts[t[0]], verts[t[1]], verts[t[2]], deltas[i]))
		if err := area.Set(ctx, f, value, b); err != nil {
			return nil, nil, err
		}
	}
	return delta, area, nil
}
