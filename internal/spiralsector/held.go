package spiralsector

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// HeldCopy is one tool's held mesh for PlacedHeldShells. Linked is the
// caller's source check: for the first tool, that it carries its LoftFaces
// trim record; for each later tool, that it is a recorded rigid copy of that
// trim under exactly its move.
type HeldCopy struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Linked bool
}

// PlacedHeldShells bounds each copy's held shell against the exact recorded
// placement of the first held shell. The intrinsic trim-to-mesh displacement
// belongs to the source certificate and is not charged here.
func PlacedHeldShells(ctx context.Context, copies []HeldCopy, moves []r3.Transform) ([]Shell, error) {
	if len(copies) < 2 || len(copies) != len(moves) || moves[0] != r3.Identity() {
		return nil, fmt.Errorf("%w: spiral held copies lack their source placement", decaderr.ErrUnsupported)
	}
	first := copies[0]
	if !first.Linked || len(first.Verts) == 0 || len(first.Tris) == 0 {
		return nil, fmt.Errorf("%w: spiral held source is incomplete", decaderr.ErrUnsupported)
	}
	parts := make([]Shell, 0, len(copies))
	parts = append(parts, Shell{Verts: first.Verts, Tris: first.Tris,
		VertexBound: make([]float64, len(first.Verts))})
	for i := 1; i < len(copies); i++ {
		placed := copies[i]
		if !placed.Linked || len(placed.Verts) != len(first.Verts) || len(placed.Tris) != len(first.Tris) {
			return nil, fmt.Errorf("%w: spiral held copy %d lost source correspondence", decaderr.ErrUnsupported, i+1)
		}
		for j, tri := range placed.Tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if tri != first.Tris[j] {
				return nil, fmt.Errorf("%w: spiral held copy %d changed source facets", decaderr.ErrUnsupported, i+1)
			}
		}
		placementBound := make([]float64, len(placed.Verts))
		for j, v := range placed.Verts {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			exact := meshbool.ExactRigidImage(moves[i].Basis(), moves[i].Translation(), first.Verts[j])
			placementBound[j] = proofbound.Radius3D(
				proofbound.RatFloatUp(meshbool.CoordDistance(exact, v)))
			if proofbound.IsNonFinite(placementBound[j]) {
				return nil, fmt.Errorf("%w: spiral held copy %d has no finite placement bound", decaderr.ErrUnsupported, i+1)
			}
		}
		bound := facetproof.FacetBoundMax(placed.Tris, placementBound)
		if proofbound.IsNonFinite(bound) {
			return nil, fmt.Errorf("%w: spiral held copy %d has no finite shell bound", decaderr.ErrUnsupported, i+1)
		}
		parts = append(parts, Shell{Verts: placed.Verts, Tris: placed.Tris,
			VertexBound: placementBound, MeshBound: bound})
	}
	return parts, nil
}
