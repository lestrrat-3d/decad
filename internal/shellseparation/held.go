package shellseparation

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// HeldCopy is one tool's held mesh. Linked is the caller's provenance check:
// the first entry is the current source, and each later entry is its recorded
// rigid copy under exactly its move.
type HeldCopy struct {
	Verts  []r3.Vec
	Tris   [][3]int
	Linked bool
}

// PlacedHeldShells bounds each copy's held shell against the exact recorded
// placement of the first held shell. The intrinsic trim-to-mesh displacement
// belongs to the source certificate and is not charged here.
func PlacedHeldShells(ctx context.Context, copies []HeldCopy, moves []r3.Transform) ([]Shell, error) {
	if ctx == nil || len(copies) < 2 || len(copies) != len(moves) || moves[0] != r3.Identity() {
		return nil, fmt.Errorf("%w: held copies lack their source placement", decaderr.ErrUnsupported)
	}
	first := copies[0]
	if !first.Linked || len(first.Verts) == 0 || len(first.Tris) == 0 {
		return nil, fmt.Errorf("%w: held source is incomplete", decaderr.ErrUnsupported)
	}
	source := Shell{Verts: first.Verts, Tris: first.Tris,
		VertexBound: make([]float64, len(first.Verts))}
	if !validShell(source) {
		return nil, fmt.Errorf("%w: held source has invalid geometry", decaderr.ErrUnsupported)
	}
	parts := make([]Shell, 0, len(copies))
	parts = append(parts, source)
	for i := 1; i < len(copies); i++ {
		placed := copies[i]
		if !moves[i].IsValid() || !placed.Linked ||
			len(placed.Verts) != len(first.Verts) || len(placed.Tris) != len(first.Tris) {
			return nil, fmt.Errorf("%w: held copy %d lost source correspondence", decaderr.ErrUnsupported, i+1)
		}
		for j, tri := range placed.Tris {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			want := first.Tris[j]
			if moves[i].IsReflection() {
				want[1], want[2] = want[2], want[1]
			}
			if tri != want {
				return nil, fmt.Errorf("%w: held copy %d changed source facets", decaderr.ErrUnsupported, i+1)
			}
		}
		placementBound := make([]float64, len(placed.Verts))
		for j, v := range placed.Verts {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			if !proofbound.FiniteVec(v) {
				return nil, fmt.Errorf("%w: held copy %d has a nonfinite vertex", decaderr.ErrUnsupported, i+1)
			}
			exact := meshbool.ExactRigidImage(moves[i].Basis(), moves[i].Translation(), first.Verts[j])
			placementBound[j] = proofbound.Radius3D(
				proofbound.RatFloatUp(meshbool.CoordDistance(exact, v)))
			if proofbound.IsNonFinite(placementBound[j]) {
				return nil, fmt.Errorf("%w: held copy %d has no finite placement bound", decaderr.ErrUnsupported, i+1)
			}
		}
		bound := facetproof.FacetBoundMax(placed.Tris, placementBound)
		if proofbound.IsNonFinite(bound) {
			return nil, fmt.Errorf("%w: held copy %d has no finite shell bound", decaderr.ErrUnsupported, i+1)
		}
		parts = append(parts, Shell{Verts: placed.Verts, Tris: placed.Tris,
			VertexBound: placementBound, MeshBound: bound})
		if !validShell(parts[len(parts)-1]) {
			return nil, fmt.Errorf("%w: held copy %d has invalid bounds", decaderr.ErrUnsupported, i+1)
		}
	}
	return parts, nil
}
