package prismtess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// rectangleFaceBoundsReplay composes the same per-face bounds without a map.
func rectangleFaceBoundsReplay(ctx context.Context, mesh *topo.Mesh, inputs []tessellation.PrismReplayFaceBound[*topo.Face],
	store []float64, section float64, budget *proofbound.ReplayBudget) error {
	if mesh == nil || budget == nil || len(mesh.Triangles) != len(mesh.Source) {
		return fmt.Errorf("%w: invalid prism face bounds", decaderr.ErrUnsupported)
	}
	faceStore, err := topo.NewFaceBoundTable(ctx, len(mesh.Source), budget)
	if err != nil {
		return err
	}
	for i, face := range mesh.Source {
		previous, _, err := faceStore.Lookup(ctx, face, budget)
		if err != nil {
			return err
		}
		for _, v := range mesh.Triangles[i] {
			if v < 0 || v >= len(store) {
				return fmt.Errorf("%w: invalid prism vertex", decaderr.ErrBooleanFailed)
			}
			previous = math.Max(previous, store[v])
		}
		if err := faceStore.Set(ctx, face, previous, budget); err != nil {
			return err
		}
	}
	out, err := topo.NewFaceBoundTable(ctx, faceStore.Len(), budget)
	if err != nil {
		return err
	}
	err = faceStore.Each(ctx, budget, func(face *topo.Face, s float64) error {
		trim, axial := 0.0, 0.0
		for _, input := range inputs {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if input.Face == face {
				trim, axial = input.Trim, input.Axial
				break
			}
		}
		bound := proofbound.UpRound(trim + s + section + axial)
		if proofbound.IsNonFinite(bound) {
			return fmt.Errorf("%w: nonfinite prism face bound", decaderr.ErrUnsupported)
		}
		if err := out.Set(ctx, face, bound, budget); err != nil {
			return err
		}
		mesh.Bound = max(mesh.Bound, bound)
		return nil
	})
	if err != nil {
		return err
	}
	mesh.ReplayFaceBounds = out
	return nil
}

func rectangleCapAreasReplay(ctx context.Context, mesh *topo.Mesh, store []float64, start, end *topo.Face,
	budget *proofbound.ReplayBudget) ([2]float64, error) {
	var out [2]float64
	if mesh == nil || budget == nil || len(mesh.Source) != len(mesh.Triangles) {
		return out, fmt.Errorf("%w: invalid prism cap areas", decaderr.ErrUnsupported)
	}
	for i, tri := range mesh.Triangles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return out, err
		}
		if mesh.Source[i] != start && mesh.Source[i] != end {
			continue
		}
		for _, v := range tri {
			if v < 0 || v >= len(mesh.Vertices) || v >= len(store) {
				return out, fmt.Errorf("%w: invalid prism cap vertex", decaderr.ErrBooleanFailed)
			}
		}
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		a, b, c := mesh.Vertices[tri[0]], mesh.Vertices[tri[1]], mesh.Vertices[tri[2]]
		held := b.Sub(a).Cross(c.Sub(a)).Len() / 2
		for j, face := range [2]*topo.Face{start, end} {
			if mesh.Source[i] == face {
				out[j] = proofbound.AbsSumUpper(out[j], held, proofbound.FacetAreaTermSlop(a, b, c), proofbound.PerturbedTriangleAreaAllow(a, b, c, d))
			}
		}
	}
	return out, nil
}
