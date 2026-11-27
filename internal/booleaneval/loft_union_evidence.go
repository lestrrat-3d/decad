package booleaneval

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type loftUnionSourceKey struct{}

// loftUnionSource is supplied only by the public operation-history adapter.
// It authenticates that joined is the unshifted Union of loft and prism.
type loftUnionSource struct{ joined, loft, prism *topo.Body }

// WithLoftUnionSource carries an authenticated Union's original operands to
// the mesh evaluator. The caller must authenticate that history relation.
func WithLoftUnionSource(ctx context.Context, joined, loft, prism *topo.Body) context.Context {
	return context.WithValue(ctx, loftUnionSourceKey{}, loftUnionSource{joined, loft, prism})
}

//nolint:nilnil // A nil binding means that this optional source proof does not apply.
func prepareLoftUnionBinding(ctx context.Context, source loftUnionSource, joined *topo.Body,
	mesh *topo.Mesh, prepared *meshbool.BoolMesh, meshes OperandMesher, chord float64) (*loftUnionFacets, error) {
	if source.joined != joined || source.loft == nil || source.prism == nil || meshes == nil {
		return nil, nil
	}
	separated, err := separatedLoftPrismSections(ctx, source.loft, source.prism, proofbound.NewStrictReplayBudget())
	if err != nil || !separated {
		return nil, err
	}
	original, _, err := meshes.TessellateOperand(ctx, source.loft, chord)
	if err != nil {
		return nil, err
	}
	oldBool, err := meshbool.PrepContext(ctx, original.Vertices, original.Triangles, make([]int, len(original.Triangles)))
	if err != nil {
		return nil, err
	}
	return bindLoftUnionFacets(source.loft, original, oldBool, mesh, prepared,
		proofbound.NewStrictReplayBudget().WorkBudget(ctx))
}

type mappedLoftEdgeCertifier struct {
	binding *loftUnionFacets
	source  meshbool.SharedEdgeCertifier
}

func (c *mappedLoftEdgeCertifier) CertifySharedEdge(ctx context.Context, i, k, j, l int) (bool, error) {
	original, ok := c.binding.originalEdge(j, l)
	if !ok {
		return false, nil
	}
	matched, err := c.source.CertifySharedEdge(ctx, i, k, original, l)
	if err != nil && ctx.Err() == nil && errors.Is(err, decaderr.ErrUnsupported) {
		return false, nil
	}
	return matched, err
}

func loftUnionCapEvidence(ctx context.Context, binding *loftUnionFacets, target *topo.Mesh, targetBool *meshbool.BoolMesh,
	budget *proofbound.WorkBudget) func(*topo.Face, *topo.Face) (bool, error) {
	lp, ok := binding.loftBody.Payload.(loftrecord.Payload)
	if !ok {
		return nil
	}
	start, _, ok := lp.DirectCapFacetRange(binding.loftBody, 0)
	if !ok || start < 0 || start >= len(binding.original.Source) {
		return nil
	}
	capFace := binding.original.Source[start]
	source := directLoftCapContactEvidence(ctx, target, targetBool,
		binding.loftBody, binding.original, binding.oldBool)
	if source == nil {
		return nil
	}
	if err := budget.ReserveBytesMul(uint64(len(binding.joined.Source)), 64); err != nil {
		return nil
	}
	caps := make(map[*topo.Face]bool)
	return func(targetFace, joinedFace *topo.Face) (bool, error) {
		matched, cached := caps[joinedFace]
		if !cached {
			var err error
			matched, err = binding.startCapFace(joinedFace, budget)
			if err != nil {
				if ctx.Err() == nil && errors.Is(err, decaderr.ErrUnsupported) {
					return false, nil
				}
				return false, err
			}
			caps[joinedFace] = matched
		}
		if !matched {
			return false, nil
		}
		matched, err := source(targetFace, capFace)
		if err != nil && ctx.Err() == nil && errors.Is(err, decaderr.ErrUnsupported) {
			return false, nil
		}
		return matched, err
	}
}
