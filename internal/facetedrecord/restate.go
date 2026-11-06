package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// Restate restates body's held mesh, whose payload is fp, as a mesh node: the
// polygons ARE the boundary this evaluator holds (core §6.1), carrying their
// own proven bound. It cannot refine them — the analytic identity is gone — so
// a chord tolerance finer than the held bound is ErrUnsupported, never a mesh
// whose bound overstates its trust.
func Restate(ctx context.Context, body *topo.Body, fp Payload, chord float64) (*topo.Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if chord < fp.MeshBound {
		if sigma := fp.SurfaceBounds(); sigma != nil {
			return restateSurface(ctx, body, fp, sigma, chord)
		}
		return nil, decaderr.NewMeshBoundError(chord, fp.MeshBound)
	}
	src, err := facetproof.RestateSources(ctx, topobuild.Faces(body), fp.FaceOf,
		len(fp.Verts), len(fp.VertexBound), len(fp.Tris))
	if err != nil {
		return nil, err
	}
	m := &topo.Mesh{
		Vertices:   fp.Verts,
		Triangles:  fp.Tris,
		AreaSlack:  fp.AreaSlack,
		VolSymDiff: fp.VolSymDiff,
		SymDiffOK:  true,
		Source:     src,
	}
	// The payload's own per-vertex record is restated unchanged, and each
	// face's bound is the largest δ(t) over its own facets
	// (agent-docs/faceted-vertex-bounds-design.md §4.4), so the next boolean's
	// pre-pass and rim composition read each face's and each facet's own
	// figure. Their maximum is the payload's MeshBound by construction.
	topobuild.SetVertexBounds(m, fp.VertexBound)
	if m.Bound != fp.MeshBound {
		return nil, fmt.Errorf(`%w: a faceted payload's mesh bound is not the largest of its facet bounds`, decaderr.ErrBooleanFailed)
	}
	return m, nil
}

// restateSurface restates a refined payload's held mesh under its surface
// bounds σ(f) (agent-docs/body-refine-design.md §3), for a chord below the
// crossing record's floor: every face's bound is the largest σ over its own
// facets, and the mesh publishes no per-vertex record, because σ is a
// facet-level claim. Requests at or above that floor, which every Boolean
// operand request is (booloperand.HeldFloor), keep the crossing restatement,
// so the choice is a function of the tolerance alone and the one-entry cache
// key, which carries the tolerance, never mixes the two.
func restateSurface(ctx context.Context, body *topo.Body, fp Payload, sigma []float64, chord float64) (*topo.Mesh, error) {
	held := 0.0
	for _, s := range sigma {
		held = max(held, s)
	}
	if chord < held {
		return nil, decaderr.NewMeshBoundError(chord, held)
	}
	src, err := facetproof.RestateSources(ctx, topobuild.Faces(body), fp.FaceOf,
		len(fp.Verts), len(fp.VertexBound), len(fp.Tris))
	if err != nil {
		return nil, err
	}
	m := &topo.Mesh{
		Vertices:   fp.Verts,
		Triangles:  fp.Tris,
		AreaSlack:  fp.AreaSlack,
		VolSymDiff: fp.VolSymDiff,
		SymDiffOK:  true,
		Source:     src,
	}
	for t := range m.Triangles {
		topobuild.SetFaceBound(m, m.Source[t], sigma[t])
	}
	if m.Bound != held {
		return nil, fmt.Errorf(`%w: a refined payload's mesh bound is not the largest of its surface bounds`, decaderr.ErrBooleanFailed)
	}
	return m, nil
}
