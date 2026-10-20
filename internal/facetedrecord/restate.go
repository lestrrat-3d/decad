package facetedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
)

// boundError is Restate's refusal of a chord tolerance finer than the bound
// the faceted body holds (agent-docs/tessellation-design.md §7). Only a Tessellate
// caller, who chose the tolerance, reaches it: the boolean asks a restating
// operand at its held floor and gates the facets the pair touches instead
// (booloperand.HeldFloor, meshbool.RefuseCoarseHeldContact).
type boundError struct{ requested, held float64 }

func (e *boundError) Error() string {
	requested, minimum := units.Millimeters(e.requested), units.Millimeters(e.held)
	return fmt.Sprintf(`%v: requested tolerance %s is below the faceted body's minimum mesh bound %s; retry with a tolerance of at least %s to restate the held mesh`, decaderr.ErrUnsupported, requested, minimum, minimum)
}

func (e *boundError) Unwrap() error { return decaderr.ErrUnsupported }

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
		return nil, &boundError{requested: chord, held: fp.MeshBound}
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
