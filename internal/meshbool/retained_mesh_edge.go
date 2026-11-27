package meshbool

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RetainedEdgeAtMeshEdge finds the unique retained straight topological edge
// that contains a mesh edge and joins its two source faces. m must be a
// verified operand tessellation and bm its prepared copy. A nil result means
// the correspondence is not proved; it does not classify a Boolean contact.
//
//nolint:nilnil // A nil edge and nil error mean that the correspondence was not proved.
func RetainedEdgeAtMeshEdge(ctx context.Context, m *topo.Mesh, bm *BoolMesh, facet, edge int,
	budget *proofbound.WorkBudget) (*topo.Edge, error) {
	if ctx == nil || budget == nil {
		return nil, fmt.Errorf("%w: edge comparison needs a context and work budget", decaderr.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if m == nil || bm == nil || !m.BoundaryOK || facet < 0 || facet >= len(bm.Tris) || edge < 0 || edge >= 3 ||
		facet >= len(m.Triangles) || facet >= len(m.Source) || m.Triangles[facet] != bm.Tris[facet] {
		return nil, nil
	}
	twin, ok := bm.TwinFacet(facet, edge)
	if !ok || twin < 0 || twin >= len(bm.Tris) || twin >= len(m.Triangles) || twin >= len(m.Source) ||
		m.Triangles[twin] != bm.Tris[twin] {
		return nil, nil
	}
	tri := bm.Tris[facet]
	u, v := tri[edge], tri[(edge+1)%3]
	reversed := false
	for k := range 3 {
		if bm.Tris[twin][k] == v && bm.Tris[twin][(k+1)%3] == u {
			reversed = true
		}
	}
	if !reversed {
		return nil, nil
	}
	if u < 0 || v < 0 || u >= len(m.Vertices) || v >= len(m.Vertices) || u >= len(bm.Verts) || v >= len(bm.Verts) ||
		m.Vertices[u] != bm.Verts[u] || m.Vertices[v] != bm.Verts[v] || m.Vertices[u] == m.Vertices[v] ||
		!proofbound.FiniteVec(m.Vertices[u]) || !proofbound.FiniteVec(m.Vertices[v]) {
		return nil, nil
	}
	first, second := m.Source[facet], m.Source[twin]
	if first == nil || second == nil || first == second {
		return nil, nil
	}
	var found *topo.Edge
	for _, l1 := range first.Loops {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if l1 == nil {
			return nil, nil
		}
		for _, ce1 := range l1.CoEdges {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			for _, l2 := range second.Loops {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				if l2 == nil {
					return nil, nil
				}
				for _, ce2 := range l2.CoEdges {
					if err := budget.Step(); err != nil {
						return nil, err
					}
					candidate := ce1.Edge
					if candidate == nil || candidate != ce2.Edge || !retainedContains(candidate, first, second, m.Vertices[u], m.Vertices[v]) {
						continue
					}
					if found != nil && found != candidate {
						return nil, nil
					}
					found = candidate
				}
			}
		}
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	return found, nil
}

func retainedContains(e *topo.Edge, first, second *topo.Face, u, v r3.Vec) bool {
	if !e.RetainedCurve || e.Start == nil || e.End == nil || len(e.Faces) != 2 ||
		(e.Faces[0] != first || e.Faces[1] != second) && (e.Faces[0] != second || e.Faces[1] != first) ||
		e.Start.Bound.Kind() != units.Length || e.End.Bound.Kind() != units.Length ||
		e.Start.Bound.Base() != 0 || e.End.Bound.Base() != 0 || e.Start.Position == e.End.Position ||
		!proofbound.FiniteVec(e.Start.Position) || !proofbound.FiniteVec(e.End.Position) {
		return false
	}
	if _, ok := e.Curve.(surfacegeom.Line3); !ok {
		return false
	}
	a, b := proof.XptOf(e.Start.Position), proof.XptOf(e.End.Position)
	direction := proof.Xsub(b, a)
	within := func(v r3.Vec) bool {
		p := proof.XptOf(v)
		cross := proof.Xcross(direction, proof.Xsub(p, a))
		return cross.X.Sign() == 0 && cross.Y.Sign() == 0 && cross.Z.Sign() == 0 && PointOnSegment3D(a, b, p)
	}
	return within(u) && within(v)
}
