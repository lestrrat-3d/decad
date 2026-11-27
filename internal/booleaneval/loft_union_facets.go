package booleaneval

import (
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// loftUnionFacets binds unchanged ordered triangles of a joined tool to the
// verified original loft mesh. A missing or duplicate source triangle is not
// evidence of a match.
type loftUnionFacets struct {
	loftBody *topo.Body
	original *topo.Mesh
	oldBool  *meshbool.BoolMesh
	joined   *topo.Mesh
	newBool  *meshbool.BoolMesh
	byPoints map[[3]r3.Vec]int
}

func bindLoftUnionFacets(loftBody *topo.Body, original *topo.Mesh, oldBool *meshbool.BoolMesh,
	joined *topo.Mesh, newBool *meshbool.BoolMesh, budget *proofbound.WorkBudget) (*loftUnionFacets, error) {
	if loftBody == nil || original == nil || oldBool == nil || joined == nil || newBool == nil || budget == nil ||
		!original.BoundaryOK || !original.SymDiffOK || !joined.BoundaryOK || !joined.SymDiffOK ||
		len(original.Triangles) != len(oldBool.Tris) ||
		len(joined.Triangles) != len(newBool.Tris) || len(original.Source) != len(original.Triangles) ||
		len(joined.Source) != len(joined.Triangles) {
		return nil, nil //nolint:nilnil // A nil map means the source correspondence is unavailable.
	}
	if err := budget.ReserveBytesMul(uint64(len(original.Triangles)), 256); err != nil {
		return nil, err
	}
	b := &loftUnionFacets{loftBody: loftBody, original: original, oldBool: oldBool, joined: joined,
		newBool: newBool, byPoints: make(map[[3]r3.Vec]int, len(original.Triangles))}
	for i, tri := range original.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if tri != oldBool.Tris[i] {
			return nil, nil //nolint:nilnil // A changed prepared mesh cannot use this source.
		}
		points, ok := meshFacetPoints(original, oldBool, i)
		if !ok {
			return nil, nil //nolint:nilnil // A changed or invalid vertex declines the proof.
		}
		if _, exists := b.byPoints[points]; exists {
			b.byPoints[points] = -1
		} else {
			b.byPoints[points] = i
		}
	}
	return b, budget.Err()
}

func meshFacetPoints(mesh *topo.Mesh, prepared *meshbool.BoolMesh, facet int) ([3]r3.Vec, bool) {
	var points [3]r3.Vec
	if facet < 0 || facet >= len(mesh.Triangles) || facet >= len(prepared.Tris) || mesh.Triangles[facet] != prepared.Tris[facet] {
		return points, false
	}
	for i, v := range mesh.Triangles[facet] {
		if v < 0 || v >= len(mesh.Vertices) || v >= len(prepared.Verts) || mesh.Vertices[v] != prepared.Verts[v] ||
			!proofbound.FiniteVec(mesh.Vertices[v]) {
			return points, false
		}
		points[i] = mesh.Vertices[v]
	}
	return points, true
}

func (b *loftUnionFacets) originalFacet(facet int) (int, bool) {
	points, ok := meshFacetPoints(b.joined, b.newBool, facet)
	if !ok {
		return 0, false
	}
	original, ok := b.byPoints[points]
	return original, ok && original >= 0
}

// startCapFace accepts a joined face only when all its held facets are
// authenticated original start-cap triangles.
func (b *loftUnionFacets) startCapFace(face *topo.Face, budget *proofbound.WorkBudget) (bool, error) {
	lp, ok := b.loftBody.Payload.(loftrecord.Payload)
	if !ok || face == nil || budget == nil {
		return false, nil
	}
	seen := false
	for i, source := range b.joined.Source {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if source != face {
			continue
		}
		original, ok := b.originalFacet(i)
		if !ok {
			return false, nil
		}
		points, ok := meshFacetPoints(b.joined, b.newBool, i)
		if !ok || !lp.DirectCapFacetMatches(b.loftBody, b.original, original, 0, points) {
			return false, nil
		}
		seen = true
	}
	return seen, budget.Err()
}

// originalEdge requires the joined facet and its twin to be the original
// adjacent facets with the same directed edge indices.
func (b *loftUnionFacets) originalEdge(facet, edge int) (int, bool) {
	if edge < 0 || edge >= 3 {
		return 0, false
	}
	original, ok := b.originalFacet(facet)
	if !ok {
		return 0, false
	}
	joinedTwin, ok := b.newBool.TwinFacet(facet, edge)
	if !ok {
		return 0, false
	}
	originalTwin, ok := b.originalFacet(joinedTwin)
	if !ok {
		return 0, false
	}
	twin, ok := b.oldBool.TwinFacet(original, edge)
	return original, ok && twin == originalTwin
}
