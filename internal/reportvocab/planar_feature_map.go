package reportvocab

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// PlanarFeatureMap resolves kernel face ids to the two bodies' live topology
// and orders features by that topology (multibody §9.5).
type PlanarFeatureMap struct {
	sides [2]planarTopology
}

type planarTopology struct {
	faces    []*topo.Face
	faceAt   map[*topo.Face]int
	edgeAt   map[*topo.Edge]int
	vertexAt map[*topo.Vertex]int
}

// NewPlanarFeatureMap validates the snapshot face ids against the live bodies.
func NewPlanarFeatureMap(a, b *topo.Body, sa, sb *planar.PlanarSolid) (*PlanarFeatureMap, error) {
	m := &PlanarFeatureMap{}
	for i, side := range []struct {
		body  *topo.Body
		solid *planar.PlanarSolid
	}{{a, sa}, {b, sb}} {
		topology := planarTopology{faces: topobuild.Faces(side.body), faceAt: make(map[*topo.Face]int),
			edgeAt: make(map[*topo.Edge]int), vertexAt: make(map[*topo.Vertex]int)}
		for _, id := range side.solid.Faces {
			if id < 0 || id >= len(topology.faces) {
				return nil, fmt.Errorf("%w: planar snapshot names face %d of a body with %d faces",
					decaderr.ErrDegenerate, id, len(topology.faces))
			}
		}
		for at, face := range topology.faces {
			topology.faceAt[face] = at
		}
		for at, edge := range topobuild.Edges(side.body) {
			topology.edgeAt[edge] = at
		}
		for at, vertex := range topobuild.Vertices(side.body) {
			topology.vertexAt[vertex] = at
		}
		m.sides[i] = topology
	}
	return m, nil
}

// Feature resolves one side's kernel feature. A facet is its face; an edge is
// the one live edge both faces share; a vertex is the one live vertex every
// face holds. Anything else has no single source identity.
func (m *PlanarFeatureMap) Feature(side int, f planar.PatchFeature) (ContactFeature, bool) {
	topology := &m.sides[side]
	if len(f.Faces) == 0 {
		return ContactFeature{}, false
	}
	faces := make([]*topo.Face, len(f.Faces))
	for i, id := range f.Faces {
		faces[i] = topology.faces[id]
	}
	switch f.Kind {
	case planar.FeatureFacet:
		if len(faces) != 1 {
			return ContactFeature{}, false
		}
		return ContactFeature{Face: faces[0]}, true
	case planar.FeatureEdge:
		if len(faces) != 2 {
			return ContactFeature{}, false
		}
		var found *topo.Edge
		for _, edge := range planarFaceEdges(faces[0]) {
			if edge == found || !slices.Contains(edge.Faces, faces[1]) {
				continue
			}
			if found != nil {
				return ContactFeature{}, false
			}
			found = edge
		}
		return ContactFeature{Edge: found}, found != nil
	case planar.FeatureVertex:
		var found *topo.Vertex
		for _, edge := range planarFaceEdges(faces[0]) {
			for _, vertex := range []*topo.Vertex{edge.Start, edge.End} {
				if vertex == nil || vertex == found || !planarVertexOnFaces(vertex, faces[1:]) {
					continue
				}
				if found != nil {
					return ContactFeature{}, false
				}
				found = vertex
			}
		}
		return ContactFeature{Vertex: found}, found != nil
	}
	return ContactFeature{}, false
}

// Order ranks faces, edges, then vertices in their body's topology order.
func (m *PlanarFeatureMap) Order(side int, f ContactFeature) [2]int {
	topology := &m.sides[side]
	switch {
	case f.Face != nil:
		return [2]int{0, topology.faceAt[f.Face]}
	case f.Edge != nil:
		return [2]int{1, topology.edgeAt[f.Edge]}
	default:
		return [2]int{2, topology.vertexAt[f.Vertex]}
	}
}

func planarFaceEdges(face *topo.Face) []*topo.Edge {
	var edges []*topo.Edge
	for _, loop := range face.Loops {
		for _, use := range loop.CoEdges {
			edges = append(edges, use.Edge)
		}
	}
	return edges
}

func planarVertexOnFaces(vertex *topo.Vertex, faces []*topo.Face) bool {
	for _, face := range faces {
		held := false
		for _, edge := range planarFaceEdges(face) {
			if edge.Start == vertex || edge.End == vertex {
				held = true
				break
			}
		}
		if !held {
			return false
		}
	}
	return true
}
