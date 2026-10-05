package decad

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
)

// This file admits a proven exact planar relation to a contact manifold
// (docs/multibody-dynamics-design.md §9.3) and maps the kernel's face ids
// back to the bodies' live topology. internal/pair builds the exact points;
// contact_faceted_patch.go turns them into bounded public witnesses.

// publishPlanarManifold computes the manifold of a Touching pair, or the
// shallow-penetration patch of an Overlapping convex pair, and publishes it
// on report. A withheld manifold leaves report.Manifold nil with its reason:
// AmbiguousFeature for a contact set outside §9.3's table, PointTooCoarse
// or NoNormalProof for a witness or normal over the request. A penetration
// that the patch cannot certify keeps the reason report already carries.
func publishPlanarManifold(budget *workBudget, report *ContactReport, a, b *pair.PlanarSolid,
	result pair.PlanarResult, convexA, convexB bool) error {
	var points []pair.PatchPoint
	switch result.Relation {
	case pair.Touching:
		manifold, err := pair.PlanarTouchManifold(a, b, result.Contacts, convexA, convexB, budget.step)
		if err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = sourceBoxReason(manifold.Reason)
			return nil
		}
		points = manifold.Points
	case pair.Overlapping:
		if !convexA || !convexB {
			return nil
		}
		var err error
		points, err = pair.PlanarPenetrationManifold(a, b, budget.step)
		if err != nil || points == nil {
			return err
		}
	default:
		return nil
	}
	features, err := newPlanarFeatureMap(report.A, report.B, a, b)
	if err != nil {
		return err
	}
	manifold, reason := planarPatchManifold(report.Request, points, features)
	if manifold == nil {
		report.Reason = reason
		return nil
	}
	report.Manifold, report.Reason = manifold, ContactNoReason
	return nil
}

// planarFeatureMap resolves kernel face ids to the two bodies' live
// topology, and orders features by that topology (§9.5).
type planarFeatureMap struct {
	sides [2]planarTopology
}

type planarTopology struct {
	faces    []*Face
	faceAt   map[*Face]int
	edgeAt   map[*Edge]int
	vertexAt map[*Vertex]int
}

func newPlanarFeatureMap(a, b *Body, sa, sb *pair.PlanarSolid) (*planarFeatureMap, error) {
	m := &planarFeatureMap{}
	for i, side := range []struct {
		body  *Body
		solid *pair.PlanarSolid
	}{{a, sa}, {b, sb}} {
		topology := planarTopology{faces: side.body.Faces(), faceAt: make(map[*Face]int),
			edgeAt: make(map[*Edge]int), vertexAt: make(map[*Vertex]int)}
		for _, id := range side.solid.Faces {
			if id < 0 || id >= len(topology.faces) {
				return nil, fmt.Errorf("%w: planar snapshot names face %d of a body with %d faces",
					ErrDegenerate, id, len(topology.faces))
			}
		}
		for at, face := range topology.faces {
			topology.faceAt[face] = at
		}
		for at, edge := range side.body.Edges() {
			topology.edgeAt[edge] = at
		}
		for at, vertex := range side.body.Vertices() {
			topology.vertexAt[vertex] = at
		}
		m.sides[i] = topology
	}
	return m, nil
}

// feature resolves one side's kernel feature. A facet is its face; an edge is
// the one live Edge both of its faces share; a vertex is the one live Vertex
// every one of its faces holds. Anything else has no single source identity.
func (m *planarFeatureMap) feature(side int, f pair.PatchFeature) (ContactFeature, bool) {
	topology := &m.sides[side]
	if len(f.Faces) == 0 {
		return ContactFeature{}, false
	}
	faces := make([]*Face, len(f.Faces))
	for i, id := range f.Faces {
		faces[i] = topology.faces[id]
	}
	switch f.Kind {
	case pair.FeatureFacet:
		if len(faces) != 1 {
			return ContactFeature{}, false
		}
		return ContactFeature{Face: faces[0]}, true
	case pair.FeatureEdge:
		if len(faces) != 2 {
			return ContactFeature{}, false
		}
		var found *Edge
		for _, edge := range faces[0].Edges() {
			if edge == found || !slices.Contains(edge.Faces(), faces[1]) {
				continue
			}
			if found != nil {
				return ContactFeature{}, false
			}
			found = edge
		}
		return ContactFeature{Edge: found}, found != nil
	case pair.FeatureVertex:
		var found *Vertex
		for _, edge := range faces[0].Edges() {
			for _, vertex := range []*Vertex{edge.Start(), edge.End()} {
				if vertex == nil || vertex == found || !vertexOnFaces(vertex, faces[1:]) {
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

// order is the feature's rank in its body's topology: faces, then edges, then
// vertices, each in the body's own order.
func (m *planarFeatureMap) order(side int, f ContactFeature) [2]int {
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

func vertexOnFaces(vertex *Vertex, faces []*Face) bool {
	for _, face := range faces {
		held := false
		for _, edge := range face.Edges() {
			if edge.Start() == vertex || edge.End() == vertex {
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
