package surfacegroup

import "github.com/lestrrat-3d/decad/internal/proofbound"

// LinkEdge records one edge's endpoints and its face uses for a vertex-link audit.
type LinkEdge[F comparable, V comparable] struct {
	Start, End     V
	Faces          []F
	EndpointsValid bool
}

// LinkViolation identifies the first failed vertex-link condition.
type LinkViolation uint8

const (
	LinkValid LinkViolation = iota
	LinkIncompleteEdge
	LinkDegree
	LinkShape
	LinkDisconnected
)

// AuditVertexLinks checks that every vertex's incident faces form one path
// or cycle, with the lone-face self-closing rim as a valid degenerate link.
func AuditVertexLinks[E comparable, F comparable, V comparable](
	budget *proofbound.WorkBudget, edges map[E]LinkEdge[F, V],
) (LinkViolation, error) {
	type vertexLink struct {
		faces     map[F]struct{}
		degree    map[F]int
		neighbors map[F]map[F]struct{}
	}
	links := make(map[V]*vertexLink)
	linkFor := func(vertex V) *vertexLink {
		link := links[vertex]
		if link == nil {
			link = &vertexLink{
				faces: make(map[F]struct{}), degree: make(map[F]int),
				neighbors: make(map[F]map[F]struct{}),
			}
			links[vertex] = link
		}
		return link
	}
	for _, edge := range edges {
		if !edge.EndpointsValid || len(edge.Faces) < 1 || len(edge.Faces) > 2 {
			return LinkIncompleteEdge, nil
		}
		for _, vertex := range [...]V{edge.Start, edge.End} {
			if err := budget.Step(); err != nil {
				return LinkValid, err
			}
			link := linkFor(vertex)
			for _, face := range edge.Faces {
				link.faces[face] = struct{}{}
			}
			if len(edge.Faces) != 2 {
				continue
			}
			a, b := edge.Faces[0], edge.Faces[1]
			link.degree[a]++
			link.degree[b]++
			if link.neighbors[a] == nil {
				link.neighbors[a] = make(map[F]struct{})
			}
			if link.neighbors[b] == nil {
				link.neighbors[b] = make(map[F]struct{})
			}
			link.neighbors[a][b] = struct{}{}
			link.neighbors[b][a] = struct{}{}
		}
	}
	for _, link := range links {
		if err := budget.Step(); err != nil {
			return LinkValid, err
		}
		ends := 0
		for face := range link.faces {
			switch degree := link.degree[face]; {
			case degree == 1:
				ends++
			case degree == 2:
			case degree == 0 && len(link.faces) == 1:
				// One full-circle rim uses the same vertex at both ends.
			default:
				return LinkDegree, nil
			}
		}
		if len(link.faces) > 1 && ends != 0 && ends != 2 {
			return LinkShape, nil
		}
		var seed F
		for face := range link.faces {
			seed = face
			break
		}
		seen := map[F]struct{}{seed: {}}
		stack := []F{seed}
		for len(stack) != 0 {
			last := len(stack) - 1
			face := stack[last]
			stack = stack[:last]
			for neighbor := range link.neighbors[face] {
				if _, ok := seen[neighbor]; ok {
					continue
				}
				seen[neighbor] = struct{}{}
				stack = append(stack, neighbor)
			}
		}
		if len(seen) != len(link.faces) {
			return LinkDisconnected, nil
		}
	}
	return LinkValid, nil
}
